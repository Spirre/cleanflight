/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef GTUNE

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "config/config.h"
#include "blackbox/blackbox.h"

#include "io/rc_controls.h"

#include "config/runtime_config.h"

extern uint8_t motorCount;

/*
 ****************************************************************************
 ***                    G_Tune                                            ***
 ****************************************************************************
	G_Tune Mode
	This is the multiwii implementation of ZERO-PID Algorithm
	http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
	The algorithm has been originally developed by Mohammad Hefny (mohammad.hefny@gmail.com)

	You may use/modify this algorithm on your own risk, kindly refer to above link in any future distribution.
 */

/*
   version 1.0.0: MIN & MAX & Tuned Band
   version 1.0.1:
                a. error is gyro reading not rc - gyro.
                b. OldError = Error no averaging.
                c. No Min MAX BOUNDRY
    version 1.0.2:
                a. no boundaries
                b. I - Factor tune.
                c. time_skip

   Crashpilot: Reduced to just P tuning in a predefined range - so it is not "zero pid" anymore.
   Tuning is limited to just work when stick is centered besides that YAW is tuned in non Acro as well.
   See also:
   http://diydrones.com/profiles/blogs/zero-pid-tunes-for-multirotors-part-2
   http://www.multiwii.com/forum/viewtopic.php?f=8&t=5190
   Gyrosetting 2000DPS
   GyroScale = (1 / 16,4 ) * RADX(see board.h) = 0,001064225154 digit per rad/s

    pidProfile->gtune_lolimP[ROLL]   = 20; [10..200] Lower limit of ROLL P during G tune.
    pidProfile->gtune_lolimP[PITCH]  = 20; [10..200] Lower limit of PITCH P during G tune.
    pidProfile->gtune_lolimP[YAW]    = 20; [10..200] Lower limit of YAW P during G tune.
    pidProfile->gtune_hilimP[ROLL]   = 70; [0..200]  Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[PITCH]  = 70; [0..200]  Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[YAW]    = 70; [0..200]  Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_pwr            = 0;  [0..10] Strength of adjustment
*/

void calculate_Gtune(bool inirun, uint8_t ax, pidProfile_t *pidProfile)
{
    static  int8_t time_skip[3];
    static  int16_t OldError[3], result_P64[3];
    static  int32_t AvgGyro[3];
    int16_t error, diff_G, threshP;
    uint8_t i;

    if (inirun)
    {
        for (i = 0; i < 3; i++)
        {
            if ((pidProfile->gtune_hilimP[i] && pidProfile->gtune_lolimP[i] > pidProfile->gtune_hilimP[i]) ||    // User config error disable axis for tuning
               (motorCount < 4 && i == FD_YAW)) pidProfile->gtune_hilimP[i] = 0;        // Disable Yawtuning for everything below a quadcopter
            if(pidProfile->P8[i] < pidProfile->gtune_lolimP[i]) pidProfile->P8[i] = pidProfile->gtune_lolimP[i];
            result_P64[i] = (int16_t)pidProfile->P8[i] << 6;                            // 6 bit extra resolution for P.
            OldError[i]   = 0;
            time_skip[i]  = -125;
        }
    }
    else
    {
        if(rcCommand[ax] || (ax != FD_YAW && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))))    // Block Tuning on stickinput. Always allow Gtune on YAW, Roll & Pitch only in acromode
        {
            OldError[ax]  = 0;
            time_skip[ax] = -125;                                               // Some settletime after stick center. (125 + 16)* 3ms clycle = 423ms (ca.)
        }
        else
        {
            if (!time_skip[ax]) AvgGyro[ax] = 0;
            time_skip[ax]++;
            if (time_skip[ax] > 0)
            {
                if (ax == FD_YAW) AvgGyro[ax] += 32 * ((int16_t)gyroData[ax] / 32);// Chop some jitter and average
                else AvgGyro[ax] += 128 * ((int16_t)gyroData[ax] / 128);        // Chop some jitter and average
            }

            if (time_skip[ax] == 16)                                            // ca 48 ms
            {
                AvgGyro[ax] /= time_skip[ax];                                   // AvgGyro[ax] has now very clean gyrodata
                time_skip[ax] = 0;

                if (ax == FD_YAW)
                {
                    threshP = 20;
                    error   = -AvgGyro[ax];
                }
                else
                {
                    threshP = 10;
                    error   = AvgGyro[ax];
                }

                if (pidProfile->gtune_hilimP[ax] && error && OldError[ax] && error != OldError[ax]) // Don't run when not needed or pointless to do so
                {
                    diff_G = ABS(error) - ABS(OldError[ax]);
                    if ((error > 0 && OldError[ax] > 0) || (error < 0 && OldError[ax] < 0))
                    {
                        if (diff_G > threshP) result_P64[ax] += 64 + pidProfile->gtune_pwr;// Shift balance a little on the plus side.
                        else
                        {
                            if (diff_G < -threshP)
                            {
                                if (ax == FD_YAW) result_P64[ax] -= 64 + pidProfile->gtune_pwr;
                                else result_P64[ax] -= 32;
                            }
                        }
                    }
                    else
                    {
                        if (ABS(diff_G) > threshP && ax != FD_YAW) result_P64[ax] -= 32; // Don't use antiwobble for YAW
                    }
                    result_P64[ax] = constrain(result_P64[ax], (int16_t)pidProfile->gtune_lolimP[ax] << 6, (int16_t)pidProfile->gtune_hilimP[ax] << 6);
                    pidProfile->P8[ax] = result_P64[ax] >> 6;
                }
                OldError[ax] = error;
            }
        }
    }
}

#endif

