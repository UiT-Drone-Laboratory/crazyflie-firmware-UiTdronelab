/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include "math.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();  // motors.c (drivers)

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_tv2 *control)
{
  #ifdef QUAD_FORMATION_X
    double r = control->roll / 4.0;
    double p = control->pitch / 4.0;
   /* uint16_t M1;

    M1 = limitThrust((uint32_t)((0.0962*sqrt(416*(control->thrust-r+p-control->yaw) +49)-0.6731)*65535)) ;*/
    // I used the model described in Greiff thesis {T = 0.26*d^2 + 0.35*d}
    // d = duty cycle

    motorPower.m1 = limitThrust((uint32_t)((0.0962*sqrt(416*(control->thrust-r+p-control->yaw) +49)-0.6731)*65535));
    motorPower.m2 = limitThrust((uint32_t)((0.0962*sqrt(416*(control->thrust-r-p+control->yaw) +49)-0.6731)*65536));
    motorPower.m3 = limitThrust((uint32_t)((0.0962*sqrt(416*(control->thrust+r-p-control->yaw) +49)-0.6731)*65536));
    motorPower.m4 = limitThrust((uint32_t)((0.0962*sqrt(416*(control->thrust+r+p+control->yaw) +49)-0.6731)*65536));


    /*motorPower.m1 = limitThrust(control->thrust - r + p - control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p + control->yaw);
    motorPower.m3 = limitThrust(control->thrust + r - p - control->yaw);
    motorPower.m4 = limitThrust(control->thrust + r + p + control->yaw);*/

  //  DEBUG_PRINT("m1 = %i \n", (uint16_t)motorPower.m1);

    /*double beta2 = 9.78e-8,beta1 = -1.97e-7,beta0 = 1.51e-3,alpha2 = 0.26,alpha1 = 0.35,k = 2.2e-8,k_Q = 1e-9,l = 0.046;
    double a = 1/(4*k),b = 1/(4*k*l/sqrt(2)),c = 1/(4*k_Q);
    double T = control->thrust, taux = control->roll,tauy = control->pitch,tauz = control->yaw;
    double d1 = (-alpha1/(2*alpha2) + 1/(2*alpha2)*sqrt(alpha1*alpha1+4*alpha2*(beta2*(a*T-b*(taux+tauy)-c*tauz)+beta1*sqrt(a*T-b*(taux+tauy)-c*tauz)+beta0)));
    double d2 = (-alpha1/(2*alpha2) + 1/(2*alpha2)*sqrt(alpha1*alpha1+4*alpha2*(beta2*(a*T-b*(taux-tauy)+c*tauz)+beta1*sqrt(a*T-b*(taux-tauy)+c*tauz)+beta0)));
    double d3 = (-alpha1/(2*alpha2) + 1/(2*alpha2)*sqrt(alpha1*alpha1+4*alpha2*(beta2*(a*T+b*(taux+tauy)-c*tauz)+beta1*sqrt(a*T+b*(taux+tauy)-c*tauz)+beta0)));
    double d4 = (-alpha1/(2*alpha2) + 1/(2*alpha2)*sqrt(alpha1*alpha1+4*alpha2*(beta2*(a*T+b*(taux-tauy)+c*tauz)+beta1*sqrt(a*T+b*(taux-tauy)+c*tauz)+beta0)));

    motorPower.m1 = d1 > 1 ? 65536 : d1*65536;
    motorPower.m2 = d2 > 1 ? 65536 : d2*65536;
    motorPower.m3 = d3 > 1 ? 65536 : d3*65536;
    motorPower.m4 = d4 > 1 ? 65536 : d4*65536;*/

    //DEBUG_PRINT("d1 = %1.3f, d2 = %1.3f\n",d1, d2);

  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll  - control->yaw);
    motorPower.m3 = limitThrust(control->thrust - control->pitch + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + control->roll  - control->yaw);


  #endif

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {

    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);

/*    motorsSetRatio(MOTOR_M1, 0*motorPower.m1);
        motorsSetRatio(MOTOR_M2, 0*motorPower.m2);
        motorsSetRatio(MOTOR_M3, 0*motorPower.m3);
        motorsSetRatio(MOTOR_M4, 0*motorPower.m4);*/


  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)
