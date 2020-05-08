/*
 * quaternion_control.h
 *
 *  Created on: Oct 21, 2019
 *      Author: commander
 */

#ifndef SRC_MODULES_INTERFACE_QUATERNION_CONTROL_H_
#define SRC_MODULES_INTERFACE_QUATERNION_CONTROL_H_

#include "stabilizer.h"

void quaternionControl(state_t *state, sensorData_t *sensor);

extern control_tv2 Econtrol;

#endif /* SRC_MODULES_INTERFACE_QUATERNION_CONTROL_H_ */
