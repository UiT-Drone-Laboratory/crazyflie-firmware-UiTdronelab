/*
 * euler_control.h
 *
 *  Created on: Oct 17, 2019
 *      Author: commander
 */

#ifndef __SRC_MODULES_INTERFACE_EULER_CONTROL_H__
#define __SRC_MODULES_INTERFACE_EULER_CONTROL_H__

#include "stabilizer.h"

void eulerControl(state_t *state, sensorData_t *sensor);

extern control_tv2 Econtrol;

#endif /* __SRC_MODULES_INTERFACE_EULER_CONTROL_H__ */
