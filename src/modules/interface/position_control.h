/*
 * position_control.h
 *
 *  Created on: Dec 12, 2019
 *      Author: commander
 */

#ifndef SRC_MODULES_INTERFACE_POSITION_CONTROL_H_
#define SRC_MODULES_INTERFACE_POSITION_CONTROL_H_

#include "stabilizer.h"

void positionControl(double posq[], double velq[]);
extern control_tpos ctrlpos;


#endif /* SRC_MODULES_INTERFACE_POSITION_CONTROL_H_ */
