#ifndef SRC_MODULES_INTERFACE_REACTIVE_CONTROL_H_
#define SRC_MODULES_INTERFACE_REACTIVE_CONTROL_H_
#include "stabilizer.h"

void reactive_control(state_t *state, sensorData_t *sensor);
extern control_tv2 Econtrol;

#endif /* SRC_MODULES_INTERFACE_REACTIVE_CONTROL_H_ */
