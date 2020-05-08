/*
 * Code for the communication between crazyflie and PC to transmit sensor's data
 */

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"

#include "led.h"

#include "euler_control.h"
#include "quaternion_control.h"
#include "position_control.h"
#include "reactive_control.h"
#include "crtp.h"
//#include "crtp_localization_service.h"

#include "estimator_kalman.h"
#include "lpsTwrTag.h"
#include "mem.h"

#include "communication.h"

typedef enum {
	configureAcc,
	measureNoiseFloor,
	measureProp,
	testBattery,
	restartBatTest,
	evaluateResult,
	testDone
} TestState;

static bool isInit;
static double currentPos[3];
static double currentVel[3];
point_t pos1;
velocity_t vel1;
//Private function
static void communicationTask();
//static bool dataTransfer();
//STATIC_MEM_TASK_ALLOC(communicationTask, COMMUNICATION_TASK_STACKSIZE);

static bool startPropTest = false;

#ifdef RUN_PROP_TEST_AT_STARTUP
  static TestState testState = configureAcc;
#else
static TestState testState = testDone;
#endif


void communicationInit()
{
	 if (isInit)
	    return;

	 xTaskCreate(communicationTask, COMMUNICATION_TASK_NAME, COMMUNICATION_TASK_STACKSIZE, NULL, COMMUNICATION_TASK_PRI, NULL);

	//xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, STABILIZER_TASK_STACKSIZE,
				//NULL, STABILIZER_TASK_PRI, NULL);
	isInit = true;
}

void communicationTask()
{	uint32_t lastWakeTime;

	//wait for the system to start
	systemWaitStart();
	// Wait for sensors to be calibrated
		lastWakeTime = xTaskGetTickCount();
		while (!sensorsAreCalibrated()) {
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
		}
	//dataTransfer();
	while(1){
		sensorsWaitDataReady();
		if (startPropTest != false) {

					testState = configureAcc;
					startPropTest = false;
				}
		currentPos[0]=pos1.x;
		 currentPos[1]=pos1.y;
		 currentPos[2]=pos1.z;

		 currentVel[0]=vel1.x;
		 currentVel[1]=vel1.y;
		 currentVel[2]=vel1.z;
		 positionControl(currentPos, currentVel);

		DEBUG_PRINT("phid= %f \n",  ctrlpos.phid);
		DEBUG_PRINT("thetad= %f \n", ctrlpos.thetad);
		DEBUG_PRINT("thrust= %f \n", ctrlpos.thrust);
	}
}
/*
bool dataTransfer()
{
	 currentPos[0]=pos1.x;
	 currentPos[1]=pos1.y;
	 currentPos[2]=pos1.z;

	 currentVel[0]=vel1.x;
	 currentVel[1]=vel1.y;
	 currentVel[2]=vel1.z;
	 positionControl(currentPos, currentVel);
	DEBUG_PRINT("phid=%f \n",  ctrlpos.phid);
	DEBUG_PRINT("thetad=%f \n", ctrlpos.thetad);
	DEBUG_PRINT("thrust=%f \n", ctrlpos.thrust);
	return true;
}*/
