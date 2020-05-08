/*
 * led_sequence.c
 *
 *  Created on: Oct 4, 2019
 *      Author: bitcraze
 */
#include <stdbool.h>

#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
//#include "led_sequence.h"

void led_sequence (void);

void led_sequence (void)
{

	DEBUG_PRINT("Hello world Jose \n");

 /*xTaskCreate(led_sequenceRun,        // function in (never exit)
		     LEDSEQUENCE_TASK_NAME,  // name for the task
	         50,  // large of the stack
			 NULL,                   // parameters to the task function (only pointers)
	         0,        // priority
			 NULL);                  // handle task

*/
}

void led_sequenceRun (void)
{

	while(1)
	{
	ledSet(0, 1); //M3 only blue
	ledSet(2, 1); //M4 red
	ledSet(1, 0); //M4 green
	ledSet(4, 0); //M1 red
	ledSet(3, 1); //M1 green
	vTaskDelay(M2T(250));
	ledSet(0, 0); //M3 only blue
	ledSet(2, 1); //M4 red
	ledSet(1, 1); //M4 green
	ledSet(4, 1); //M1 red
	ledSet(3, 1); //M1 green
	vTaskDelay(M2T(250));
	}
}
