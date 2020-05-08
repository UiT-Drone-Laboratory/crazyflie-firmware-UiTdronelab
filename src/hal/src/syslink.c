/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * syslink.c: Communication between NRF51 and STM32
 */
#define DEBUG_MODULE "SL"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "config.h"
#include "debug.h"
#include "syslink.h"
#include "radiolink.h"
#include "uart_syslink.h"
#include "configblock.h"
#include "pm.h"
#include "ow.h"
#include "stabilizer.h"

static bool isInit = false;
static uint8_t sendBuffer[64];
//extern bool flag;

static void syslinkRouteIncommingPacket(SyslinkPacket *slp);

static xSemaphoreHandle syslinkAccess;

radio_t radio;

/* Syslink task, handles communication between nrf and stm and dispatch messages
 */
static void syslinkTask(void *param)
{
  SyslinkPacket slp;

  float Rps4, Pps4, Yps4;
  uint8_t Tps4;
  char cdata[4];

  while(1)
  {
    uartslkGetPacketBlocking(&slp);
    syslinkRouteIncommingPacket(&slp);

    //Radio information
    for(int i=1;i<=4;i++)
    	cdata[i-1] = slp.data[i];  //Roll
    memcpy(&Rps4,cdata,4);

    for(int i=5;i<=8;i++)
    	cdata[i-5] = slp.data[i]; //Pitch
    memcpy(&Pps4,cdata,4);

    for(int i=9;i<=12;i++)
    	cdata[i-9] = slp.data[i]; //Yaw
    memcpy(&Yps4,cdata,4);

    Tps4 = slp.data[14];

    if(Rps4 < 3.5f || Rps4 > 4.1f)
    	radio.Rps4 = Rps4;

    if(Pps4 < 0.0013f || Pps4>0.0015f)
    	radio.Pps4 = Pps4;

    radio.Yps4 = Yps4;
    radio.Tps4 = Tps4;

//    DEBUG_PRINT("%f \n", (double)radio.Rps4);
  }
}

static void syslinkRouteIncommingPacket(SyslinkPacket *slp)
{
  uint8_t groupType;

  groupType = slp->type & SYSLINK_GROUP_MASK;

  switch (groupType)
  {
    case SYSLINK_RADIO_GROUP:
      radiolinkSyslinkDispatch(slp);
      break;
    case SYSLINK_PM_GROUP:
      pmSyslinkUpdate(slp);
      break;
    case SYSLINK_OW_GROUP:
      owSyslinkRecieve(slp);
      break;
    default:
      DEBUG_PRINT("Unknown packet:%X.\n", slp->type);
      break;
  }
}

/*
 * Public functions
 */

void syslinkInit()
{
  if(isInit)
    return;

  vSemaphoreCreateBinary(syslinkAccess);

  if (xTaskCreate(syslinkTask, SYSLINK_TASK_NAME,
                  SYSLINK_TASK_STACKSIZE, NULL, SYSLINK_TASK_PRI, NULL) == pdPASS)
  {
    isInit = true;
  }
}

bool syslinkTest()
{
  return isInit;
}

int syslinkSendPacket(SyslinkPacket *slp)
{
  int i = 0;
  int dataSize;
  uint8_t cksum[2] = {0};

  xSemaphoreTake(syslinkAccess, portMAX_DELAY);

  ASSERT(slp->length <= SYSLINK_MTU);

  sendBuffer[0] = SYSLINK_START_BYTE1;
  sendBuffer[1] = SYSLINK_START_BYTE2;
  sendBuffer[2] = slp->type;
  sendBuffer[3] = slp->length;

  memcpy(&sendBuffer[4], slp->data, slp->length);
  dataSize = slp->length + 6;
  // Calculate checksum delux
  for (i = 2; i < dataSize - 2; i++)
  {
    cksum[0] += sendBuffer[i];
    cksum[1] += cksum[0];
  }
  sendBuffer[dataSize-2] = cksum[0];
  sendBuffer[dataSize-1] = cksum[1];

  uartslkSendDataDmaBlocking(dataSize, sendBuffer);

  xSemaphoreGive(syslinkAccess);

  return 0;
}
