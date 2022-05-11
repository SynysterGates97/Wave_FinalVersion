/*
 * necomimi_queue.c
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */

#include "necomimi_queue.h"
#include "cmsis_os.h"

#define NECOMIMI_MUTEX_WAIT_TIME_MS 1000

osMutexDef(necomimiQueueMutex);
osMutexId necomimiQueueMutexId;

uint32_t necomimiQueueElementsInQueue;
uint32_t necomimiQueueBeginIndex;
uint32_t necomimiQueueEndIndex;

NecomimiPacketUnit necomimiQueue[NECOMIMI_QUEUE_SIZE];

bool necomimi_queue_init()
{
	necomimiQueueElementsInQueue = 0;
	necomimiQueueEndIndex = 0;
	necomimiQueueBeginIndex = 0;

	necomimiQueueMutexId = osMutexCreate(osMutex(necomimiQueueMutex));

	return necomimiQueueMutexId != NULL;
}
bool necomimi_queue_enque(NecomimiPacketUnit *necomimiPacket)
{
	osStatus waitResult = osMutexWait(necomimiQueueMutexId, NECOMIMI_MUTEX_WAIT_TIME_MS);
	if(waitResult == osOK)
	{


		osMutexRelease(necomimiQueueMutexId);
		return true;
	}

	osMutexRelease(necomimiQueueMutexId);
	return false;
}
// Это позволит не взаимодействовать с мьютексом снаружи: "NecomimiPacketUnit *outputNecomimiPacket"
bool necomimi_queue_deque(NecomimiPacketUnit *outputNecomimiPacket)
{
	osStatus waitResult = osMutexWait(necomimiQueueMutexId, NECOMIMI_MUTEX_WAIT_TIME_MS);
	if(waitResult == osOK)
	{
		if (necomimiQueueElementsInQueue > 0)
		{
			memcpy(outputNecomimiPacket, necomimiQueue,sizeof(necomimiQueue));
			necomimiQueueElementsInQueue--;

			if(necomimiQueueEndIndex > necomimiQueueBeginIndex)
			{
				necomimiQueueEndIndex--;
			}
			else if (necomimiQueueEndIndex < necomimiQueueBeginIndex)
			{

			}


		}

		osMutexRelease(necomimiQueueMutexId);
		return true;
	}

	osMutexRelease(necomimiQueueMutexId);
	return false;
}
