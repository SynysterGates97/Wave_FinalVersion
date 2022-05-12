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

// По терминологии FIFO - это индекс вставки, вакантное место
uint32_t necomimiFirstInsertIndex;
uint32_t necomimiLastInsertIndex;

uint32_t necomimiQueueBeginIndex;
uint32_t necomimiQueueEndIndex;

NecomimiPacketUnit necomimiQueue[NECOMIMI_QUEUE_SIZE];

bool necomimi_queue_init()
{
	necomimiQueueElementsInQueue = 0;

	necomimiQueueEndIndex = 0;
	necomimiQueueInsertIndex = 0;

	necomimiQueueMutexId = osMutexCreate(osMutex(necomimiQueueMutex));

	return necomimiQueueMutexId != NULL;
}

bool necomimi_queue_enque(NecomimiPacketUnit *necomimiPacket)
{
	osStatus waitResult = osMutexWait(necomimiQueueMutexId, NECOMIMI_MUTEX_WAIT_TIME_MS);
	if(waitResult == osOK)
	{
		if (necomimiQueueElementsInQueue < NECOMIMI_QUEUE_SIZE)
		{
			memcpy(necomimiQueue[necomimiQueueInsertIndex], necomimiPacket, sizeof(NecomimiPacketUnit));
			necomimiQueueInsertIndex++;

			if (necomimiQueueInsertIndex > NECOMIMI_QUEUE_SIZE - 1)
			{
				necomimiQueueEndIndex = 0;
			}

			if (necomimiQueueInsertIndex == necomimiQueueBeginIndex)
			{
				necomimiQueueBeginIndex++;
				if (necomimiQueueBeginIndex > NECOMIMI_QUEUE_SIZE - 1)
				{
					necomimiQueueBeginIndex = 0;
				}
			}
		}

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
