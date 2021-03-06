/*
 * necomimi_queue.c
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */

#include "necomimi_queue.h"
#include "cmsis_os.h"

#define NECOMIMI_MUTEX_WAIT_TIME_MS osWaitForever

osMutexDef(necomimiQueueMutex);
osMutexId necomimiQueueMutexId;

uint32_t necomimiQueueElementsInQueue;

// По терминологии FIFO - это индекс вставки, вакантное место
static uint32_t necomimiQueueBeginIndex;
static uint32_t necomimiQueueEndIndex;

static NecomimiPacketUnit necomimiQueue[NECOMIMI_QUEUE_SIZE];

static bool isNecomimiQueueInitialized = false;

bool necomimi_queue_init()
{
	necomimiQueueElementsInQueue = 0;

	necomimiQueueBeginIndex = 0;
	necomimiQueueEndIndex = 0;

	necomimiQueueMutexId = osMutexCreate(osMutex(necomimiQueueMutex));

	isNecomimiQueueInitialized = necomimiQueueMutexId != NULL;
	return isNecomimiQueueInitialized;
}

uint32_t necomimi_get_elements_count()
{
	return necomimiQueueElementsInQueue;
}
bool necomimi_queue_enque(NecomimiPacketUnit *necomimiPacket)
{
	if(isNecomimiQueueInitialized)
	{
		osStatus waitResult = osMutexWait(necomimiQueueMutexId, NECOMIMI_MUTEX_WAIT_TIME_MS);
		if(waitResult == osOK)
		{
			static uint32_t packetsEnquedCount = 0;
			necomimiQueue[necomimiQueueEndIndex] = *necomimiPacket;

			necomimiQueue[necomimiQueueEndIndex].packetNumber = packetsEnquedCount++;
			necomimiQueueEndIndex++;

			necomimiQueueElementsInQueue++;
			if(necomimiQueueElementsInQueue > NECOMIMI_QUEUE_SIZE)
				necomimiQueueElementsInQueue = NECOMIMI_QUEUE_SIZE;

			if (necomimiQueueEndIndex > NECOMIMI_QUEUE_SIZE - 1)
			{
				necomimiQueueEndIndex = 0;
			}

			if (necomimiQueueEndIndex == necomimiQueueBeginIndex)
			{
				necomimiQueueBeginIndex++;
				if (necomimiQueueBeginIndex > NECOMIMI_QUEUE_SIZE - 1)
				{
					necomimiQueueBeginIndex = 0;
				}
			}

			osMutexRelease(necomimiQueueMutexId);
			return true;
		}
		osMutexRelease(necomimiQueueMutexId);
	}
	return false;
}

// Это позволит не взаимодействовать с мьютексом снаружи: "NecomimiPacketUnit *outputNecomimiPacket"
bool necomimi_queue_deque(NecomimiPacketUnit *outputNecomimiPacket)
{
	if(isNecomimiQueueInitialized)
	{
		osStatus waitResult = osMutexWait(necomimiQueueMutexId, NECOMIMI_MUTEX_WAIT_TIME_MS);
		if(waitResult == osOK)
		{
			if (necomimiQueueElementsInQueue > 0)
			{
				*outputNecomimiPacket = necomimiQueue[necomimiQueueBeginIndex];
				necomimiQueueElementsInQueue--;

				if(necomimiQueueBeginIndex < NECOMIMI_QUEUE_SIZE - 1)
				{
					necomimiQueueBeginIndex++;
				}
				else
				{
					necomimiQueueBeginIndex = 0;
				}

				if(necomimiQueueBeginIndex == necomimiQueueEndIndex)
				{
					necomimiQueueEndIndex++;
					if (necomimiQueueEndIndex > NECOMIMI_QUEUE_SIZE - 1)
					{
						necomimiQueueEndIndex = 0;
					}
				}

				osMutexRelease(necomimiQueueMutexId);
				return true;
			}

			osMutexRelease(necomimiQueueMutexId);
			return false;
		}

		osMutexRelease(necomimiQueueMutexId);
	}
	return false;
}
