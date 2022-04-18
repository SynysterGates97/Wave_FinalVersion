/*
 * necomimi_queue.h
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */

#ifndef NECOMIMI_NECOMIMI_QUEUE_H_
#define NECOMIMI_NECOMIMI_QUEUE_H_

#include "necomimi.h"
#include "stdint.h"
#include "necomimi_packet.h"


#define NECOMIMI_QUEUE_SIZE 50

bool necomimi_queue_enque(NecomimiPacketUnit *necomimiPacket);
NecomimiPacketUnit necomimi_queue_deque();

extern uint32_t necomimiQueueElementsInQueue;
extern NecomimiPacketUnit necomimiQueue[NECOMIMI_QUEUE_SIZE];

#endif /* NECOMIMI_NECOMIMI_QUEUE_H_ */
