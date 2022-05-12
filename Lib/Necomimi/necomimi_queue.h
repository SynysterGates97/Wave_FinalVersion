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

uint32_t necomimi_get_elements_count();
bool necomimi_queue_init();
bool necomimi_queue_enque(NecomimiPacketUnit *necomimiPacket);
// Это позволит не взаимодействовать с мьютексом снаружи: "NecomimiPacketUnit *outputNecomimiPacket"
bool necomimi_queue_deque(NecomimiPacketUnit *outputNecomimiPacket);


#endif /* NECOMIMI_NECOMIMI_QUEUE_H_ */
