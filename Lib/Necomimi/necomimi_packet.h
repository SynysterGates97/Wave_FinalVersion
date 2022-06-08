/*
 * necomimi_packet.h
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */

#ifndef NECOMIMI_NECOMIMI_PACKET_H_
#define NECOMIMI_NECOMIMI_PACKET_H_

#include "stdint.h"

typedef struct
{
	uint8_t meditationLevel;
	uint8_t attentionLevel;
	uint32_t packetNumber;
}NecomimiPacketUnit;

#endif /* NECOMIMI_NECOMIMI_PACKET_H_ */
