/*
 * necomimi.h
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */
#include "stdint.h"

#ifndef NECOMIMI_NECOMIMI_H_
#define NECOMIMI_NECOMIMI_H_

#define NECOMIMI_QUEUE_SIZE 50

enum ParsingResult
{
	PARSED_OK,
	HEADER_NOT_FOUND,
	NOT_ENOUGH_DATA,
	WRONG_CRC,
	ERROR
};

typedef struct
{
	uint8_t meditationLevel;
	uint8_t attentionLevel;
}NecomimiPacketUnit;

uint32_t necomimi_parse_packet(uint8_t *buffer, uint32_t size);
NecomimiPacketUnit necomimi_dequeue_packet_from_queue();
uint32_t necomimi_get_elements_in_queue();

extern NecomimiPacketUnit necomimiPacketsQueue[NECOMIMI_PACKETS_BUF_SIZE];

#endif /* NECOMIMI_NECOMIMI_H_ */
