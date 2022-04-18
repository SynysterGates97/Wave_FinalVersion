/*
 * necomimi.h
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */
#include "stdint.h"
#include "necomimi_packet.h"
#include "stdbool.h"

#ifndef NECOMIMI_NECOMIMI_H_
#define NECOMIMI_NECOMIMI_H_


enum ParsingResult
{
	PARSED_OK,
	HEADER_NOT_FOUND,
	NOT_ENOUGH_DATA,
	WRONG_CRC,
	ERROR
};

uint32_t necomimi_parse_packet(uint8_t *buffer, uint32_t size);

#endif /* NECOMIMI_NECOMIMI_H_ */
