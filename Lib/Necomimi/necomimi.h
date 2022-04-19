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
	NECO_PARSE_PARSED_OK,
	NECO_PARSE_HEADER_NOT_FOUND,
	NECO_PARSE_NOT_ENOUGH_DATA,
	NECO_PARSE_WRONG_CRC,
	NECO_PARSE_ERROR
};

uint32_t necomimi_parse_packet(uint8_t *buffer, uint32_t size);

#endif /* NECOMIMI_NECOMIMI_H_ */
