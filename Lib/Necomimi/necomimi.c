/*
 * necomimi.c
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */

#include "necomimi.h"
#include "necomimi_queue.h"

// Минимальный размер пакета 6 байт. Пакеты идут быстро и буферизуются на низком уровне.
// Для виндовс приложения я делал 256 байт. Но здесь сделаю 512, из-за меньшей производительности.

#define NECOMIMI_BUFFER_SIZE 512

#define NECOMIMI_MINIMAL_PACKET_SIZE 6
#define NECOMIMI_HEADER_BYTE 0xAA

enum CodeLevels
{
	BATTERY_LEVEL = 0x01,
	POOR_SIGNAL_QUALITY = 0x02,
	HEART_RATE = 0x03,
	ATTENTION = 0x04,
	MEDITATION = 0x05,
	RAW_8BIT = 0x06,
	RAW_MARKER = 0x07,
	RAW_WAVE_VALUE = 0x80,
	EEG_POWER = 0x81,
	ASIC_EEG_POWER = 0x83,
	RRINTERVAL = 0x86,
	NEVER_USED = 0x55
};

// количество ещё нераспарсенных байт в буфере.
// буфер будет всегда сдвигаться в право, до тех пока не найдется 0xAA
static uint32_t _parsingBytesCount = 0;
static uint32_t _parsingBeginIndex = 0;
static uint8_t _buffer[NECOMIMI_BUFFER_SIZE];

static bool _bufferize_raw_packets(uint8_t *buffer, uint32_t size)
{
	if(size + _parsingBeginIndex + _parsingBytesCount <= NECOMIMI_BUFFER_SIZE)
	{
		memcpy(_buffer + _parsingBytesCount + _parsingBeginIndex, buffer, size);
	}
	//TODO: в противном случае, пытаться скопировать хотя бы часть.
}

uint32_t necomimi_parse_packet(uint8_t *buffer, uint32_t size)
{
	_bufferize_raw_packets(buffer, size);
	bool needToParse = true;
	while (needToParse)
	{
		// TODO: логика парсинга
		needToParse = false;
	}

}

