/*
 * necomimi.c
 *
 *  Created on: 18 апр. 2022 г.
 *      Author: Morgan
 */

#include "necomimi.h"
#include "necomimi_queue.h"
#include "string.h"

#include "lcd.h"

// Минимальный размер пакета 6 байт. Пакеты идут быстро и буферизуются на низком уровне.
// Для виндовс приложения я делал 256 байт. Но здесь сделаю 512, из-за меньшей производительности.

#define NECOMIMI_BUFFER_SIZE 256

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
static uint8_t _buffer[NECOMIMI_BUFFER_SIZE];
static int _parse_packet(uint8_t *buffer, uint32_t size);

static NecomimiPacketUnit packetToQueue;

static int _parse_necomimi_header(uint8_t *buffer, uint32_t beginIndex, uint32_t bufSize)
{
	int parsingIndex = beginIndex;
	while (bufSize - parsingIndex >= 6)
	{
		if (buffer[parsingIndex] == NECOMIMI_HEADER_BYTE)
		{
			parsingIndex++;
			if (buffer[parsingIndex] == NECOMIMI_HEADER_BYTE)
			{
				parsingIndex++;
				int sizeOfPayload = buffer[parsingIndex];
				if (sizeOfPayload != 0xAA)
				{
					return parsingIndex;
				}
				else
				{
					parsingIndex++;
				}
				//Разобран HEADER
				int length = buffer[parsingIndex + 2];
			}
		}
		else
		{
			parsingIndex++;
		}
	}
	return -1;
}

static bool _bufferize_raw_packets(uint8_t *buffer, uint32_t size)
{
	if(size + _parsingBytesCount <= NECOMIMI_BUFFER_SIZE)
	{
		memcpy(_buffer + _parsingBytesCount, buffer, size);
		_parsingBytesCount += size;
	}
	//TODO: в противном случае, пытаться скопировать хотя бы часть.
}

uint32_t necomimi_parse_packet(uint8_t *buffer, uint32_t size)
{
	_bufferize_raw_packets(buffer, size);
	uint32_t parsingBytesCountBeforeParsing = _parsingBytesCount;

	uint32_t parsedUpToIndex = _parse_packet(_buffer, size);

	static uint8_t transferBuffer[NECOMIMI_BUFFER_SIZE] = { 0 };
	memcpy(transferBuffer, _buffer, NECOMIMI_BUFFER_SIZE);
	memcpy(_buffer, transferBuffer + parsedUpToIndex, parsingBytesCountBeforeParsing - parsedUpToIndex);

	_parsingBytesCount = parsingBytesCountBeforeParsing - parsedUpToIndex;
}

static bool _is_necomimi_crc_ok(uint8_t *payload, int beginIndex, int crcIndex, int bufLen)
{
	uint8_t calcCrc8Nec = 0;
	//TODO: все же верхний код должен делать проверку
	if (crcIndex >= bufLen - 1)
		return false;

	for (int i = beginIndex; i <= crcIndex; i++)
	{
		calcCrc8Nec += payload[i];
	}
	calcCrc8Nec = (uint8_t)(~calcCrc8Nec);

	if (calcCrc8Nec == payload[crcIndex + 1])
	{
		return true;
	}
	else
	{
		return false;
	}
}

static int _parse_packet(uint8_t *buffer, uint32_t size)
{
	//минимальный размер пакета по факту -  6 байт
	int parsingIndex = 0;
	int newParsedValues = 0;
	while (_parsingBytesCount - parsingIndex >= 6)
	{
		int headerOffset = _parse_necomimi_header(buffer, parsingIndex, _parsingBytesCount);

		if (headerOffset != -1)
		{
			parsingIndex = headerOffset;

			int sizeOfPacket = buffer[parsingIndex];
			if (sizeOfPacket + 1 <= _parsingBytesCount)
			{
				parsingIndex++;
				//Данных хватает
				int payloadBeginIndex = parsingIndex;
				//todo: при отладке проверить.
				int crcIndex = payloadBeginIndex + sizeOfPacket - 1;

				bool isCrcOk = _is_necomimi_crc_ok(buffer, payloadBeginIndex, crcIndex, _parsingBytesCount);

				if (isCrcOk)
				{
					memset(&packetToQueue, 0, sizeof(NecomimiPacketUnit));
					NecomimiPacketUnit parsedPacket = { 0 };
					bool isThereAttentionOrMediationPacks = false;
					while (parsingIndex < crcIndex)
					{
						uint32_t codeLevel = buffer[parsingIndex];
						switch (codeLevel)
						{
							case (ATTENTION):
								{
//									newParsedNecomimiPacket.AttentionCount = attentionCount;
//									newParsedNecomimiPacket.ESenseAttention = buffer[parsingIndex + 1];

									packetToQueue.attentionLevel = buffer[parsingIndex + 1];
									parsingIndex += 2;

									isThereAttentionOrMediationPacks = true;
									break;
								}
							case (MEDITATION):
								{
//									static meditationCount = 0;
//									char bufStr[25] = { 0 };

//									sprintf(bufStr, "MEDITATION:%d %d ", buffer[parsingIndex + 1], meditationCount++);
									packetToQueue.meditationLevel = buffer[parsingIndex + 1];
//									LCD_Clear();
//									LCD_SetPos(0, 1);
//									LCD_String(bufStr);
//									newParsedNecomimiPacket.ESenseMeditation = buffer[parsingIndex + 1];

									isThereAttentionOrMediationPacks = true;
									parsingIndex += 2;
									break;
								}
							case (POOR_SIGNAL_QUALITY):
								{
//									newParsedNecomimiPacket.PoorSignalQuality = buffer[parsingIndex + 1];
									parsingIndex += 2;
									break;
								}
							case (BATTERY_LEVEL):
								{
//									newParsedNecomimiPacket.BatteryLevel = buffer[parsingIndex + 1];
									parsingIndex += 2;
									break;
								}
								//не готово
							case (ASIC_EEG_POWER):
								{
									//(AsicEegPower, 0, 24);
									//TODO: на всякий случай дописать
									//newParsedNecomimiPacket.AsicEegPower = buffer[parsingIndex + 1];
									parsingIndex += 25;
									break;
								}
							case (EEG_POWER):
								{
									//(AsicEegPower, 0, 24);
									//TODO: на всякий случай дописать
									//newParsedNecomimiPacket.EegPower = buffer[parsingIndex + 1];
									parsingIndex += 33;
									break;
								}
							case (HEART_RATE):
								{
//									newParsedNecomimiPacket.HeartRate = buffer[parsingIndex + 1];
									parsingIndex += 2;
									break;
								}
							case (NEVER_USED):
								{
									parsingIndex += 2;
									break;
								}
							case (RAW_8BIT):
								{
//									newParsedNecomimiPacket.RawWaveValue8bit = buffer[parsingIndex + 1];
									parsingIndex += 2;
									break;
								}
							case (RAW_MARKER):
								{
//									newParsedNecomimiPacket.RawWaveMarker = buffer[parsingIndex + 1];
									parsingIndex += 2;
									break;
								}
							case (RAW_WAVE_VALUE):
								{
									uint16_t firstByte = buffer[parsingIndex + 1];
									uint16_t secondByte = buffer[parsingIndex + 2];
//									newParsedNecomimiPacket.RawWaveValue16bit = (uint16_t)(firstByte << 8 | secondByte);
									parsingIndex += 3;
									break;
								}
							case (RRINTERVAL):
								{
									uint16_t firstByte = buffer[parsingIndex + 1];
									uint16_t secondByte = buffer[parsingIndex + 2];
//									newParsedNecomimiPacket.PrintervalMs = (UInt16)(firstByte << 8 | secondByte);
									parsingIndex += 3;
									break;
								}
							default:
								parsingIndex++;
								break;
						}

					}
					if(isThereAttentionOrMediationPacks)
					{
						static uint32_t attentionCount = 0;

						static char bufStr[40] = { 0 };

//						parsedPacket.attentionLevel = buffer[parsingIndex + 1];

						sprintf(bufStr, "A:%d;M:%d,C:%d", packetToQueue.attentionLevel, packetToQueue.meditationLevel, attentionCount++);

						LCD_SetPos(0, 1);
						LCD_String(bufStr);

						necomimi_queue_enque(&packetToQueue);
					}
				}
			}
			else
			{
				parsingIndex++;
			}
		}
		else
		{
			break;
		}
	}
	return parsingIndex;
}



