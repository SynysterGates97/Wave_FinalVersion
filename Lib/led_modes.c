#include "led_modes.h"

// Количество точек по которым будет браться среднее для алгоримтма
// простого скользящего среднего.
// SMA - simple moving average
#define SMA_COUNTS 100

static uint16_t smaQueue[SMA_COUNTS];

static uint8_t smaQueueEnqueIndex = 0;
static uint16_t smaQueueElementsCount = 0;

// Используется строго для вызова для одних целей.
// Пока не вижу необходимости вычисления скользящего среднего для других целей,
// кроме эмуляции огня.
uint16_t sma_get_average_value(uint16_t value)
{
	smaQueue[smaQueueEnqueIndex] = value;
	smaQueueEnqueIndex++;

	if (smaQueueEnqueIndex > SMA_COUNTS - 1)
	{
		smaQueueEnqueIndex = smaQueueEnqueIndex - SMA_COUNTS;
	}
	else
	{
		smaQueueElementsCount++;
	}

	uint16_t sumOfElements = 0;
	for (int i = 0; i < smaQueueElementsCount; ++i)
	{
		sumOfElements += smaQueue[i];
	}

	// return sma_samples_sum()/smaElementsInSumCount
	return sumOfElements / smaQueueElementsCount;

}
