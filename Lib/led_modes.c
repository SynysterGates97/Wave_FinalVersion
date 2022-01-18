#include "led_modes.h"

// Количество точек по которым будет браться среднее для алгоримтма
// простого скользящего среднего.
// SMA - simple moving average
#define SMA_COUNTS 100

static uint16_t smaSamples[SMA_COUNTS];

static uint8_t lastElementInSamples = 0;
static uint8_t firstElementInSamples= 0;

static uint16_t smaElementsInSumCount = 0;

// Используется строго для вызова для одних целей.
// Пока не вижу необходимости вычисления скользящего среднего для других целей,
// кроме эмуляции огня.
uint16_t sma_get_average_value(uint16_t value)
{
	if (lastElementInSamples == firstElementInSamples &&
			firstElementInSamples == 0)
	{
		smaSamples[firstElementInSamples] = value;
	}
	else if(lastElementInSamples > firstElementInSamples)
	{

	}

	if (smaElementsInSumCount > SMA_COUNTS)
	{

	}

	// return sma_samples_sum()/smaElementsInSumCount
	return 0;

}
