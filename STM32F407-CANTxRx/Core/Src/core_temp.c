/*
 * core_temp.c
 *
 *  Created on: Aug 26, 2026
 *      Author: Debasish Das
 */

#include "core_temp.h"

#include "stm32f407xx.h"
#include "stm32f4xx.h"

extern ADC_HandleTypeDef hadc1;

#include "main.h"
#include <stdint.h>

/* ADC specific declarations Starts */
#define MSG_ID_TEMP       0x100U
#define ADC_MAX_VALUE     4095.0f
#define ADC_REFERENCE     3.3f

#define TEMP_CAL1_ADDR    ((uint16_t *)0x1FFF7A2CU)
#define TEMP_CAL2_ADDR    ((uint16_t *)0x1FFF7A2EU)

#define TEMP_CAL1         (*TEMP_CAL1_ADDR)
#define TEMP_CAL2         (*TEMP_CAL2_ADDR)

#define TEMP_CAL1_TEMP    30.0f
#define TEMP_CAL2_TEMP    110.0f
/* ADC specific declarations Ends */


float STM32_ReadTemperature(void)
{
    uint32_t adcValue;
    float temperature;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return -999.0f;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 100U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return -999.0f;
    }

    adcValue = HAL_ADC_GetValue(&hadc1);

    (void)HAL_ADC_Stop(&hadc1);

    /*
     * Linear interpolation using factory calibration:
     *
     * CAL1 = ADC value at 30 °C
     * CAL2 = ADC value at 110 °C
     */
    temperature = ((float)((int32_t)adcValue -
                           (int32_t)TEMP_CAL1) *
                   (TEMP_CAL2_TEMP - TEMP_CAL1_TEMP)) /
                  ((float)((int32_t)TEMP_CAL2 -
                           (int32_t)TEMP_CAL1));

    temperature += TEMP_CAL1_TEMP;

    return temperature;
}
