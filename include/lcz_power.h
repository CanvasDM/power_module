/**
 * @file lcz_power.h
 * @brief Controls power measurement system and software reset.
 *
 * Copyright (c) 2020-2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */
#ifndef __LCZ_POWER_H__
#define __LCZ_POWER_H__

/******************************************************************************/
/* Includes                                                                   */
/******************************************************************************/
#include <devicetree.h>
#include <hal/nrf_power.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Board definitions                                                          */
/******************************************************************************/

/* Default and minimum measurement times between readings */
#define DEFAULT_LCZ_POWER_TIMER_PERIOD_MS (CONFIG_LCZ_ADC_SAMPLE_PERIOD * 1000)
#define MINIMUM_LCZ_POWER_TIMER_PERIOD_MS 500

/* Reboot types */
#define REBOOT_TYPE_NORMAL 0
#define REBOOT_TYPE_BOOTLOADER 1

/* ADC0 device */
#define ADC0 DT_PROP(DT_NODELABEL(adc), label)

/* clang-format off */
#define ADC_RESOLUTION               12
#define ADC_ACQUISITION_TIME         ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_CHANNEL_ID               0
#define ADC_SATURATION               2048
#define ADC_LIMIT_VALUE              4095.0
#define ADC_REFERENCE_VOLTAGE        0.6
#define ADC_DECIMAL_DIVISION_FACTOR  100.0 /* Keeps to 2 decimal places */
#define ADC_GAIN_FACTOR_SIX          6.0
#define ADC_GAIN_FACTOR_TWO          2.0
#define ADC_GAIN_FACTOR_ONE          1.0
#define ADC_GAIN_FACTOR_HALF         0.5
#define GPREGRET_BOOTLOADER_VALUE    0xb1
/* clang-format on */

#ifdef __cplusplus
}
#endif

#endif /* __LCZ_POWER_H__ */
