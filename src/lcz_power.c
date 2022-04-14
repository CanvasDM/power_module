/**
 * @file lcz_power.c
 * @brief Voltage measurement control
 *
 * Copyright (c) 2020-2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(lcz_power, CONFIG_LCZ_POWER_LOG_LEVEL);

/******************************************************************************/
/* Includes                                                                   */
/******************************************************************************/
#include <stdio.h>
#include <zephyr/types.h>
#include <kernel.h>
#include <drivers/gpio.h>
#include <hal/nrf_saadc.h>
#include <drivers/adc.h>
#include <logging/log_ctrl.h>
#include <Framework.h>
#include <FrameworkMacros.h>
#include <framework_ids.h>
#include <framework_msgcodes.h>
#include <framework_types.h>
#include <BufferPool.h>
#include <locking_defs.h>
#include <locking.h>

#ifdef CONFIG_REBOOT
#include <sys/reboot.h>
#endif

#include "lcz_power.h"

/******************************************************************************/
/* Local Constant, Macro and Type Definitions                                 */
/******************************************************************************/
#define LCZ_POWER_PRIORITY K_PRIO_PREEMPT(1)

/* clang-format off */
/* Port and pin number of the voltage measurement enabling functionality */
#if defined(CONFIG_BOARD_MG100)
#define MEASURE_ENABLE_PORT DT_PROP(DT_NODELABEL(gpio1), label)
#define MEASURE_ENABLE_PIN 10
#elif defined(CONFIG_BOARD_PINNACLE_100_DVK)
#define MEASURE_ENABLE_PORT DT_PROP(DT_NODELABEL(gpio0), label)
#define MEASURE_ENABLE_PIN 28
#else
#error "A measurement enable pin must be defined for this board."
#endif

/* clang-format off */
#define ADC_VOLTAGE_TOP_RESISTOR     14.1
#define ADC_VOLTAGE_BOTTOM_RESISTOR  1.1
#define MEASURE_STATUS_ENABLE        1
#define MEASURE_STATUS_DISABLE       0
/* clang-format on */

/******************************************************************************/
/* Local Data Definitions                                                     */
/******************************************************************************/
/* NEED CONFIG_ADC_CONFIGURABLE_INPUTS */

static struct adc_channel_cfg m_1st_channel_cfg = {
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
#if defined CONFIG_BOARD_PINNACLE_100_DVK
	.input_positive = NRF_SAADC_INPUT_AIN5
#elif defined CONFIG_BOARD_MG100
	.input_positive = NRF_SAADC_INPUT_AIN0
#else
#error "An ADC input must be defined for this hardware variant."
#endif
};

static int16_t m_sample_buffer;
static struct k_timer lcz_power_timer;
static struct k_work lcz_power_work;
static bool timer_enabled;
static uint32_t timer_interval = DEFAULT_LCZ_POWER_TIMER_PERIOD_MS;

static FwkMsgTask_t lcz_power_task;
K_THREAD_STACK_DEFINE(lcz_power_thread_stack,
		      CONFIG_LCZ_POWER_THREAD_STACK_SIZE);
K_MSGQ_DEFINE(lcz_power_queue, FWK_QUEUE_ENTRY_SIZE,
	      CONFIG_LCZ_POWER_THREAD_QUEUE_DEPTH, FWK_QUEUE_ALIGNMENT);

/******************************************************************************/
/* Local Function Prototypes                                                  */
/******************************************************************************/
static int lcz_power_init(const struct device *device);
static bool lcz_power_measure_adc(const struct device *adc_dev,
				  enum adc_gain gain,
				  const struct adc_sequence sequence);
static void lcz_power_run(FwkId_t *target);
static void system_workq_lcz_power_timer_handler(struct k_work *item);
static void lcz_power_timer_callback(struct k_timer *timer_id);

static DispatchResult_t lcz_power_measure_now(FwkMsgReceiver_t *receiver,
					  FwkMsg_t *msg);
static DispatchResult_t lcz_power_mode_set(FwkMsgReceiver_t *receiver,
				       FwkMsg_t *msg);
static DispatchResult_t lcz_power_interval_get(FwkMsgReceiver_t *receiver,
					   FwkMsg_t *msg);

#ifdef CONFIG_REBOOT
static DispatchResult_t lcz_power_reboot(FwkMsgReceiver_t *receiver,
					 FwkMsg_t *msg);
#endif

static FwkMsgHandler_t *lcz_power_dispatcher(FwkMsgCode_t msg_code);

static void lcz_power_thread(void *arg1, void *arg2, void *arg3);

/******************************************************************************/
/* Global Function Definitions                                                */
/******************************************************************************/
SYS_INIT(lcz_power_init, APPLICATION, CONFIG_LCZ_POWER_INIT_PRIORITY);

/******************************************************************************/
/* Local Function Definitions                                                 */
/******************************************************************************/
static DispatchResult_t lcz_power_measure_now(FwkMsgReceiver_t *receiver,
					      FwkMsg_t *msg)
{
	lcz_power_measure_now_msg_t *fmsg = (lcz_power_measure_now_msg_t *)msg;

	lcz_power_run((fmsg->target_sender == true ? &msg->header.txId : NULL));

	return DISPATCH_OK;
}

static DispatchResult_t lcz_power_mode_set(FwkMsgReceiver_t *receiver,
					   FwkMsg_t *msg)
{
	lcz_power_mode_msg_t *fmsg = (lcz_power_mode_msg_t *)msg;

	if (fmsg->interval_time >= MINIMUM_LCZ_POWER_TIMER_PERIOD_MS) {
		timer_interval = fmsg->interval_time;
	}

	if (fmsg->enabled == true && timer_enabled == false) {
		k_timer_start(&lcz_power_timer, K_MSEC(timer_interval),
			      K_MSEC(timer_interval));
	} else if (fmsg->enabled == false && timer_enabled == true) {
		k_timer_stop(&lcz_power_timer);
	}

	timer_enabled = fmsg->enabled;

	if (fmsg->enabled == true) {
		/* Take a reading right away */
		lcz_power_run(NULL);
	}

	return DISPATCH_OK;
}

static DispatchResult_t lcz_power_interval_get(FwkMsgReceiver_t *receiver,
					       FwkMsg_t *msg)
{
	lcz_power_mode_msg_t *fmsg = (lcz_power_mode_msg_t *)BufferPool_Take(
						sizeof(lcz_power_mode_msg_t));

	if (fmsg != NULL) {
		fmsg->header.msgCode = FMC_LCZ_SENSOR_CONFIG_GET;
		fmsg->header.txId = FWK_ID_LCZ_POWER;
		fmsg->header.rxId = msg->header.txId;
		fmsg->instance = 0;
		fmsg->enabled = timer_enabled;
		fmsg->interval_time = timer_interval;

		Framework_Send(msg->header.txId, (FwkMsg_t *)fmsg);
	}

	return DISPATCH_OK;
}

#ifdef CONFIG_REBOOT
static DispatchResult_t lcz_power_reboot(FwkMsgReceiver_t *receiver,
					 FwkMsg_t *msg)
{
	lcz_power_reboot_msg_t *fmsg = (lcz_power_reboot_msg_t *)msg;

	/* Log panic will cause all buffered logs to be output */
	LOG_INF("Rebooting module%s...",
		(fmsg->reboot_type == REBOOT_TYPE_BOOTLOADER ?
			" into UART bootloader" :
			""));
#if defined(CONFIG_LOG) && !defined(CONFIG_LOG_MODE_MINIMAL)
	log_panic();
#endif

	/* And reboot the module */
	sys_reboot((fmsg->reboot_type == REBOOT_TYPE_BOOTLOADER ?
						GPREGRET_BOOTLOADER_VALUE :
						0));
}
#endif

static FwkMsgHandler_t *lcz_power_dispatcher(FwkMsgCode_t msg_code)
{
	if (msg_code == FMC_LCZ_SENSOR_MEASURE_NOW) {
		return lcz_power_measure_now;
	} else if (msg_code == FMC_LCZ_SENSOR_CONFIG_SET) {
		return lcz_power_mode_set;
	} else if (msg_code == FMC_LCZ_SENSOR_CONFIG_GET) {
		return lcz_power_interval_get;
#ifdef CONFIG_REBOOT
	} else if (msg_code == FMC_LCZ_POWER_REBOOT) {
		return lcz_power_reboot;
#endif
	}

	return NULL;
}

static void lcz_power_thread(void *arg1, void *arg2, void *arg3)
{
        FwkMsgTask_t *task = (FwkMsgTask_t *)arg1;

        while (true) {
                Framework_MsgReceiver(&task->rxer);
        }
}

static bool lcz_power_measure_adc(const struct device *adc_dev,
				  enum adc_gain gain,
				  const struct adc_sequence sequence)
{
	int ret = 0;

	/* Setup ADC with desired gain */
	m_1st_channel_cfg.gain = gain;
	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (ret) {
		LOG_ERR("adc_channel_setup failed with %d", ret);
		return false;
	}

	/* Take ADC reading */
	ret = adc_read(adc_dev, &sequence);
	if (ret) {
		LOG_ERR("adc_read failed with %d", ret);
		return false;
	}

	return true;
}

static void lcz_power_run(FwkId_t *target)
{
	int ret;
	bool finished = false;
	uint8_t scaling;

	/* Find the ADC device */
	const struct device *adc_dev = device_get_binding(ADC0);
	if (adc_dev == NULL) {
		LOG_ERR("ADC device name %s not found", ADC0);
		return;
	}

	(void)memset(&m_sample_buffer, 0, sizeof(m_sample_buffer));

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = &m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	/* Prevent other ADC uses */
	locking_take(LOCKING_ID_adc, K_FOREVER);

	/* Enable power supply voltage to be monitored */
	ret = gpio_pin_set(device_get_binding(MEASURE_ENABLE_PORT),
			   MEASURE_ENABLE_PIN, MEASURE_STATUS_ENABLE);
	if (ret) {
		LOG_ERR("Error setting power GPIO");
		return;
	}

	/* Measure voltage with 1/2 scaling which is suitable for higher
	   voltage supplies */
	lcz_power_measure_adc(adc_dev, ADC_GAIN_1_2, sequence);
	scaling = ADC_GAIN_FACTOR_TWO;

	if (m_sample_buffer >= ADC_SATURATION) {
		/* We have reached saturation point, do not try the next ADC
		   scaling */
		finished = true;
	}

	if (finished == false) {
		/* Measure voltage with unity scaling which is suitable for
		   medium voltage supplies */
		lcz_power_measure_adc(adc_dev, ADC_GAIN_1, sequence);
		scaling = ADC_GAIN_FACTOR_ONE;

		if (m_sample_buffer >= ADC_SATURATION) {
			/* We have reached saturation point, do not try the
			   next ADC scaling */
			finished = true;
		}
	}

	if (finished == false) {
		/* Measure voltage with double scaling which is suitable for
		   low voltage supplies, such as 2xAA batteries */
		lcz_power_measure_adc(adc_dev, ADC_GAIN_2, sequence);
		scaling = ADC_GAIN_FACTOR_HALF;
	}

	/* Disable the voltage monitoring FET */
	ret = gpio_pin_set(device_get_binding(MEASURE_ENABLE_PORT),
			   MEASURE_ENABLE_PIN, MEASURE_STATUS_DISABLE);

	if (ret) {
		LOG_ERR("Error setting power GPIO");
	}

	locking_give(LOCKING_ID_adc);

	lcz_power_measure_msg_t *fmsg =
			(lcz_power_measure_msg_t *)BufferPool_Take(
					sizeof(lcz_power_measure_msg_t));

	if (fmsg != NULL) {
		fmsg->header.msgCode = FMC_LCZ_SENSOR_MEASURED;
		fmsg->header.txId = FWK_ID_LCZ_POWER;
		fmsg->instance = 0;
		fmsg->configuration = LCZ_POWER_CONFIGURATION_POTENTIAL_DIVIDER;

		if (scaling == ADC_GAIN_FACTOR_TWO) {
			fmsg->gain = ADC_GAIN_2;
		} else if (scaling == ADC_GAIN_FACTOR_ONE) {
			fmsg->gain = ADC_GAIN_1;
		} else if (scaling == ADC_GAIN_FACTOR_HALF) {
			fmsg->gain = ADC_GAIN_1_2;
		}

		fmsg->voltage = (float)m_sample_buffer / ADC_LIMIT_VALUE *
				ADC_REFERENCE_VOLTAGE *
				ADC_VOLTAGE_TOP_RESISTOR /
				ADC_VOLTAGE_BOTTOM_RESISTOR * scaling;

		if (target == NULL) {
#ifdef CONFIG_FILTER
			/* With filtering, send targetted message to filter */
			fmsg->header.rxId = FWK_ID_EVENT_FILTER;
			Framework_Send(FWK_ID_EVENT_FILTER, (FwkMsg_t *)fmsg);
#else
			/* Without filtering, send broadcast */
			fmsg->header.rxId = FWK_ID_RESERVED;
			Framework_Broadcast((FwkMsg_t *)fmsg,
					    sizeof(lcz_power_measure_msg_t));
#endif
		} else {
			/* Targetted message, send only to target */
			fmsg->header.rxId = *target;
			Framework_Send(*target, (FwkMsg_t *)fmsg);
		}
	}
}

static void system_workq_lcz_power_timer_handler(struct k_work *item)
{
	lcz_power_run(NULL);
}

/******************************************************************************/
/* Interrupt Service Routines                                                 */
/******************************************************************************/
static void lcz_power_timer_callback(struct k_timer *timer_id)
{
	/* Add item to system work queue so that it can be handled in task
	 * context because ADC cannot be used in interrupt context (mutex)
	 */
	k_work_submit(&lcz_power_work);
}

/******************************************************************************/
/* SYS INIT                                                                   */
/******************************************************************************/
static int lcz_power_init(const struct device *device)
{
	ARG_UNUSED(device);
	int ret;

	/* Setup work-queue and repetitive timer */
	k_timer_init(&lcz_power_timer, lcz_power_timer_callback, NULL);
	k_work_init(&lcz_power_work, system_workq_lcz_power_timer_handler);

	/* Configure the VIN_ADC_EN pin as an output set low to disable the
	   power supply voltage measurement */
	ret = gpio_pin_configure(device_get_binding(MEASURE_ENABLE_PORT),
				 MEASURE_ENABLE_PIN, (GPIO_OUTPUT));
	if (ret) {
		LOG_ERR("Error configuring power GPIO");
		return -EIO;
	}

	ret = gpio_pin_set(device_get_binding(MEASURE_ENABLE_PORT),
			   MEASURE_ENABLE_PIN, MEASURE_STATUS_DISABLE);
	if (ret) {
		LOG_ERR("Error setting power GPIO");
		return -EIO;
	}

	/* Create thread for framework message processing */
	lcz_power_task.rxer.id = FWK_ID_LCZ_POWER;
	lcz_power_task.rxer.rxBlockTicks = K_FOREVER;
	lcz_power_task.rxer.pMsgDispatcher = lcz_power_dispatcher;
	lcz_power_task.timerDurationTicks = K_MSEC(0);
	lcz_power_task.timerPeriodTicks = K_MSEC(0);
	lcz_power_task.rxer.pQueue = &lcz_power_queue;

	Framework_RegisterTask(&lcz_power_task);

	lcz_power_task.pTid =
		k_thread_create(&lcz_power_task.threadData,
				lcz_power_thread_stack,
				K_THREAD_STACK_SIZEOF(lcz_power_thread_stack),
				lcz_power_thread, &lcz_power_task, NULL, NULL,
				LCZ_POWER_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(lcz_power_task.pTid, "lcz_power");

	return 0;
}
