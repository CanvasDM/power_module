#include <drivers/adc.h>
#include <nrfx_power.h>

typedef enum {
	LCZ_POWER_CONFIGURATION_DIRECT = 0,
	LCZ_POWER_CONFIGURATION_POTENTIAL_DIVIDER
} power_measure_configuration_t;

typedef struct {
	FwkMsgHeader_t header;
	uint8_t instance;
	power_measure_configuration_t configuration;
	enum adc_gain gain;
	float voltage;
} power_measure_msg_t;

typedef struct {
	FwkMsgHeader_t header;
	uint8_t instance;
	bool target_sender;
} power_measure_now_msg_t;

typedef struct {
	FwkMsgHeader_t header;
	uint8_t instance;
	bool enabled;
	uint32_t interval_time;
} power_mode_msg_t;

typedef struct {
	FwkMsgHeader_t header;
	uint8_t instance;
	bool enabled;
	nrf_power_pof_thr_t power_level;
} power_fail_mode_msg_t;

typedef struct {
	FwkMsgHeader_t header;
	uint8_t reboot_type;
} power_reboot_msg_t;
