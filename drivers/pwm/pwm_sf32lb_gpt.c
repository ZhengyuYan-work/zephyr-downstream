/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_gpt_pwm

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <ll_gptim.h>

#define MAX_CH_NUM      4U

LOG_MODULE_REGISTER(pwm_sf32lb, CONFIG_PWM_LOG_LEVEL);

struct pwm_sf32lb_config {
	GPT_TypeDef *tim;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	uint16_t prescaler;
};

static int pwm_sf32lb_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_sf32lb_config *config = dev->config;
	GPT_TypeDef *tim = config->tim;

	if (channel >= MAX_CH_NUM) {
		LOG_ERR("Invalid PWM channel: %u. Must be 0-3.", channel);
		return -EINVAL;
	}

	LOG_DBG("Setting PWM period_cycles: %d, pulse_cycles: %d", period_cycles, pulse_cycles);

	if ((period_cycles > UINT16_MAX) || (pulse_cycles > UINT16_MAX)) {
		LOG_ERR("Cannot set PWM output, value exceeds 16-bit timer limit.");
		return -ENOTSUP;
	}

	if (period_cycles == 0U) {
		ll_gptim_disable_channel(tim, channel + 1U);
		return 0;
	}

	ll_gptim_disable_channel(tim, channel + 1U);

	ll_gptim_set_channel_polarity(tim, channel + 1U,
				      (flags & PWM_POLARITY_INVERTED) ? 1U : 0U);

	ll_gptim_set_auto_reload(tim, (uint16_t)(period_cycles - 1U));
	ll_gptim_set_compare(tim, channel + 1U, (uint16_t)pulse_cycles);

	ll_gptim_set_output_compare_mode(tim, channel + 1U, LL_GPTIM_OC_MODE_PWM1);
	ll_gptim_set_output_compare_preload(tim, channel + 1U, 1U);

	ll_gptim_enable_channel(tim, channel + 1U);

	return 0;
}

static int pwm_sf32lb_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					 uint64_t *cycles)
{
	const struct pwm_sf32lb_config *config = dev->config;
	uint32_t clock_freq;
	int ret;

	if (channel >= MAX_CH_NUM) {
		LOG_ERR("Invalid PWM channel: %u. Must be 0-3.", channel);
		return -EINVAL;
	}

	ret = sf32lb_clock_control_get_rate_dt(&config->clock, &clock_freq);
	if (ret < 0) {
		return ret;
	}

	*cycles = (uint64_t)(clock_freq / (config->prescaler + 1U));

	return ret;
}

static int pwm_sf32lb_init(const struct device *dev)
{
	const struct pwm_sf32lb_config *config = dev->config;
	GPT_TypeDef *tim = config->tim;
	int ret;

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure pins");
		return ret;
	}

	ll_gptim_set_prescaler(tim, config->prescaler);
	ll_gptim_generate_update(tim);
	ll_gptim_enable(tim);

	return ret;
}

static DEVICE_API(pwm, pwm_sf32lb_driver_api) = {
	.set_cycles = pwm_sf32lb_set_cycles,
	.get_cycles_per_sec = pwm_sf32lb_get_cycles_per_sec,
};

#define PWM_SF32LB_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct pwm_sf32lb_config pwm_sf32lb_config_##n = {                            \
		.tim = (GPT_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock = SF32LB_CLOCK_DT_INST_PARENT_SPEC_GET(n),                                  \
		.prescaler = DT_PROP(DT_INST_PARENT(n), sifli_prescaler),                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, pwm_sf32lb_init, NULL, NULL,                                      \
			      &pwm_sf32lb_config_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,       \
			      &pwm_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SF32LB_DEFINE)
