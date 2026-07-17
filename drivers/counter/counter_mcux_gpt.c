/*
 * Copyright (c) 2019, Linaro Limited.
 * Copyright 2024-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_gpt

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#if defined(CONFIG_GIC)
#include <zephyr/drivers/interrupt_controller/gic.h>
#endif /* CONFIG_GIC */

#include <fsl_gpt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>

LOG_MODULE_REGISTER(mcux_gpt, CONFIG_COUNTER_LOG_LEVEL);

#define DEV_CFG(_dev) ((const struct mcux_gpt_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct mcux_gpt_data *)(_dev)->data)

struct mcux_gpt_config {
	/* info must be first element */
	struct counter_config_info info;

	DEVICE_MMIO_NAMED_ROM(gpt_mmio);

	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	bool enable_free_run;
	unsigned int irq;
	void (*irq_config_func)(void);
};

struct mcux_gpt_data {
	DEVICE_MMIO_NAMED_RAM(gpt_mmio);
	counter_alarm_callback_t alarm_callback;
	counter_top_callback_t top_callback;
	void *alarm_user_data;
	void *top_user_data;
	uint32_t guard_period;
	bool late_alarm_pending;
	struct k_spinlock lock;
};

static inline void mcux_gpt_set_irq_pending(unsigned int irq)
{
#if defined(CONFIG_GIC)
	arm_gic_irq_set_pending(irq);
#else  /* NVIC */
	NVIC_SetPendingIRQ(irq);
#endif /* CONFIG_GIC */
}

static inline bool mcux_gpt_irq_is_pending(unsigned int irq)
{
#if defined(CONFIG_GIC)
	return arm_gic_irq_is_pending(irq);
#else  /* NVIC */
	return NVIC_GetPendingIRQ((IRQn_Type)irq) != 0U;
#endif /* CONFIG_GIC */
}

static GPT_Type *get_base(const struct device *dev)
{
	return (GPT_Type *)DEVICE_MMIO_NAMED_GET(dev, gpt_mmio);
}

static int mcux_gpt_start(const struct device *dev)
{
	GPT_Type *base = get_base(dev);

	GPT_StartTimer(base);

	return 0;
}

static int mcux_gpt_stop(const struct device *dev)
{
	GPT_Type *base = get_base(dev);

	GPT_StopTimer(base);

	return 0;
}

static int mcux_gpt_get_value(const struct device *dev, uint32_t *ticks)
{
	GPT_Type *base = get_base(dev);

	*ticks = GPT_GetCurrentTimerCount(base);
	return 0;
}

static int mcux_gpt_set_alarm(const struct device *dev, uint8_t chan_id,
			      const struct counter_alarm_cfg *alarm_cfg)
{
	const struct mcux_gpt_config *config = dev->config;
	GPT_Type *base = get_base(dev);
	struct mcux_gpt_data *data = dev->data;
	uint32_t ticks = alarm_cfg->ticks;
	bool absolute = (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0;

	if (chan_id >= config->info.channels) {
		LOG_ERR("Invalid channel id");
		return -EINVAL;
	}

	if (alarm_cfg->callback == NULL) {
		return -EINVAL;
	}

	if (data->alarm_callback) {
		return -EBUSY;
	}

	/* OCR1 is an absolute compare against the free-running counter. If the
	 * counter has already passed the compare value, the match does not fire
	 * until the 32-bit counter wraps (~179 s at 24 MHz). When a guard
	 * period is configured, a late absolute alarm is rejected with
	 * -ETIME. If COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE is also set, the
	 * callback is additionally delivered from the GPT ISR — the same
	 * context a timely alarm fires from — by pending the IRQ in software.
	 *
	 * The spinlock makes the read-check-program sequence atomic:
	 * preemption cannot let the counter advance past the target between
	 * the counter read and the OCR1 write, and it also serializes this
	 * sequence against cancel_alarm() and the ISR.
	 */
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t now = GPT_GetCurrentTimerCount(base);

	if (!absolute) {
		ticks += now;
	}

	if (data->guard_period > 0) {
		/* "Late" (counting up): the counter has advanced past the
		 * target by fewer than guard_period ticks.
		 */
		bool late = (uint32_t)(now - ticks) < data->guard_period;

		if (late) {
			if (alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) {
				/* The target is in the past. Deliver the
				 * callback from the GPT ISR — the same context
				 * a timely alarm fires from — by pending the
				 * IRQ in software. OCR1 is left unprogrammed:
				 * the compare value is behind the counter, so a
				 * hardware match would not fire until the
				 * 32-bit wrap.
				 */
				data->alarm_callback = alarm_cfg->callback;
				data->alarm_user_data = alarm_cfg->user_data;
				data->late_alarm_pending = true;
				mcux_gpt_set_irq_pending(config->irq);
				k_spin_unlock(&data->lock, key);
				return -ETIME;
			}

			k_spin_unlock(&data->lock, key);
			return -ETIME;
		}
	}

	/* Commit to the alarm: store the callback and program OCR1 inside the
	 * critical section so the ISR cannot observe a half-installed alarm.
	 */
	data->alarm_callback = alarm_cfg->callback;
	data->alarm_user_data = alarm_cfg->user_data;

	GPT_SetOutputCompareValue(base, kGPT_OutputCompare_Channel1,
				  ticks);
	GPT_EnableInterrupts(base, kGPT_OutputCompare1InterruptEnable);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int mcux_gpt_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct mcux_gpt_config *config = dev->config;
	GPT_Type *base = get_base(dev);
	struct mcux_gpt_data *data = dev->data;

	if (chan_id >= config->info.channels) {
		LOG_ERR("Invalid channel id");
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	GPT_DisableInterrupts(base, kGPT_OutputCompare1InterruptEnable);
	data->alarm_callback = NULL;
	data->late_alarm_pending = false;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static uint32_t mcux_gpt_get_guard_period(const struct device *dev,
					 uint32_t flags)
{
	struct mcux_gpt_data *data = dev->data;

	if (flags & COUNTER_GUARD_PERIOD_LATE_TO_SET) {
		return data->guard_period;
	}

	return 0U;
}

static int mcux_gpt_set_guard_period(const struct device *dev, uint32_t guard,
				    uint32_t flags)
{
	struct mcux_gpt_data *data = dev->data;

	if (!(flags & COUNTER_GUARD_PERIOD_LATE_TO_SET)) {
		return -ENOSYS;
	}

	data->guard_period = guard;

	return 0;
}

void mcux_gpt_isr(const struct device *dev)
{
	GPT_Type *base = get_base(dev);
	struct mcux_gpt_data *data = dev->data;
	uint32_t current = GPT_GetCurrentTimerCount(base);
	uint32_t status;

	status =  GPT_GetStatusFlags(base, kGPT_OutputCompare1Flag |
				     kGPT_RollOverFlag);
	GPT_ClearStatusFlags(base, status);
	barrier_dsync_fence_full();

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	bool late_alarm_pending = data->late_alarm_pending;
	counter_alarm_callback_t alarm_cb = data->alarm_callback;
	void *alarm_user_data = data->alarm_user_data;

	if (late_alarm_pending) {
		/* Late alarm dispatched via a software-pended IRQ
		 * (COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE). OF1 was not
		 * programmed, so do not require the hardware flag.
		 */
		data->late_alarm_pending = false;
		data->alarm_callback = NULL;
	} else if ((status & kGPT_OutputCompare1Flag) && alarm_cb) {
		GPT_DisableInterrupts(base,
				      kGPT_OutputCompare1InterruptEnable);
		data->alarm_callback = NULL;
	} else {
		alarm_cb = NULL;
	}
	k_spin_unlock(&data->lock, key);

	if (alarm_cb) {
		alarm_cb(dev, 0, current, alarm_user_data);
	}

	if ((status & kGPT_RollOverFlag) && data->top_callback) {
		data->top_callback(dev, data->top_user_data);
	}
}

static uint32_t mcux_gpt_get_pending_int(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
	GPT_Type *base = get_base(dev);
	struct mcux_gpt_data *data = dev->data;

	if (GPT_GetStatusFlags(base, kGPT_OutputCompare1Flag) != 0U) {
		return 1U;
	}

	/* Covers the late-alarm path: OCR1 is left unprogrammed and the
	 * callback is instead delivered via a software-pended IRQ, so the
	 * hardware flag above never gets set for it.
	 */
	if (data->late_alarm_pending && mcux_gpt_irq_is_pending(config->irq)) {
		return 1U;
	}

	return 0U;
}

static int mcux_gpt_set_top_value(const struct device *dev,
				  const struct counter_top_cfg *cfg)
{
	const struct mcux_gpt_config *config = dev->config;
	GPT_Type *base = get_base(dev);
	struct mcux_gpt_data *data = dev->data;

	if (cfg->ticks != config->info.max_top_value) {
		LOG_ERR("Wrap can only be set to 0x%x",
			config->info.max_top_value);
		return -ENOTSUP;
	}

	data->top_callback = cfg->callback;
	data->top_user_data = cfg->user_data;

	GPT_EnableInterrupts(base, kGPT_RollOverFlagInterruptEnable);

	return 0;
}

static uint32_t mcux_gpt_get_top_value(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;

	return config->info.max_top_value;
}

static int mcux_gpt_reset(const struct device *dev)
{
	GPT_Type *base = get_base(dev);
	bool was_free_run = (base->CR & GPT_CR_FRR_MASK) != 0;
	uint32_t saved_ir;
	bool of1_before;

	if (was_free_run) {
		/* Snapshot the OF1 state. */
		of1_before = (GPT_GetStatusFlags(base, kGPT_OutputCompare1Flag) != 0U);
		/* Save and disable all GPT interrupts to guard against
		 * a spurious OCR1 match during the Restart mode window.
		 */
		saved_ir = base->IR;
		base->IR = 0U;
		/* Switch to Restart mode (FRR=0) to enable the OCR1
		 * write-reset mechanism.
		 */
		base->CR &= ~GPT_CR_FRR_MASK;
	}

	/* The GPT hardware resets CNT to 0 on any write to OCR1
	 * when operating in Restart mode.
	 */
	base->OCR[0] = base->OCR[0];

	if (was_free_run) {
		/* Restore Free-Run mode. */
		base->CR |= GPT_CR_FRR_MASK;

		/* If OF1 was not set before but is set now, clear it
		 * so it does not fire as a false alarm when IR is restored.
		 */
		if (!of1_before &&
		    (GPT_GetStatusFlags(base, kGPT_OutputCompare1Flag) != 0U)) {
			GPT_ClearStatusFlags(base, kGPT_OutputCompare1Flag);
		}

		/* Restore interrupts. */
		base->IR = saved_ir;
	}

	return 0;
}

static int mcux_gpt_init(const struct device *dev)
{
	const struct mcux_gpt_config *config = dev->config;
	gpt_config_t gptConfig;
	uint32_t clock_freq;
	GPT_Type *base;

	DEVICE_MMIO_NAMED_MAP(dev, gpt_mmio, K_MEM_CACHE_NONE | K_MEM_DIRECT_MAP);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	/* Adjust divider to match expected freq */
	if (clock_freq % config->info.freq) {
		LOG_ERR("Cannot Adjust GPT freq to %u\n", config->info.freq);
		LOG_ERR("clock src is %u\n", clock_freq);
		return -EINVAL;
	}

	GPT_GetDefaultConfig(&gptConfig);
	gptConfig.enableFreeRun = config->enable_free_run;
	gptConfig.clockSource = kGPT_ClockSource_Periph;
	gptConfig.divider = clock_freq / config->info.freq;
	base = get_base(dev);
	GPT_Init(base, &gptConfig);

	config->irq_config_func();

	return 0;
}

static DEVICE_API(counter, mcux_gpt_driver_api) = {
	.start = mcux_gpt_start,
	.stop = mcux_gpt_stop,
	.get_value = mcux_gpt_get_value,
	.set_alarm = mcux_gpt_set_alarm,
	.cancel_alarm = mcux_gpt_cancel_alarm,
	.set_top_value = mcux_gpt_set_top_value,
	.get_pending_int = mcux_gpt_get_pending_int,
	.get_top_value = mcux_gpt_get_top_value,
	.get_guard_period = mcux_gpt_get_guard_period,
	.set_guard_period = mcux_gpt_set_guard_period,
	.reset = mcux_gpt_reset,
};

#define GPT_DEVICE_INIT_MCUX(n)						\
	static struct mcux_gpt_data mcux_gpt_data_ ## n;		\
	static void mcux_gpt_irq_config_ ## n(void);			\
									\
	static const struct mcux_gpt_config mcux_gpt_config_ ## n = {	\
		DEVICE_MMIO_NAMED_ROM_INIT(gpt_mmio, DT_DRV_INST(n)),	\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clock_subsys =						\
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
		.enable_free_run = (DT_INST_ENUM_IDX_OR(n, run_mode, 0) == 1),\
		.irq = DT_INST_IRQN(n),\
		.info = {						\
			.max_top_value = UINT32_MAX,			\
			.freq = DT_INST_PROP(n, gptfreq),           \
			.channels = 1,					\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,		\
		},							\
		.irq_config_func = mcux_gpt_irq_config_ ## n,		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			    mcux_gpt_init,				\
			    NULL,					\
			    &mcux_gpt_data_ ## n,			\
			    &mcux_gpt_config_ ## n,			\
			    POST_KERNEL,				\
			    CONFIG_COUNTER_INIT_PRIORITY,		\
			    &mcux_gpt_driver_api);			\
									\
	static void mcux_gpt_irq_config_ ## n(void)			\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    mcux_gpt_isr, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));				\
	}								\

DT_INST_FOREACH_STATUS_OKAY(GPT_DEVICE_INIT_MCUX)
