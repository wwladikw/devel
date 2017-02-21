/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/*
 * Author: Wladislav Wiebe <wladislav.kw@gmail.com
 *
 * This driver is an experimental version for the
 * http://www.ti.com/lit/ug/sprugv5a/sprugv5a.pdf
 * timer device - fitted for the seL4 project.
 */

#include <stdio.h>
#include <assert.h>

#include <utils/util.h>

#include <platsupport/timer.h>
#include <platsupport/plat/timer.h>
#include "../../stubtimer.h"


/* Timer register bitfields */
#define TCR_ENAMODE_MASK		0xC0
#define TCR_ENAMODE_ONESHOT_MASK	0x40
#define TCR_ENAMODE_PERIODIC_MASK	0x80

#define TGCR_TIM_UNRESET_MASK		0x03
#define INTCTLSTAT_ENINT_MASK		0x01
#define INTCTLSTAT_ACK_MASK		0x03

#define TICKS_PER_SECOND 204800000
#define TIMER_INTERVAL_TICKS(ns) ((uint32_t)(1ULL * (ns) * TICKS_PER_SECOND / 1000 / 1000 / 1000))

/* Ensure completion with Data Memory Barrier */
#define dmb()     asm volatile("dmb" ::: "memory")


struct keystone_timer_map {
	char padding1[4];
	uint32_t emumgt_clkspd; /* Emulation management and clock speed register */
	char padding2[6];
	uint32_t cntlo;		/* Counter register low */
	uint32_t cnthi;		/* Counter register high */
	uint32_t prdlo;		/* Period register low */
	uint32_t prdhi;		/* Period register high */
	uint32_t tcr;		/* Timer control register */
	uint32_t tgcr;		/* Timer global control register */
	uint32_t wdtcr;		/* Watchdog timer control register */
	char padding3[6];
	uint32_t rello;		/* Timer reload register low */
	uint32_t relhi;		/* Timer reload register high */
	uint32_t caplo;		/* Timer capture register low */
	uint32_t caphi;		/* Timer reload register high */
	uint32_t intctlstat;	/* Timer interrupt control and status register */
};

typedef struct keystone_timer {
	volatile struct keystone_timer_map *hw;
	uint32_t irq;
} keystone_timer_t;

static void
keystone_timer_reset(const pstimer_t *timer)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;

	/* disable, use internal clock source */
	kt->hw->tcr = 0;
	dmb();
	/* reset timer as 64-bit, no pre-scaler, plus features are disabled */
	kt->hw->tgcr = 0;
	/* unreset timer */
	kt->hw->tgcr = TGCR_TIM_UNRESET_MASK;

	/* init counter to zero */
	kt->hw->cntlo = 0;
	kt->hw->cnthi = 0;

	/* enable timer interrupts */
	kt->hw->intctlstat = INTCTLSTAT_ENINT_MASK;
}

static int
keystone_timer_stop(const pstimer_t *timer)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;

	kt->hw->tcr &= ~(TCR_ENAMODE_MASK);

	return 0;
}

static int
keystone_timer_start(const pstimer_t *timer)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;

	kt->hw->tcr |= TCR_ENAMODE_MASK;

	return 0;
}

static int
keystone_set_timeo(const pstimer_t *timer, uint64_t ns, int tcrFlags)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;
	uint32_t tcr;
	uint32_t off;
	uint64_t ticks = TIMER_INTERVAL_TICKS(ns);

	if (ticks < 2)
		return EINVAL;

	tcr = kt->hw->tcr;
	off = tcr & ~(TCR_ENAMODE_MASK);

	/* set enable mode */
	tcr |= tcrFlags;

	/* disable timer */
	kt->hw->tcr = off;
	/* here we have to be sure the timer has been disabled */
	dmb();
	/* reset counter to zero, set new period */
	kt->hw->cntlo = 0;
	kt->hw->cnthi = 0;
	kt->hw->prdlo = (ticks & 0xffffffff);
	kt->hw->prdhi = (ticks >> 32);

	/* clear interrupt status bit */
	kt->hw->intctlstat = INTCTLSTAT_ACK_MASK;

	/*
	 * enable timer
	 */
	dmb();
	kt->hw->tcr = tcr;

	return 0;
}

static int
keystone_periodic(const pstimer_t *timer, uint64_t ns)
{
	return keystone_set_timeo(timer, ns, TCR_ENAMODE_PERIODIC_MASK);
}

static int
keystone_oneshot_relative(const pstimer_t *timer, uint64_t ns)
{
	return keystone_set_timeo(timer, ns, TCR_ENAMODE_ONESHOT_MASK);
}

static uint64_t
keystone_get_time(const pstimer_t *timer)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;

	return kt->hw->cntlo;
}

static void
keystone_handle_irq(const pstimer_t *timer, uint32_t irq)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;
	kt->hw->intctlstat = INTCTLSTAT_ACK_MASK;
}

static uint32_t
keystone_get_nth_irq(const pstimer_t *timer, uint32_t n)
{
	keystone_timer_t *kt = (keystone_timer_t *)timer->data;

	return kt->irq;
}

static pstimer_t timers[NTIMERS];
static keystone_timer_t kts[NTIMERS];

pstimer_t *
ps_get_timer(enum timer_id id, timer_config_t *config)
{
	pstimer_t *timer;
	keystone_timer_t *kt;

	if (id >= NTIMERS)
		return NULL;

	timer = &timers[id];
	kt = &kts[id];

	timer->properties.upcounter = false;
	timer->properties.timeouts = true;
	timer->properties.relative_timeouts = true;
	timer->properties.periodic_timeouts = true;
	timer->properties.absolute_timeouts = false;
	timer->properties.bit_width = 32;
	timer->properties.irqs = 1;

	timer->data = kt;
	timer->start = keystone_timer_start;
	timer->stop = keystone_timer_stop;
	timer->get_time = keystone_get_time;
	timer->oneshot_absolute = stub_timer_timeout;
	timer->oneshot_relative = keystone_oneshot_relative;
	timer->periodic = keystone_periodic;
	timer->handle_irq = keystone_handle_irq;
	timer->get_nth_irq = keystone_get_nth_irq;

	kt->hw = (struct keystone_timer_map *)config->vaddr;
	kt->irq = config->irq;

	keystone_timer_reset(timer);
	return timer;
}
