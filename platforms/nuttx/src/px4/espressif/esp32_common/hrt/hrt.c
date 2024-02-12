/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_hrt.c
 *
 * High-resolution timer callouts and timekeeping.
 *
 * RP2040's internal 64 bit timer can be used for this purpose.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX RP2040 driver per se; rather, we
 * claim the timer and then drive it directly.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <nuttx/queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/log.h>

#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_tim.h"

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#if !defined(CONFIG_BUILD_FLAT)
#include <px4_platform_common/defines.h>
#include <px4_platform/board_ctrl.h>
#include <px4_platform_common/sem.h>

#define HRT_ENTRY_QUEUE_MAX_SIZE 3
static px4_sem_t g_wait_sem;
static struct hrt_call *next_hrt_entry[HRT_ENTRY_QUEUE_MAX_SIZE];
static int hrt_entry_queued = 0;
static bool suppress_entry_queue_error = false;
static bool hrt_entry_queue_error = false;

void hrt_usr_call(void *arg)
{
	// This is called from hrt interrupt
	if (hrt_entry_queued < HRT_ENTRY_QUEUE_MAX_SIZE) {
		next_hrt_entry[hrt_entry_queued++] = (struct hrt_call *)arg;

	} else {
		hrt_entry_queue_error = true;
	}

	px4_sem_post(&g_wait_sem);
}

#endif



#ifdef HRT_TIMER
#define ESP32S3_HRT_TIMER HRT_TIMER
#define ESP32S3_HRT_TIMER_PRESCALER (APB_CLK_FREQ / (1000 * 1000))
/* HRT configuration */
#if   HRT_TIMER == 0
# define HRT_TIMER_BASE		0x6001F000
# if CONFIG_ESP32S3_TIM1
#  error must not set CONFIG_ESP32S3_TIM0=y and HRT_TIMER=0
# endif
#elif HRT_TIMER == 1
# define HRT_TIMER_BASE		0x60020000
# if CONFIG_ESP32S3_TIM2
#  error must not set CONFIG_ESP32S3_TIM1=y and HRT_TIMER=1
# endif
#elif HRT_TIMER == 2
# define HRT_TIMER_BASE		0x60020000
# if CONFIG_ESP32S3_TIM2
#  error must not set CONFIG_ESP32S3_TIM2=y and HRT_TIMER=2
# endif
#elif HRT_TIMER == 3
# define HRT_TIMER_BASE		0x6001F000
# if CONFIG_ESP32_TIM3
#  error must not set CONFIG_ESP32_TIM3=y and HRT_TIMER=3
# endif
#else
# error HRT_TIMER must be a value between 0 and 3
#endif



#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))


#define rLO 		REG(TIM_LO_OFFSET)
#define rHI 		REG(TIM_HI_OFFSET)
#define rUPDATE 	REG(TIM_UPDATE_OFFSET)
#define rALARMLO 	REG(TIMG_ALARM_LO_OFFSET)
#define rALARMHI 	REG(TIMG_ALARM_HI_OFFSET)

/**
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 16-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	50000

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD	65536

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Timer register accessors
 */
// #define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

// #define rCNT     	((uint64_t)((REG(TIM_HI_OFFSET) << 32) | REG(TIM_LO_OFFSET)))


/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint16_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint16_t			latency_actual;

/* latency histogram */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

/* timer-specific functions */
static void		hrt_tim_init(void);
static int		hrt_tim_isr(int irq, void *context, void *arg);
// static void		hrt_latency_update(void);

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);

struct esp32s3_tim_dev_s *tim;

int hrt_ioctl(unsigned int cmd, unsigned long arg);
/**
 * Initialise the timer we are going to use.
 */
static void
hrt_tim_init(void)
{

	tim = esp32s3_tim_init(ESP32S3_HRT_TIMER);

	if (!tim)
	{
		PX4_ERR("ERROR: Failed to initialize ESP32S3 timer\n");
	}

	ESP32S3_TIM_SETPRE(tim, ESP32S3_HRT_TIMER_PRESCALER);
	ESP32S3_TIM_SETMODE(tim, ESP32S3_TIM_MODE_UP);
	ESP32S3_TIM_CLEAR(tim);

	ESP32S3_TIM_SETCTR(tim, 0); //set counter value
	ESP32S3_TIM_RLD_NOW(tim);   //reload value now

	ESP32S3_TIM_SETALRVL(tim, 1000);		//alarm value
        ESP32S3_TIM_SETALRM(tim, true);		//enable alarm
	ESP32S3_TIM_SETARLD(tim, false);		//auto reload

	ESP32S3_TIM_SETISR(tim, hrt_tim_isr, NULL);
	ESP32S3_TIM_ENABLEINT(tim);
	ESP32S3_TIM_START(tim);
}

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int IRAM_ATTR
hrt_tim_isr(int irq, void *context, void *arg)
{

	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	// /* and schedule the next interrupt */
	hrt_call_reschedule();

	// hrt_abstime set = hrt_absolute_time() + 100;

	// rALARMLO = (uint32_t)(set & 0xffffffff);
	// rALARMHI = (uint32_t)((set >> 32) & 0xffffffff);

	ESP32S3_TIM_ACKINT(tim);
        ESP32S3_TIM_SETALRM(tim, true);			//enable alarm

// (*(volatile uint32_t *)(0x3FF4400C) = (1<<14));//LOW
// (*(volatile uint32_t *)(0x3FF44008) = (1<<14));//HIGH

// (*(volatile uint32_t *)(0x3FF44008) = (1<<4));//HIGH
	return OK;
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime IRAM_ATTR
hrt_absolute_time(void)
{
	hrt_abstime	abstime;
	//uint32_t	count;
	irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	//static volatile hrt_abstime base_time;
	//static volatile uint32_t last_count;

	/* prevent re-entry */
	flags = px4_enter_critical_section();
	uint64_t count;
	tim->ops->getcounter(tim,&count);


	abstime = (hrt_abstime)(count);

	/* get the current counter value */
	// rUPDATE = 1;
	// count = rLO;

	/*
	 * Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	// if (count < last_count) {
	// 	base_time += HRT_COUNTER_PERIOD;
	// }

	// /* save the count for next time */
	// last_count = count;

	// /* compute the current time */
	// abstime = HRT_COUNTER_SCALE(base_time + count);

	px4_leave_critical_section(flags);

	return abstime;
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
void
hrt_store_absolute_time(volatile hrt_abstime *t)
{
	irqstate_t flags = px4_enter_critical_section();
	*t = hrt_absolute_time();
	px4_leave_critical_section(flags);
}

/**
 * Initialise the high-resolution timing module.
 */
void
hrt_init(void)
{
	sq_init(&callout_queue);
	hrt_tim_init();

#if !defined(CONFIG_BUILD_FLAT)
	/* Create a semaphore for handling hrt driver callbacks */
	px4_sem_init(&g_wait_sem, 0, 0);
	/* this is a signalling semaphore */
	px4_sem_setprotocol(&g_wait_sem, SEM_PRIO_NONE);

	/* register ioctl callbacks */
	px4_register_boardct_ioctl(_HRTIOCBASE, hrt_ioctl);
#endif
}

/**
 * Call callout(arg) after interval has elapsed.
 */
void __attribute__ ((section(".iram1")))
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.
 */
void __attribute__ ((section(".iram1")))
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void __attribute__ ((section(".iram1")))
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void __attribute__ ((section(".iram1")))
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);
	px4_leave_critical_section(flags);
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool __attribute__ ((section(".iram1")))
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void __attribute__ ((section(".iram1")))
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	px4_leave_critical_section(flags);
}

static void __attribute__ ((section(".iram1")))
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				hrtinfo("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	hrtinfo("scheduled\n");
}

static void __attribute__ ((section(".iram1")))
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		hrtinfo("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			hrtinfo("call %p: %p(%p)\n", call, call->callout, call->arg);

// (*(volatile uint32_t *)(0x3FF4400C) = (1<<2));//LOW
// (*(volatile uint32_t *)(0x3FF44008) = (1<<2));//HIGH
			call->callout(call->arg);
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void __attribute__ ((section(".iram1")))
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 16 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			hrtinfo("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			hrtinfo("due soon\n");
			deadline = next->deadline;
		}
	}

	hrtinfo("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	//rALARMLO = latency_baseline = deadline & 0xffff;
	//rALARMLO = (uint32_t)(deadline & 0xffffffff);
	//rALARMHI = (uint32_t)((deadline >> 32) & 0xffffffff);
	tim->ops->setalarmvalue(tim,deadline);
}

// static void
// hrt_latency_update(void)
// {
// 	uint16_t latency = latency_actual - latency_baseline;
// 	unsigned	index;

// 	/* bounded buckets */
// 	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
// 		if (latency <= latency_buckets[index]) {
// 			latency_counters[index]++;
// 			return;
// 		}
// 	}

// 	/* catch-all at the end */
// 	latency_counters[index]++;
// }

void __attribute__ ((section(".iram1")))
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void __attribute__ ((section(".iram1")))
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

#if !defined(CONFIG_BUILD_FLAT)
/* These functions are inlined in all but NuttX protected/kernel builds */

latency_info_t get_latency(uint16_t bucket_idx, uint16_t counter_idx)
{
	latency_info_t ret = {latency_buckets[bucket_idx], latency_counters[counter_idx]};
	return ret;
}

void reset_latency_counters(void)
{
	for (int i = 0; i <= get_latency_bucket_count(); i++) {
		latency_counters[i] = 0;
	}
}

/* board_ioctl interface for user-space hrt driver */
int
hrt_ioctl(unsigned int cmd, unsigned long arg)
{
	hrt_boardctl_t *h = (hrt_boardctl_t *)arg;

	switch (cmd) {
	case HRT_WAITEVENT: {
			irqstate_t flags;
			px4_sem_wait(&g_wait_sem);
			/* Atomically update the pointer to user side hrt entry */
			flags = px4_enter_critical_section();

			/* This should be always true, but check it anyway */
			if (hrt_entry_queued > 0) {
				*(struct hrt_call **)arg = next_hrt_entry[--hrt_entry_queued];
				next_hrt_entry[hrt_entry_queued] = NULL;

			} else {
				hrt_entry_queue_error = true;
			}

			px4_leave_critical_section(flags);

			/* Warn once for entry queue being full */
			if (hrt_entry_queue_error && !suppress_entry_queue_error) {
				PX4_ERR("HRT entry error, queue size now %d", hrt_entry_queued);
				suppress_entry_queue_error = true;
			}
		}
		break;

	case HRT_ABSOLUTE_TIME:
		*(hrt_abstime *)arg = hrt_absolute_time();
		break;

	case HRT_CALL_AFTER:
		hrt_call_after(h->entry, h->time, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CALL_AT:
		hrt_call_at(h->entry, h->time, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CALL_EVERY:
		hrt_call_every(h->entry, h->time, h->interval, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CANCEL:
		if (h && h->entry) {
			hrt_cancel(h->entry);

		} else {
			PX4_ERR("HRT_CANCEL called with NULL entry");
		}

		break;

	case HRT_GET_LATENCY: {
			latency_boardctl_t *latency = (latency_boardctl_t *)arg;
			latency->latency = get_latency(latency->bucket_idx, latency->counter_idx);
		}
		break;

	case HRT_RESET_LATENCY:
		reset_latency_counters();
		break;

	default:
		return -EINVAL;
	}

	return OK;
}
#endif

#endif /* HRT_TIMER */
