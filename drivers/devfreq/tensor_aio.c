// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#include <linux/cpufreq.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/of_platform.h>
#include <linux/perf_event.h>
#include <linux/reboot.h>
#include <linux/units.h>
#include <linux/sched/clock.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/sched.h>
#include <sched.h>
#include <pelt.h>


/* Poll memperfd about every 10 ms */
#define MEMPERFD_POLL_HZ (HZ / 100)


/*
 * The minimum sample time required to measure the cycle counters. This should
 * take into account the time needed to read the monotonic clock.
 */
#define CPU_MIN_SAMPLE_NS (100 * NSEC_PER_USEC)


/* The PMU (Performance Monitor Unit) event statistics */
struct pmu_stat {
	u64 cpu_cyc;
	u64 ns;
};

struct cpu_pmu {
	raw_spinlock_t lock;
	struct pmu_stat cur;
	struct pmu_stat prev;
	struct pmu_stat sfd;
};

static DEFINE_PER_CPU(struct cpu_pmu, cpu_pmu_evs) = {
	.lock = __RAW_SPIN_LOCK_UNLOCKED(cpu_pmu_evs.lock)
};


static atomic_long_t last_run_jiffies = ATOMIC_INIT(0);
static DECLARE_SWAIT_QUEUE_HEAD(memperfd_waitq);
static unsigned int dsu_scale_factor __read_mostly __maybe_unused;
static bool in_reboot __read_mostly;
static int cpuhp_state;

enum pmu_events {
	CPU_CYCLES,
	PMU_EVT_MAX
};

static const u32 pmu_evt_id[PMU_EVT_MAX] = {
	[CPU_CYCLES] = ARMV8_PMUV3_PERFCTR_CPU_CYCLES
};

struct cpu_pmu_evt {
	struct perf_event *pev[PMU_EVT_MAX];
};

static DEFINE_PER_CPU(struct cpu_pmu_evt, pevt_pcpu);

static struct perf_event *create_pev(struct perf_event_attr *attr, int cpu)
{
	return perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
}

static void release_perf_events(int cpu)
{
	struct cpu_pmu_evt *cpev = &per_cpu(pevt_pcpu, cpu);
	int i;

	for (i = 0; i < PMU_EVT_MAX; i++) {
		if (IS_ERR(cpev->pev[i]))
			break;

		perf_event_release_kernel(cpev->pev[i]);
	}
}

static int create_perf_events(int cpu)
{
	struct cpu_pmu_evt *cpev = &per_cpu(pevt_pcpu, cpu);
	struct perf_event_attr attr = {
		.type = PERF_TYPE_RAW,
		.size = sizeof(attr),
		.pinned = 1
	};
	int i;

	for (i = 0; i < PMU_EVT_MAX; i++) {
		attr.config = pmu_evt_id[i];
		cpev->pev[i] = create_pev(&attr, cpu);
		if (WARN_ON(IS_ERR(cpev->pev[i])))
			goto release_pevs;
	}

	return 0;

release_pevs:
	release_perf_events(cpu);
	return PTR_ERR(cpev->pev[i]);
}

static u64 read_perf_event(enum pmu_events evt)
{
	struct cpu_pmu_evt *cpev = this_cpu_ptr(&pevt_pcpu);
	struct perf_event *event = cpev->pev[evt];
	u64 value;

#ifdef SYS_AMEVCNTR0_CORE_EL0
	/* Read the AMU registers directly for better speed and precision */
	switch (evt) {
	case CPU_CYCLES:
		return read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
	default:
		break;
	}
#endif

	/* Do a raw read of the PMU event to go as fast as possible */
	event->pmu->read(event);
	value = local64_read(&event->count);
	return value;
}

static inline u64 get_time_ns(void)
{
	/* sched_clock() is fine so long as times aren't compared across CPUs */
	return sched_clock();
}

static void pmu_read_events(struct pmu_stat *stat)
{
	stat->cpu_cyc = read_perf_event(CPU_CYCLES);
}

static void pmu_get_stats(struct pmu_stat *stat)
{
	pmu_read_events(stat);
	stat->ns = get_time_ns();
}

static void kick_memperfd(void)
{
	unsigned long prev, now = jiffies;

	prev = atomic_long_read(&last_run_jiffies);
	if (time_before(now, prev + MEMPERFD_POLL_HZ))
		return;

	if (atomic_long_cmpxchg_relaxed(&last_run_jiffies, prev, now) != prev)
		return;

	/* Ensure the relaxed cmpxchg is ordered before the swait_active() */
	smp_acquire__after_ctrl_dep();
	if (swait_active(&memperfd_waitq))
		swake_up_one(&memperfd_waitq);
}


static void update_freq_scale(bool tick)
{
	int cpu = raw_smp_processor_id();
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct pmu_stat cur, prev = pmu->cur, *sfd = &pmu->sfd;
	u64 freq, max_freq;

	u64 max_frequencies[] ={2016000,2016000,2016000,2803200,2803200,2803200,2803200,3187200};

	/* Check if enough time has passed to take a new sample */
	cur.ns = get_time_ns();
	if ((cur.ns - prev.ns) >= CPU_MIN_SAMPLE_NS) {
		/* Update the PMU counters without rereading the current time */
		pmu_read_events(&cur);
		raw_spin_lock(&pmu->lock);
		pmu->cur = cur;
		raw_spin_unlock(&pmu->lock);

		/* Accumulate more data for calculating the CPU's frequency */
		sfd->cpu_cyc += cur.cpu_cyc - prev.cpu_cyc;
		sfd->ns += cur.ns - prev.ns;
	}

	/*
	 * Set the CPU frequency scale measured via counters if enough data is
	 * present. This excludes idle time because although the cycle counter
	 * stops incrementing while the CPU idles, the monotonic clock doesn't.
	 */
	if (sfd->ns >= CPU_MIN_SAMPLE_NS) {
		max_freq = max_frequencies[cpu];
		freq = min(max_freq, USEC_PER_SEC * sfd->cpu_cyc / sfd->ns);
		per_cpu(arch_freq_scale, cpu) =
			SCHED_CAPACITY_SCALE * freq / max_freq;
		sfd->cpu_cyc = sfd->ns = 0;
	} else if (tick) {
		/* Reset the accumulated sfd stats on every scheduler tick */
		sfd->cpu_cyc = sfd->ns = 0;
	}

}

/*
 * The scheduler tick is used as a passive way to collect statistics on all
 * CPUs. Collecting statistics with per-CPU timers would result in the cpuidle
 * governor predicting imminent wakeups and thus selecting a shallower idle
 * state, to the detriment of power consumption. When CPUs aren't active,
 * there's no need to collect any statistics, so memperfd is designed to only
 * run when there's CPU activity.
 */
static void tensor_aio_tick(void)
{
	update_freq_scale(true);
	kick_memperfd();
}

static struct scale_freq_data tensor_aio_sfd = {
	.source = SCALE_FREQ_SOURCE_ARCH,
	.set_freq_scale = tensor_aio_tick
};

/*
 * try_to_wake_up() is probed in order to poll the TMU more often to update the
 * thermal pressure, as well as measure CPU frequency more finely. Otherwise, a
 * stale thermal pressure or CPU frequency measurement result from the scheduler
 * tick could take up to one jiffy to correct itself, which is unacceptably long
 * and results in poor scheduling decisions in the meantime. This probes TTWU
 * just before it tries to select a runqueue, updating the thermal load average
 * and CPU frequency scale right before select_task_rq() so that it can make a
 * more informed scheduling decision.
 */
static void tensor_aio_ttwu(void *data, struct task_struct *p)
{
	int cpu = raw_smp_processor_id();



	/* Don't race with CPU hotplug or reboot */
	if (unlikely(in_reboot || !cpu_active(cpu)))
		return;

	update_freq_scale(false);

}

static void tensor_aio_idle_enter(void *data, int *state,
				  struct cpuidle_device *dev)
{
	int cpu = raw_smp_processor_id();
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct pmu_stat cur, prev = pmu->cur;

	/* Don't race with CPU hotplug which creates/destroys the perf events */
	if (unlikely(in_reboot || !cpu_active(cpu)))
		return;

	/* Update the current counters one last time before idling */
	pmu_get_stats(&cur);
	raw_spin_lock(&pmu->lock);
	pmu->cur = cur;
	raw_spin_unlock(&pmu->lock);

	/* Accumulate the cycles/ns for calculating the CPU's frequency */
	pmu->sfd.cpu_cyc += cur.cpu_cyc - prev.cpu_cyc;
	pmu->sfd.ns += cur.ns - prev.ns;
}

static void tensor_aio_idle_exit(void *data, int state,
				 struct cpuidle_device *dev)
{
	int cpu = raw_smp_processor_id();
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	struct pmu_stat cur;

	/* Don't race with CPU hotplug or reboot */
	if (unlikely(in_reboot || !cpu_active(cpu))) {
		/* Reset the sfd statistics since they'll be wrong */
		pmu->sfd.cpu_cyc = pmu->sfd.ns = 0;
		return;
	}

	/* Update the current counters without updating sfd (scale_freq_data) */
	pmu_get_stats(&cur);
	raw_spin_lock(&pmu->lock);
	pmu->cur = cur;
	raw_spin_unlock(&pmu->lock);
}

static int memperf_cpuhp_up(unsigned int cpu)
{
	struct cpu_pmu *pmu = &per_cpu(cpu_pmu_evs, cpu);
	int ret;

	ret = create_perf_events(cpu);
	if (ret)
		return ret;

	/*
	 * Update and reset the statistics for this CPU as it comes online. No
	 * need to disable interrupts since tensor_aio_tick() isn't running yet,
	 * so pmu->lock can't be acquired from hard IRQ context right now.
	 */
	raw_spin_lock(&pmu->lock);
	pmu_get_stats(&pmu->cur);
	pmu->prev = pmu->cur;
	raw_spin_unlock(&pmu->lock);

	/* Reset the sfd statistics */
	pmu->sfd.cpu_cyc = pmu->sfd.ns = 0;

	/* Install tensor_aio_tick() */
	topology_set_scale_freq_source(&tensor_aio_sfd, cpumask_of(cpu));
	return 0;
}

static int memperf_cpuhp_down(unsigned int cpu)
{
	/* Stop tensor_aio_tick() from running on this CPU anymore */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpumask_of(cpu));
	release_perf_events(cpu);
	return 0;
}


static void memperfd_init(void)
{
	/*
	 * Delete the arch's scale_freq_data callback to get rid of the
	 * duplicated work by the arch's callback, since we read the same
	 * values. This also lets the frequency invariance engine work on cores
	 * with an erratum that breaks the const cycles PMU counter, since we
	 * don't use const cycles. A new scale_freq_data callback is installed
	 * in memperf_cpuhp_up().
	 */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_ARCH,
					 cpu_possible_mask);

	/* Register the CPU hotplug notifier with calls to all online CPUs */
	cpuhp_state = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "memperf",
					memperf_cpuhp_up, memperf_cpuhp_down);
	BUG_ON(cpuhp_state <= 0);

	/*
	 * Register the cpuidle callback for frequency-invariant counting needed
	 * to set the CPU frequency scale correctly in update_freq_scale().
	 */
	BUG_ON(register_trace_android_vh_cpu_idle_enter(tensor_aio_idle_enter,
							NULL));
	BUG_ON(register_trace_android_vh_cpu_idle_exit(tensor_aio_idle_exit,
						       NULL));

	/*
	 * Register a TTWU callback as well to update thermal pressure right
	 * before select_task_rq() checks the thermal pressure.
	 */
	BUG_ON(register_trace_android_rvh_try_to_wake_up(tensor_aio_ttwu,
							 NULL));

}


/* Returns true if memperfd should arm a timeout to vote down upon inactivity */
static bool memperf_work(void)
{
	struct cpu_pmu *pmu;
	bool ret = false;
	cpumask_t cpus;
	int cpu;

	/* Only consider active CPUs */
	cpumask_copy(&cpus, cpu_active_mask);

	/* Gather updated memory stall statistics for all active CPUs */
	for_each_cpu(cpu, &cpus) {
		struct pmu_stat stat;

		/* Calculate the delta for each statistic */
		pmu = &per_cpu(cpu_pmu_evs, cpu);
		raw_spin_lock_irq(&pmu->lock);
		if ((pmu->cur.ns - pmu->prev.ns) >= CPU_MIN_SAMPLE_NS) {
			stat.cpu_cyc = pmu->cur.cpu_cyc - pmu->prev.cpu_cyc;
			stat.ns = pmu->cur.ns - pmu->prev.ns;
		} else {
			/* Indicate that this CPU should be skipped */
			stat.cpu_cyc = 0;
		}
		raw_spin_unlock_irq(&pmu->lock);

		/*
		 * Skip CPUs with incomplete statistics, like CPUs that have
		 * been idle for a while and thus have had their tick suspended.
		 */
		if (!stat.cpu_cyc || !stat.ns)
			continue;
	}


	/*
	 * Reset the statistics for all CPUs by setting the start of the next
	 * sample window to the current counter values.
	 */
	for_each_cpu(cpu, &cpus) {
		pmu = &per_cpu(cpu_pmu_evs, cpu);
		raw_spin_lock_irq(&pmu->lock);
		pmu->prev = pmu->cur;
		raw_spin_unlock_irq(&pmu->lock);
	}

	return ret;
}

static void memperfd_timeout(struct timer_list *t)
{
	/*
	 * Wake up memperfd so it can vote down to the lowest state. This is
	 * done in order to prevent MIF from staying at a higher frequency than
	 * necessary and never getting a chance to vote down just because there
	 * aren't any scheduler ticks, which is how memperfd is normally driven.
	 */
	kick_memperfd();
}

static void memperfd_wait_timeout(void)
{
	struct timer_list timer;

	/*
	 * Open code freezable_schedule_timeout_interruptible() in order to
	 * make the timer deferrable, so that it doesn't kick CPUs out of idle.
	 * Also, add the timer onto CPU0 since it's usually the least idle.
	 */
	freezer_do_not_count();
	timer_setup_on_stack(&timer, memperfd_timeout, TIMER_DEFERRABLE);
	timer.expires = jiffies + MEMPERFD_POLL_HZ + 1;
	add_timer_on(&timer, 0);
	schedule();
	del_singleshot_timer_sync(&timer);
	destroy_timer_on_stack(&timer);
	freezer_count();
}

static void memperfd_wait_for_kick(bool timeout)
{
	unsigned long prev_jiffies = jiffies;
	DECLARE_SWAITQUEUE(wait);

	/*
	 * Reset last_run_jiffies to now. prepare_to_swait_exclusive() executes
	 * a barrier that ensures the last_run_jiffies store is ordered before
	 * kick_memperfd() can observe `swait_active() == true`.
	 */
	atomic_long_set(&last_run_jiffies, prev_jiffies);
	while (1) {
		prepare_to_swait_exclusive(&memperfd_waitq, &wait, TASK_IDLE);
		if (atomic_long_read(&last_run_jiffies) != prev_jiffies)
			break;
		if (timeout)
			memperfd_wait_timeout();
		else
			freezable_schedule();
	}
	finish_swait(&memperfd_waitq, &wait);
}

static int __noreturn memperf_thread(void *data)
{
	sched_set_fifo(current);
	memperfd_init();
	set_freezable();
	while (1)
		memperfd_wait_for_kick(memperf_work());
}

static int __init fie_monitoring_init(void)
{
    BUG_ON(IS_ERR(kthread_run(memperf_thread, NULL, "memperfd")));
    printk("FIE: MONITORING INIT OK");
	return 0;

}

late_initcall(fie_monitoring_init);
