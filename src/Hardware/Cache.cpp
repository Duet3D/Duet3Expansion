/*
 * Cache.cpp
 *
 *  Created on: 22 Nov 2019
 *      Author: David
 */

#include <Hardware/Cache.h>

#if USE_CACHE

#if SAME5x

inline void cache_disable()
{
	while (CMCC->SR.bit.CSTS)
	{
		CMCC->CTRL.reg = 0;
		__ISB();
		__DSB();
	}
}

inline void cache_enable()
{
	CMCC->CTRL.reg = CMCC_CTRL_CEN;
	__ISB();
	__DSB();
}

inline void cache_invalidate_all()
{
	CMCC->MAINT0.reg = CMCC_MAINT0_INVALL;
}

#endif

static bool enabled = false;

void Cache::Init() noexcept
{
	// No need to do any initialisation on SAME5x
}

void Cache::Enable() noexcept
{
	if (!enabled)
	{
		enabled = true;
		cache_invalidate_all();
		cache_enable();
	}
}

void Cache::Disable() noexcept
{
	if (enabled)
	{
		cache_disable();
		enabled = false;
	}
}

void Cache::Invalidate(const volatile void *start, size_t length) noexcept
{
	if (enabled)
	{
		//TODO can we invalidate just the relevant cache line(s)?
		// Disable interrupts throughout the sequence, otherwise we could get a task switch while the cache is disabled.
		// The can cause a crash because we end up invalidating the cache while it is enabled.
		const irqflags_t flags = cpu_irq_save();
		cache_disable();
		cache_invalidate_all();
		cache_enable();
		cpu_irq_restore(flags);
	}
}

#endif

// Entry points that can be called from ASF C code
void CacheFlushBeforeDMAReceive(const volatile void *start, size_t length) noexcept { Cache::FlushBeforeDMAReceive(start, length); }
void CacheInvalidateAfterDMAReceive(const volatile void *start, size_t length) noexcept { Cache::InvalidateAfterDMAReceive(start, length); }
void CacheFlushBeforeDMASend(const volatile void *start, size_t length) noexcept { Cache::FlushBeforeDMASend(start, length); }

// End
