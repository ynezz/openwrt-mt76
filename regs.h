/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MT76_REGS_H
#define __MT76_REGS_H

#include <linux/bitops.h>

#ifndef GENMASK
#define GENMASK(h, l)       (((U32_C(1) << ((h) - (l) + 1)) - 1) << (l))
#endif

struct mt76_queue_regs {
	u32 desc_base;
	u32 ring_size;
	u32 cpu_idx;
	u32 dma_idx;
} __packed __aligned(4);

enum mt76_cipher_type {
	MT_CIPHER_NONE,
	MT_CIPHER_WEP40,
	MT_CIPHER_WEP104,
	MT_CIPHER_TKIP,
	MT_CIPHER_AES_CCMP,
	MT_CIPHER_CKIP40,
	MT_CIPHER_CKIP104,
	MT_CIPHER_CKIP128,
	MT_CIPHER_WAPI,
};

#endif
