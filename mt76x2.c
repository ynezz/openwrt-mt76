/*
 * Copyright (C) 2015 Felix Fietkau <nbd@openwrt.org>
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
#include "mt76.h"

int mt76x2_init_device(struct mt76_dev *dev)
{
	void *status_fifo;
	int fifo_size;

	dev->rev = mt76_rr(dev, MT_ASIC_VERSION);

	fifo_size = roundup_pow_of_two(32 * sizeof(struct mt76_tx_status));
	status_fifo = devm_kzalloc(dev->dev, fifo_size, GFP_KERNEL);
	if (!status_fifo)
		return -ENOMEM;

	kfifo_init(&dev->txstatus_fifo, status_fifo, fifo_size);

	return 0;
}
