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
#include "mt76x2.h"

struct beacon_bc_data {
	struct mt76_dev *dev;
	struct sk_buff_head q;
	struct sk_buff *tail[8];
};

static int
mt76x2_write_beacon(struct mt76_dev *dev, int offset, struct sk_buff *skb)
{
	int beacon_len = dev->beacon_offsets[1] - dev->beacon_offsets[0];
	struct mt76_txwi txwi;

	if (WARN_ON_ONCE(beacon_len < skb->len + sizeof(struct mt76_txwi)))
		return -ENOSPC;

	mt76_mac_write_txwi(dev, &txwi, skb, NULL, NULL);
	txwi.flags |= cpu_to_le16(MT_TXWI_FLAGS_TS);

	mt76_wr_copy(dev, offset, &txwi, sizeof(txwi));
	offset += sizeof(txwi);

	mt76_wr_copy(dev, offset, skb->data, skb->len);
	return 0;
}

static int
__mt76x2_mac_set_beacon(struct mt76_dev *dev, u8 bcn_idx, struct sk_buff *skb)
{
	int beacon_len = dev->beacon_offsets[1] - dev->beacon_offsets[0];
	int beacon_addr = dev->beacon_offsets[bcn_idx];
	int ret = 0;
	int i;

	/* Prevent corrupt transmissions during update */
	mt76_set(dev, MT_BCN_BYPASS_MASK, BIT(bcn_idx));

	if (skb) {
		ret = mt76x2_write_beacon(dev, beacon_addr, skb);
		if (!ret)
			dev->beacon_data_mask |= BIT(bcn_idx) & dev->beacon_mask;
	} else {
		dev->beacon_data_mask &= ~BIT(bcn_idx);
		for (i = 0; i < beacon_len; i += 4)
			mt76_wr(dev, beacon_addr + i, 0);
	}

	mt76_wr(dev, MT_BCN_BYPASS_MASK, 0xff00 | ~dev->beacon_data_mask);

	return ret;
}

int mt76x2_mac_set_beacon(struct mt76_dev *dev, u8 vif_idx, struct sk_buff *skb)
{
	bool force_update = false;
	int bcn_idx = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->beacons); i++) {
		if (vif_idx == i) {
			force_update = !!dev->beacons[i] ^ !!skb;

			if (dev->beacons[i])
				dev_kfree_skb(dev->beacons[i]);

			dev->beacons[i] = skb;
			__mt76x2_mac_set_beacon(dev, bcn_idx, skb);
		} else if (force_update && dev->beacons[i]) {
			__mt76x2_mac_set_beacon(dev, bcn_idx, dev->beacons[i]);
		}

		bcn_idx += !!dev->beacons[i];
	}

	for (i = bcn_idx; i < ARRAY_SIZE(dev->beacons); i++) {
		if (!(dev->beacon_data_mask & BIT(i)))
			break;

		__mt76x2_mac_set_beacon(dev, i, NULL);
	}

	mt76_rmw_field(dev, MT_MAC_BSSID_DW1, MT_MAC_BSSID_DW1_MBEACON_N, bcn_idx - 1);
	return 0;
}

void
mt76x2_mac_set_beacon_enable(struct mt76_dev *dev, u8 vif_idx, bool val)
{
	u8 old_mask = dev->beacon_mask;
	bool en;
	u32 reg;

	if (val) {
		dev->beacon_mask |= BIT(vif_idx);
	} else {
		dev->beacon_mask &= ~BIT(vif_idx);
		mt76x2_mac_set_beacon(dev, vif_idx, NULL);
	}

	if (!!old_mask == !!dev->beacon_mask)
		return;

	en = dev->beacon_mask;

	mt76_rmw_field(dev, MT_INT_TIMER_EN, MT_INT_TIMER_EN_PRE_TBTT_EN, en);
	reg = MT_BEACON_TIME_CFG_BEACON_TX |
	      MT_BEACON_TIME_CFG_TBTT_EN |
	      MT_BEACON_TIME_CFG_TIMER_EN;
	mt76_rmw(dev, MT_BEACON_TIME_CFG, reg, reg * en);

	if (en)
		mt76_irq_enable(dev, MT_INT_PRE_TBTT | MT_INT_TBTT);
	else
		mt76_irq_disable(dev, MT_INT_PRE_TBTT | MT_INT_TBTT);
}

static void
mt76_update_beacon_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt76_dev *dev = (struct mt76_dev *) priv;
	struct mt76_vif *mvif = (struct mt76_vif *) vif->drv_priv;
	struct ieee80211_tx_info *info;
	struct sk_buff *skb = NULL;

	if (!(dev->beacon_mask & BIT(mvif->idx)))
		return;

	skb = ieee80211_beacon_get(dev->hw, vif);
	if (!skb)
		return;

	info = IEEE80211_SKB_CB(skb);
	info->flags |= IEEE80211_TX_CTL_ASSIGN_SEQ;
	mt76x2_mac_set_beacon(dev, mvif->idx, skb);
}

static void
mt76_add_buffered_bc(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct beacon_bc_data *data = priv;
	struct mt76_dev *dev = data->dev;
	struct mt76_vif *mvif = (struct mt76_vif *) vif->drv_priv;
	struct ieee80211_tx_info *info;
	struct sk_buff *skb;

	if (!(dev->beacon_mask & BIT(mvif->idx)))
		return;

	skb = ieee80211_get_buffered_bc(dev->hw, vif);
	if (!skb)
		return;

	info = IEEE80211_SKB_CB(skb);
	info->control.vif = vif;
	info->flags |= IEEE80211_TX_CTL_ASSIGN_SEQ;
	mt76_skb_set_moredata(skb, true);
	__skb_queue_tail(&data->q, skb);
	data->tail[mvif->idx] = skb;
}

void
mt76x2_pre_tbtt_tasklet(unsigned long arg)
{
	struct mt76_dev *dev = (struct mt76_dev *) arg;
	struct mt76_queue *q = &dev->q_tx[MT_TXQ_PSD];
	struct beacon_bc_data data = {};
	struct sk_buff *skb;
	int i, nframes;

	data.dev = dev;
	__skb_queue_head_init(&data.q);

	ieee80211_iterate_active_interfaces_atomic(dev->hw,
		IEEE80211_IFACE_ITER_RESUME_ALL,
		mt76_update_beacon_iter, dev);

	do {
		nframes = skb_queue_len(&data.q);
		ieee80211_iterate_active_interfaces_atomic(dev->hw,
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt76_add_buffered_bc, &data);
	} while (nframes != skb_queue_len(&data.q));

	if (!nframes)
		return;

	for (i = 0; i < ARRAY_SIZE(data.tail); i++) {
		if (!data.tail[i])
			continue;

		mt76_skb_set_moredata(data.tail[i], false);
	}

	spin_lock_bh(&q->lock);
	while ((skb = __skb_dequeue(&data.q)) != NULL) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
		struct ieee80211_vif *vif = info->control.vif;
		struct mt76_vif *mvif = (struct mt76_vif *) vif->drv_priv;

		mt76_tx_queue_skb(dev, q, skb, &mvif->group_wcid, NULL);
	}
	spin_unlock_bh(&q->lock);
}
