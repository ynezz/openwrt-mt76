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

#include <linux/delay.h>
#include "mt76.h"
#include "eeprom.h"
#include "mcu.h"

static bool
wait_for_wpdma(struct mt76_dev *dev)
{
	return mt76_poll(dev, MT_WPDMA_GLO_CFG,
			 MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
			 MT_WPDMA_GLO_CFG_RX_DMA_BUSY,
			 0, 1000);
}

int mt76_mac_start(struct mt76_dev *dev)
{
	int i;

	for (i = 0; i < 16; i++)
		mt76_rr(dev, MT_TX_AGG_CNT(i));

	for (i = 0; i < 16; i++)
		mt76_rr(dev, MT_TX_STAT_FIFO);

	memset(dev->aggr_stats, 0, sizeof(dev->aggr_stats));

	mt76_wr(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_TX);
	wait_for_wpdma(dev);
	udelay(50);

	mt76_set(dev, MT_WPDMA_GLO_CFG,
		 MT_WPDMA_GLO_CFG_TX_DMA_EN |
		 MT_WPDMA_GLO_CFG_RX_DMA_EN);

	mt76_clear(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE);

	mt76_wr(dev, MT_RX_FILTR_CFG, dev->rxfilter);

	mt76_wr(dev, MT_MAC_SYS_CTRL,
		MT_MAC_SYS_CTRL_ENABLE_TX |
		MT_MAC_SYS_CTRL_ENABLE_RX);

	mt76_irq_enable(dev, MT_INT_RX_DONE_ALL | MT_INT_TX_DONE_ALL | MT_INT_TX_STAT);

	return 0;
}

void mt76_mac_stop(struct mt76_dev *dev, bool force)
{
	bool stopped = false;
	u32 rts_cfg;
	int i;

	mt76_wr(dev, MT_MAC_SYS_CTRL, 0);

	rts_cfg = mt76_rr(dev, MT_TX_RTS_CFG);
	mt76_wr(dev, MT_TX_RTS_CFG, rts_cfg & ~MT_TX_RTS_CFG_RETRY_LIMIT);

	/* Wait for MAC to become idle */
	for (i = 0; i < 300; i++) {
		if (mt76_rr(dev, MT_MAC_STATUS) &
		    (MT_MAC_STATUS_RX | MT_MAC_STATUS_TX))
			continue;

		if (mt76_rr(dev, MT_BBP(IBI, 12)))
			continue;

		stopped = true;
		break;
	}

	if (force && !stopped) {
		mt76_set(dev, MT_BBP(CORE, 4), BIT(1));
		mt76_clear(dev, MT_BBP(CORE, 4), BIT(1));

		mt76_set(dev, MT_BBP(CORE, 4), BIT(0));
		mt76_clear(dev, MT_BBP(CORE, 4), BIT(0));
	}

	mt76_wr(dev, MT_TX_RTS_CFG, rts_cfg);
}

void mt76_mac_resume(struct mt76_dev *dev)
{
	mt76_wr(dev, MT_MAC_SYS_CTRL,
		MT_MAC_SYS_CTRL_ENABLE_TX |
		MT_MAC_SYS_CTRL_ENABLE_RX);
}


void mt76_set_tx_ackto(struct mt76_dev *dev)
{
	u8 ackto, sifs, slottime = dev->slottime;

	slottime += 3 * dev->coverage_class;

	sifs = mt76_get_field(dev, MT_XIFS_TIME_CFG,
			      MT_XIFS_TIME_CFG_OFDM_SIFS);

	ackto = slottime + sifs;
	mt76_rmw_field(dev, MT_TX_TIMEOUT_CFG,
		       MT_TX_TIMEOUT_CFG_ACKTO, ackto);
}

void mt76_stop_hardware(struct mt76_dev *dev)
{
	cancel_delayed_work_sync(&dev->cal_work);
	cancel_delayed_work_sync(&dev->mac_work);
	mt76_mcu_set_radio_state(dev, false);
	mt76_mac_stop(dev, false);
}

void mt76_cleanup(struct mt76_dev *dev)
{
	mt76_stop_hardware(dev);
	mt76_dma_cleanup(dev);
	mt76_mcu_cleanup(dev);
}

struct mt76_dev *mt76_alloc_device(struct device *pdev)
{
	struct ieee80211_hw *hw;
	struct mt76_dev *dev;

	hw = ieee80211_alloc_hw(sizeof(*dev), &mt76_ops);
	if (!hw)
		return NULL;

	dev = hw->priv;
	dev->dev = pdev;
	dev->hw = hw;
	mutex_init(&dev->mutex);
	spin_lock_init(&dev->lock);
	spin_lock_init(&dev->irq_lock);

	return dev;
}

#define CHAN2G(_idx, _freq) {			\
	.band = IEEE80211_BAND_2GHZ,		\
	.center_freq = (_freq),			\
	.hw_value = (_idx),			\
	.max_power = 30,			\
}

#define CHAN5G(_idx, _freq) {			\
	.band = IEEE80211_BAND_5GHZ,		\
	.center_freq = (_freq),			\
	.hw_value = (_idx),			\
	.max_power = 30,			\
}

static const struct ieee80211_channel mt76_channels_2ghz[] = {
	CHAN2G(1, 2412),
	CHAN2G(2, 2417),
	CHAN2G(3, 2422),
	CHAN2G(4, 2427),
	CHAN2G(5, 2432),
	CHAN2G(6, 2437),
	CHAN2G(7, 2442),
	CHAN2G(8, 2447),
	CHAN2G(9, 2452),
	CHAN2G(10, 2457),
	CHAN2G(11, 2462),
	CHAN2G(12, 2467),
	CHAN2G(13, 2472),
	CHAN2G(14, 2484),
};

static const struct ieee80211_channel mt76_channels_5ghz[] = {
	CHAN5G(36, 5180),
	CHAN5G(40, 5200),
	CHAN5G(44, 5220),
	CHAN5G(48, 5240),

	CHAN5G(52, 5260),
	CHAN5G(56, 5280),
	CHAN5G(60, 5300),
	CHAN5G(64, 5320),

	CHAN5G(100, 5500),
	CHAN5G(104, 5520),
	CHAN5G(108, 5540),
	CHAN5G(112, 5560),
	CHAN5G(116, 5580),
	CHAN5G(120, 5600),
	CHAN5G(124, 5620),
	CHAN5G(128, 5640),
	CHAN5G(132, 5660),
	CHAN5G(136, 5680),
	CHAN5G(140, 5700),

	CHAN5G(149, 5745),
	CHAN5G(153, 5765),
	CHAN5G(157, 5785),
	CHAN5G(161, 5805),
	CHAN5G(165, 5825),
};

#define CCK_RATE(_idx, _rate) {					\
	.bitrate = _rate,					\
	.flags = IEEE80211_RATE_SHORT_PREAMBLE,			\
	.hw_value = (MT_PHY_TYPE_CCK << 8) | _idx,		\
	.hw_value_short = (MT_PHY_TYPE_CCK << 8) | (8 + _idx),	\
}

#define OFDM_RATE(_idx, _rate) {				\
	.bitrate = _rate,					\
	.hw_value = (MT_PHY_TYPE_OFDM << 8) | _idx,		\
	.hw_value_short = (MT_PHY_TYPE_OFDM << 8) | _idx,	\
}

static struct ieee80211_rate mt76_rates[] = {
	CCK_RATE(0, 10),
	CCK_RATE(1, 20),
	CCK_RATE(2, 55),
	CCK_RATE(3, 110),
	OFDM_RATE(0, 60),
	OFDM_RATE(1, 90),
	OFDM_RATE(2, 120),
	OFDM_RATE(3, 180),
	OFDM_RATE(4, 240),
	OFDM_RATE(5, 360),
	OFDM_RATE(6, 480),
	OFDM_RATE(7, 540),
};

static int
mt76_init_sband(struct mt76_dev *dev, struct ieee80211_supported_band *sband,
		const struct ieee80211_channel *chan, int n_chan,
		struct ieee80211_rate *rates, int n_rates)
{
	struct ieee80211_sta_ht_cap *ht_cap;
	struct ieee80211_sta_vht_cap *vht_cap;
	void *chanlist;
	u16 mcs_map;
	int size;

	size = n_chan * sizeof(*chan);
	chanlist = devm_kmemdup(dev->dev, chan, size, GFP_KERNEL);
	if (!chanlist)
		return -ENOMEM;

	sband->channels = chanlist;
	sband->n_channels = n_chan;
	sband->bitrates = rates;
	sband->n_bitrates = n_rates;

	ht_cap = &sband->ht_cap;
	ht_cap->ht_supported = true;
	ht_cap->cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		      IEEE80211_HT_CAP_GRN_FLD |
		      IEEE80211_HT_CAP_SGI_20 |
		      IEEE80211_HT_CAP_SGI_40 |
		      IEEE80211_HT_CAP_LDPC_CODING |
		      IEEE80211_HT_CAP_TX_STBC |
		      (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);

	ht_cap->mcs.rx_mask[0] = 0xff;
	ht_cap->mcs.rx_mask[1] = 0xff;
	ht_cap->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
	ht_cap->ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	ht_cap->ampdu_density = IEEE80211_HT_MPDU_DENSITY_4;

	if (dev->cap.has_5ghz)
	{
		vht_cap = &sband->vht_cap;
		vht_cap->vht_supported = true;

		mcs_map = (IEEE80211_VHT_MCS_SUPPORT_0_9 << (0 * 2)) |
			  (IEEE80211_VHT_MCS_SUPPORT_0_9 << (1 * 2)) |
			  (IEEE80211_VHT_MCS_NOT_SUPPORTED << (2 * 2)) |
			  (IEEE80211_VHT_MCS_NOT_SUPPORTED << (3 * 2)) |
			  (IEEE80211_VHT_MCS_NOT_SUPPORTED << (4 * 2)) |
			  (IEEE80211_VHT_MCS_NOT_SUPPORTED << (5 * 2)) |
			  (IEEE80211_VHT_MCS_NOT_SUPPORTED << (6 * 2)) |
			  (IEEE80211_VHT_MCS_NOT_SUPPORTED << (7 * 2));

		vht_cap->vht_mcs.rx_mcs_map = cpu_to_le16(mcs_map);
		vht_cap->vht_mcs.tx_mcs_map = cpu_to_le16(mcs_map);
		vht_cap->cap = IEEE80211_VHT_CAP_RXLDPC |
			       IEEE80211_VHT_CAP_TXSTBC |
			       IEEE80211_VHT_CAP_RXSTBC_1 |
			       IEEE80211_VHT_CAP_SHORT_GI_80;
	}

	dev->chandef.chan = &sband->channels[0];

	return 0;
}

static int
mt76_init_sband_2g(struct mt76_dev *dev)
{
	if (!dev->cap.has_2ghz)
		return 0;

	dev->hw->wiphy->bands[IEEE80211_BAND_2GHZ] = &dev->sband_2g;
	return mt76_init_sband(dev, &dev->sband_2g,
			       mt76_channels_2ghz,
			       ARRAY_SIZE(mt76_channels_2ghz),
			       mt76_rates, ARRAY_SIZE(mt76_rates));
}

static int
mt76_init_sband_5g(struct mt76_dev *dev)
{
	if (!dev->cap.has_5ghz)
		return 0;

	dev->hw->wiphy->bands[IEEE80211_BAND_5GHZ] = &dev->sband_5g;
	return mt76_init_sband(dev, &dev->sband_5g,
			       mt76_channels_5ghz,
			       ARRAY_SIZE(mt76_channels_5ghz),
			       mt76_rates + 4, ARRAY_SIZE(mt76_rates) - 4);
}

static const struct ieee80211_iface_limit if_limits[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_ADHOC)
	}, {
		.max = 8,
		.types = BIT(NL80211_IFTYPE_STATION) |
#ifdef CONFIG_MAC80211_MESH
			 BIT(NL80211_IFTYPE_MESH_POINT) |
#endif
			 BIT(NL80211_IFTYPE_AP)
	 },
};

static const struct ieee80211_iface_combination if_comb[] = {
	{
		.limits = if_limits,
		.n_limits = ARRAY_SIZE(if_limits),
		.max_interfaces = 8,
		.num_different_channels = 1,
		.beacon_int_infra_match = true,
	}
};

int mt76_init_device(struct mt76_dev *dev)
{
	int ret;

	if (IS_76X2(dev))
		ret = mt76x2_init_device(dev);
	else
		ret = -EINVAL;

	if (ret)
		return ret;

	dev_printk(KERN_INFO, dev->dev, "ASIC revision: %08x\n", dev->rev);

	tasklet_init(&dev->pre_tbtt_tasklet, mt76_pre_tbtt_tasklet,
		     (unsigned long) dev);

	dev->slottime = 9;

	return 0;
}

int mt76_register_device(struct mt76_dev *dev)
{
	struct ieee80211_hw *hw = dev->hw;
	struct wiphy *wiphy = hw->wiphy;
	int i, ret;

	ret = mt76_init_hardware(dev);
	if (ret)
		return ret;

	SET_IEEE80211_DEV(hw, dev->dev);

	hw->queues = 4;
	hw->max_rates = 1;
	hw->max_report_rates = 7;
	hw->max_rate_tries = 1;
	hw->extra_tx_headroom = 2;

	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, PS_NULLFUNC_STACK);
	ieee80211_hw_set(hw, SUPPORTS_HT_CCK_RATES);
	ieee80211_hw_set(hw, HOST_BROADCAST_PS_BUFFERING);
	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	ieee80211_hw_set(hw, SUPPORTS_RC_TABLE);
	ieee80211_hw_set(hw, SUPPORT_FAST_XMIT);

	hw->sta_data_size = sizeof(struct mt76_sta);
	hw->vif_data_size = sizeof(struct mt76_vif);
	hw->txq_data_size = sizeof(struct mt76_txq);

	dev->macaddr[0] &= ~BIT(1);
	SET_IEEE80211_PERM_ADDR(hw, dev->macaddr);

	for (i = 0; i < ARRAY_SIZE(dev->macaddr_list); i++) {
		u8 *addr = dev->macaddr_list[i].addr;
		memcpy(addr, dev->macaddr, ETH_ALEN);

		if (!i)
			continue;

		addr[0] |= BIT(1);
		addr[0] ^= ((i - 1) << 2);
	}
	wiphy->addresses = dev->macaddr_list;
	wiphy->n_addresses = ARRAY_SIZE(dev->macaddr_list);

	wiphy->features |= NL80211_FEATURE_ACTIVE_MONITOR;

	wiphy->interface_modes =
		BIT(NL80211_IFTYPE_STATION) |
		BIT(NL80211_IFTYPE_AP) |
#ifdef CONFIG_MAC80211_MESH
		BIT(NL80211_IFTYPE_MESH_POINT) |
#endif
		BIT(NL80211_IFTYPE_ADHOC);

	wiphy->iface_combinations = if_comb;
	wiphy->n_iface_combinations = ARRAY_SIZE(if_comb);

	ret = mt76_init_sband_2g(dev);
	if (ret)
		goto fail;

	ret = mt76_init_sband_5g(dev);
	if (ret)
		goto fail;

	INIT_LIST_HEAD(&dev->txwi_cache);
	INIT_DELAYED_WORK(&dev->cal_work, mt76_phy_calibrate);
	INIT_DELAYED_WORK(&dev->mac_work, mt76_mac_work);

	ret = ieee80211_register_hw(hw);
	if (ret)
		goto fail;

	mt76_init_debugfs(dev);

	return 0;

fail:
	mt76_stop_hardware(dev);
	return ret;
}


