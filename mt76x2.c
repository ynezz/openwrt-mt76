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
#include "eeprom.h"
#include "mcu.h"

static void
mt76_write_mac_initvals(struct mt76_dev *dev)
{
	static const struct mt76_reg_pair vals[] = {
		/* Copied from MediaTek reference source */
		{ MT_PBF_SYS_CTRL,		0x00080c00 },
		{ MT_PBF_CFG,			0x1efebcff },
		{ MT_FCE_PSE_CTRL,		0x00000001 },
		{ MT_MAC_SYS_CTRL,		0x0000000c },
		{ MT_MAX_LEN_CFG,		0x003e3f00 },
		{ MT_AMPDU_MAX_LEN_20M1S,	0xaaa99887 },
		{ MT_AMPDU_MAX_LEN_20M2S,	0x000000aa },
		{ MT_XIFS_TIME_CFG,		0x33a40d0a },
		{ MT_BKOFF_SLOT_CFG,		0x00000209 },
		{ MT_TBTT_SYNC_CFG,		0x00422010 },
		{ MT_PWR_PIN_CFG,		0x00000000 },
		{ 0x1238,			0x001700c8 },
		{ MT_TX_SW_CFG0,		0x00101001 },
		{ MT_TX_SW_CFG1,		0x00010000 },
		{ MT_TX_SW_CFG2,		0x00000000 },
		{ MT_TXOP_CTRL_CFG,		0x0000583f },
		{ MT_TX_RTS_CFG,		0x00092b20 },
		{ MT_TX_TIMEOUT_CFG,		0x000a2290 },
		{ MT_TX_RETRY_CFG,		0x47f01f0f },
		{ MT_EXP_ACK_TIME,		0x002c00dc },
		{ MT_TX_PROT_CFG6,		0xe3f42004 },
		{ MT_TX_PROT_CFG7,		0xe3f42084 },
		{ MT_TX_PROT_CFG8,		0xe3f42104 },
		{ MT_PIFS_TX_CFG,		0x00060fff },
		{ MT_RX_FILTR_CFG,		0x00015f97 },
		{ MT_LEGACY_BASIC_RATE,		0x0000017f },
		{ MT_HT_BASIC_RATE,		0x00004003 },
		{ MT_PN_PAD_MODE,		0x00000002 },
		{ MT_TXOP_HLDR_ET,		0x00000002 },
		{ 0xa44,			0x00000000 },
		{ MT_HEADER_TRANS_CTRL_REG,	0x00000000 },
		{ MT_TSO_CTRL,			0x00000000 },
		{ MT_AUX_CLK_CFG,		0x00000000 },
		{ MT_DACCLK_EN_DLY_CFG,		0x00000000 },
		{ MT_TX_ALC_CFG_4,		0x00000000 },
		{ MT_TX_ALC_VGA3,		0x00000000 },
		{ MT_TX_PWR_CFG_0,		0x3a3a3a3a },
		{ MT_TX_PWR_CFG_1,		0x3a3a3a3a },
		{ MT_TX_PWR_CFG_2,		0x3a3a3a3a },
		{ MT_TX_PWR_CFG_3,		0x3a3a3a3a },
		{ MT_TX_PWR_CFG_4,		0x3a3a3a3a },
		{ MT_TX_PWR_CFG_7,		0x3a3a3a3a },
		{ MT_TX_PWR_CFG_8,		0x0000003a },
		{ MT_TX_PWR_CFG_9,		0x0000003a },
		{ MT_EFUSE_CTRL,		0x0000d000 },
		{ MT_PAUSE_ENABLE_CONTROL1,	0x0000000a },
		{ MT_FCE_WLAN_FLOW_CONTROL1,	0x60401c18 },
		{ MT_WPDMA_DELAY_INT_CFG,	0x94ff0000 },
		{ MT_TX_SW_CFG3,		0x00000004 },
		{ MT_HT_FBK_TO_LEGACY,		0x00001818 },
		{ MT_VHT_HT_FBK_CFG1,		0xedcba980 },
		{ MT_PROT_AUTO_TX_CFG,		0x00830083 },
		{ MT_HT_CTRL_CFG,		0x000001ff },
	};

	mt76_write_reg_pairs(dev, vals, ARRAY_SIZE(vals));
}

static void
mt76_fixup_xtal(struct mt76_dev *dev)
{
	u16 eep_val;
	s8 offset = 0;

	eep_val = mt76_eeprom_get(dev, MT_EE_XTAL_TRIM_2);

	offset = eep_val & 0x7f;
	if ((eep_val & 0xff) == 0xff)
		offset = 0;
	else if (eep_val & 0x80)
		offset = 0 - offset;

	eep_val >>= 8;
	if (eep_val == 0x00 || eep_val == 0xff) {
		eep_val = mt76_eeprom_get(dev, MT_EE_XTAL_TRIM_1);
		eep_val &= 0xff;

		if (eep_val == 0x00 || eep_val == 0xff)
			eep_val = 0x14;
	}

	eep_val &= 0x7f;
	mt76_rmw_field(dev, MT_XO_CTRL5, MT_XO_CTRL5_C2_VAL, eep_val + offset);
	mt76_set(dev, MT_XO_CTRL6, MT_XO_CTRL6_C2_CTRL);

	eep_val = mt76_eeprom_get(dev, MT_EE_NIC_CONF_2);
	switch (MT76_GET(MT_EE_NIC_CONF_2_XTAL_OPTION, eep_val)) {
	case 0:
		mt76_wr(dev, MT_XO_CTRL7, 0x5c1fee80);
		break;
	case 1:
		mt76_wr(dev, MT_XO_CTRL7, 0x5c1feed0);
		break;
	default:
		break;
	}
}

static void
mt76_init_beacon_offsets(struct mt76_dev *dev)
{
	u16 base = MT_BEACON_BASE;
	u32 regs[4] = {};
	int i;

	for (i = 0; i < 16; i++) {
		u16 addr = dev->beacon_offsets[i];

		regs[i / 4] |= ((addr - base) / 64) << (8 * (i % 4));
	}

	for (i = 0; i < 4; i++)
		mt76_wr(dev, MT_BCN_OFFSET(i), regs[i]);
}

static void
mt76_mac_pbf_init(struct mt76_dev *dev)
{
	u32 val;

	val = MT_PBF_SYS_CTRL_MCU_RESET |
	      MT_PBF_SYS_CTRL_DMA_RESET |
	      MT_PBF_SYS_CTRL_MAC_RESET |
	      MT_PBF_SYS_CTRL_PBF_RESET |
	      MT_PBF_SYS_CTRL_ASY_RESET;

	mt76_set(dev, MT_PBF_SYS_CTRL, val);
	mt76_clear(dev, MT_PBF_SYS_CTRL, val);

	mt76_wr(dev, MT_PBF_TX_MAX_PCNT, 0xefef3f1f);
	mt76_wr(dev, MT_PBF_RX_MAX_PCNT, 0xfebf);
}

static bool
mt76_wait_for_mac(struct mt76_dev *dev)
{
	int i;

	for (i = 0; i < 500; i++) {
		switch (mt76_rr(dev, MT_MAC_CSR0)) {
		case 0:
		case ~0:
			break;
		default:
			return true;
		}
		msleep(5);
	}

	return false;
}

static int
mt76x2_mac_reset(struct mt76_dev *dev, bool hard)
{
	static const u8 null_addr[ETH_ALEN] = {};
	u32 val;
	int i, k;

	if (!mt76_wait_for_mac(dev))
		return -ETIMEDOUT;

	val = mt76_rr(dev, MT_WPDMA_GLO_CFG);

	val &= ~(MT_WPDMA_GLO_CFG_TX_DMA_EN |
		 MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
		 MT_WPDMA_GLO_CFG_RX_DMA_EN |
		 MT_WPDMA_GLO_CFG_RX_DMA_BUSY |
		 MT_WPDMA_GLO_CFG_DMA_BURST_SIZE);
	val |= MT76_SET(MT_WPDMA_GLO_CFG_DMA_BURST_SIZE, 3);

	mt76_wr(dev, MT_WPDMA_GLO_CFG, val);

	mt76_mac_pbf_init(dev);
	mt76_write_mac_initvals(dev);
	mt76_fixup_xtal(dev);

	mt76_clear(dev, MT_MAC_SYS_CTRL,
		   MT_MAC_SYS_CTRL_RESET_CSR |
		   MT_MAC_SYS_CTRL_RESET_BBP);

	if (is_mt7612(dev))
		mt76_clear(dev, MT_COEXCFG0, MT_COEXCFG0_COEX_EN);

	mt76_set(dev, MT_EXT_CCA_CFG, 0x0000f000);
	mt76_clear(dev, MT_TX_ALC_CFG_4, BIT(31));

	mt76_wr(dev, MT_RF_BYPASS_0, 0x06000000);
	mt76_wr(dev, MT_RF_SETTING_0, 0x08800000);
	msleep(5);
	mt76_wr(dev, MT_RF_BYPASS_0, 0x00000000);

	mt76_wr(dev, MT_MCU_CLOCK_CTL, 0x1401);
	mt76_clear(dev, MT_FCE_L2_STUFF, MT_FCE_L2_STUFF_WR_MPDU_LEN_EN);

	mt76_wr(dev, MT_MAC_ADDR_DW0, get_unaligned_le32(dev->macaddr));
	mt76_wr(dev, MT_MAC_ADDR_DW1, get_unaligned_le16(dev->macaddr + 4));

	mt76_wr(dev, MT_MAC_BSSID_DW0, get_unaligned_le32(dev->macaddr));
	mt76_wr(dev, MT_MAC_BSSID_DW1, get_unaligned_le16(dev->macaddr + 4) |
		MT76_SET(MT_MAC_BSSID_DW1_MBSS_MODE, 3) | /* 8 beacons */
		MT_MAC_BSSID_DW1_MBSS_LOCAL_BIT);

	/* Fire a pre-TBTT interrupt 8 ms before TBTT */
	mt76_rmw_field(dev, MT_INT_TIMER_CFG, MT_INT_TIMER_CFG_PRE_TBTT,
		       8 << 4);
	mt76_wr(dev, MT_INT_TIMER_EN, 0);

	mt76_wr(dev, MT_BCN_BYPASS_MASK, 0xffff);
	if (!hard)
		return 0;

	for (i = 0; i < 256; i++)
		mt76_mac_wcid_setup(dev, i, 0, NULL);

	for (i = 0; i < 16; i++)
		for (k = 0; k < 4; k++)
			mt76_mac_shared_key_setup(dev, i, k, NULL);

	for (i = 0; i < 8; i++) {
		mt76_mac_set_bssid(dev, i, null_addr);
		mt76x2_mac_set_beacon(dev, i, NULL);
	}

	for (i = 0; i < 16; i++)
		mt76_rr(dev, MT_TX_STAT_FIFO);

	mt76_set(dev, MT_MAC_APC_BSSID_H(0), MT_MAC_APC_BSSID0_H_EN);
	mt76_init_beacon_offsets(dev);

	mt76_set_tx_ackto(dev);

	return 0;
}

static void
mt76_power_on_rf_patch(struct mt76_dev *dev)
{
	mt76_set(dev, 0x10130, BIT(0) | BIT(16));
	udelay(1);

	mt76_clear(dev, 0x1001c, 0xff);
	mt76_set(dev, 0x1001c, 0x30);

	mt76_wr(dev, 0x10014, 0x484f);
	udelay(1);

	mt76_set(dev, 0x10130, BIT(17));
	udelay(125);

	mt76_clear(dev, 0x10130, BIT(16));
	udelay(50);

	mt76_set(dev, 0x1014c, BIT(19) | BIT(20));
}

static void
mt76_power_on_rf(struct mt76_dev *dev, int unit)
{
	int shift = unit ? 8 : 0;

	/* Enable RF BG */
	mt76_set(dev, 0x10130, BIT(0) << shift);
	udelay(10);

	/* Enable RFDIG LDO/AFE/ABB/ADDA */
	mt76_set(dev, 0x10130, (BIT(1) | BIT(3) | BIT(4) | BIT(5)) << shift);
	udelay(10);

	/* Switch RFDIG power to internal LDO */
	mt76_clear(dev, 0x10130, BIT(2) << shift);
	udelay(10);

	mt76_power_on_rf_patch(dev);

	mt76_set(dev, 0x530, 0xf);
}

static void
mt76_power_on(struct mt76_dev *dev)
{
	u32 val;

	/* Turn on WL MTCMOS */
	mt76_set(dev, MT_WLAN_MTC_CTRL, MT_WLAN_MTC_CTRL_MTCMOS_PWR_UP);

	val = MT_WLAN_MTC_CTRL_STATE_UP |
	      MT_WLAN_MTC_CTRL_PWR_ACK |
	      MT_WLAN_MTC_CTRL_PWR_ACK_S;

	mt76_poll(dev, MT_WLAN_MTC_CTRL, val, val, 1000);

	mt76_clear(dev, MT_WLAN_MTC_CTRL, 0x7f << 16);
	udelay(10);

	mt76_clear(dev, MT_WLAN_MTC_CTRL, 0xf << 24);
	udelay(10);

	mt76_set(dev, MT_WLAN_MTC_CTRL, 0xf << 24);
	mt76_clear(dev, MT_WLAN_MTC_CTRL, 0xfff);

	/* Turn on AD/DA power down */
	mt76_clear(dev, 0x11204, BIT(3));

	/* WLAN function enable */
	mt76_set(dev, 0x10080, BIT(0));

	/* Release BBP software reset */
	mt76_clear(dev, 0x10064, BIT(18));

	mt76_power_on_rf(dev, 0);
	mt76_power_on_rf(dev, 1);
}

static void
mt76_set_wlan_state(struct mt76_dev *dev, bool enable)
{
	u32 val = mt76_rr(dev, MT_WLAN_FUN_CTRL);

	if (enable)
		val |= (MT_WLAN_FUN_CTRL_WLAN_EN |
			MT_WLAN_FUN_CTRL_WLAN_CLK_EN);
	else
		val &= ~(MT_WLAN_FUN_CTRL_WLAN_EN |
			 MT_WLAN_FUN_CTRL_WLAN_CLK_EN);

	mt76_wr(dev, MT_WLAN_FUN_CTRL, val);
	udelay(20);
}

static void
mt76_reset_wlan(struct mt76_dev *dev, bool enable)
{
	u32 val;

	val = mt76_rr(dev, MT_WLAN_FUN_CTRL);

	val &= ~MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL;

	if (val & MT_WLAN_FUN_CTRL_WLAN_EN) {
		val |= MT_WLAN_FUN_CTRL_WLAN_RESET_RF;
		mt76_wr(dev, MT_WLAN_FUN_CTRL, val);
		udelay(20);

		val &= ~MT_WLAN_FUN_CTRL_WLAN_RESET_RF;
	}

	mt76_wr(dev, MT_WLAN_FUN_CTRL, val);
	udelay(20);

	mt76_set_wlan_state(dev, enable);
}

static int
mt76x2_init_hardware(struct mt76_dev *dev)
{
	static const u16 beacon_offsets[16] = {
		/* 1024 byte per beacon */
		0xc000,
		0xc400,
		0xc800,
		0xcc00,
		0xd000,
		0xd400,
		0xd800,
		0xdc00,

		/* BSS idx 8-15 not used for beacons */
		0xc000,
		0xc000,
		0xc000,
		0xc000,
		0xc000,
		0xc000,
		0xc000,
		0xc000,
	};
	u32 val;
	int ret;

	dev->beacon_offsets = beacon_offsets;
	dev->chainmask = 0x202;

	val = mt76_rr(dev, MT_WPDMA_GLO_CFG);
	val &= MT_WPDMA_GLO_CFG_DMA_BURST_SIZE |
	       MT_WPDMA_GLO_CFG_BIG_ENDIAN |
	       MT_WPDMA_GLO_CFG_HDR_SEG_LEN;
	val |= MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE;
	mt76_wr(dev, MT_WPDMA_GLO_CFG, val);

	mt76_reset_wlan(dev, true);
	mt76_power_on(dev);

	ret = mt76_eeprom_init(dev);
	if (ret)
		return ret;

	ret = mt76x2_mac_reset(dev, true);
	if (ret)
		return ret;

	ret = mt76_dma_init(dev);
	if (ret)
		return ret;

	set_bit(MT76_STATE_INITIALIZED, &dev->state);
	ret = mt76_mac_start(dev);
	if (ret)
		return ret;

	ret = mt76_mcu_init(dev);
	if (ret)
		return ret;

	mt76_mac_stop(dev, false);
	dev->rxfilter = mt76_rr(dev, MT_RX_FILTR_CFG);

	/* Fix up ASPM configuration */

	/* RG_SSUSB_G1_CDR_BIR_LTR = 0x9 */
	mt76_rmw_field(dev, 0x15a10, 0x1f << 16, 0x9);

	/* RG_SSUSB_G1_CDR_BIC_LTR = 0xf */
	mt76_rmw_field(dev, 0x15a0c, 0xf << 28, 0xf);

	/* RG_SSUSB_CDR_BR_PE1D = 0x3 */
	mt76_rmw_field(dev, 0x15c58, 0x3 << 6, 0x3);

	return 0;
}

static void
mt76x2_mac_work(struct work_struct *work)
{
	struct mt76_dev *dev = container_of(work, struct mt76_dev,
					    mac_work.work);
	int i, idx;

	for (i = 0, idx = 0; i < 16; i++) {
		u32 val = mt76_rr(dev, MT_TX_AGG_CNT(i));
		dev->aggr_stats[idx++] += val & 0xffff;
		dev->aggr_stats[idx++] += val >> 16;
	}

	ieee80211_queue_delayed_work(dev->hw, &dev->mac_work,
				     MT_CALIBRATE_INTERVAL);
}

static const struct mt76_chip_ops chip_ops = {
	.init_hardware = mt76x2_init_hardware,
};

int mt76x2_init_device(struct mt76_dev *dev)
{
	void *status_fifo;
	int fifo_size;

	dev->ops = &chip_ops;
	dev->rev = mt76_rr(dev, MT_ASIC_VERSION);

	fifo_size = roundup_pow_of_two(32 * sizeof(struct mt76_tx_status));
	status_fifo = devm_kzalloc(dev->dev, fifo_size, GFP_KERNEL);
	if (!status_fifo)
		return -ENOMEM;

	kfifo_init(&dev->txstatus_fifo, status_fifo, fifo_size);

	INIT_DELAYED_WORK(&dev->mac_work, mt76x2_mac_work);
	tasklet_init(&dev->pre_tbtt_tasklet, mt76x2_pre_tbtt_tasklet,
		     (unsigned long) dev);

	return 0;
}
