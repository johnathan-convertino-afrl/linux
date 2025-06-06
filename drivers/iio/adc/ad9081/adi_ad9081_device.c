// SPDX-License-Identifier: GPL-2.0
/*!
 * @brief     device level API implementation
 *
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup __AD9081_DEVICE_API__
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_ad9081_config.h"
#include "adi_ad9081_hal.h"

/*============= D A T A ====================*/
static uint8_t ad9081_api_revision[3] = { 1, 0, 0 };

/*============= C O D E ====================*/
int32_t adi_ad9081_device_boot_pre_clock(adi_ad9081_device_t *device)
{
	int32_t err;
	uint8_t i, core_status, laminate_id, die_id;
	adi_cms_chip_id_t chip_id;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	for (i = 0; i < 100; i++) {
		err = adi_ad9081_hal_reg_get(device, 0x3742,
					     &core_status); /* @msg2 */
		AD9081_ERROR_RETURN(err);
		if (core_status <
		    0x71) { /* (0x71) boot_powerup_enable_clocks_done_msg */
			err = adi_ad9081_hal_delay_us(device, 1000);
			AD9081_ERROR_RETURN(err);
		} else {
			break;
		}
	}
	if (core_status < 0x71) {
		/* boot has some problem */
		AD9081_LOG_ERR(
			"Boot did not reach expected spot in boot_pre_clock()");

		/* log msg regs when error happens */
		err = adi_ad9081_hal_reg_get(device, 0x3740,
					     &core_status); /* @msg0 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_reg_get(device, 0x3741,
					     &core_status); /* @msg1 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_reg_get(device, 0x3742,
					     &core_status); /* @msg2 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_reg_get(device, 0x3743,
					     &core_status); /* @msg3 */
		AD9081_ERROR_RETURN(err);
	}

	/* log chip id */
	err = adi_ad9081_device_chip_id_get(device, &chip_id);
	AD9081_ERROR_RETURN(err);
	device->dev_info.dev_rev = chip_id.dev_revision;
	err = adi_ad9081_hal_log_write(device, ADI_CMS_LOG_MSG,
				       "device is ad%x r%d", chip_id.prod_id,
				       chip_id.dev_revision);
	AD9081_ERROR_RETURN(err);
	if (device->dev_info.dev_rev == 0) { /* not support r0 */
		AD9081_LOG_ERR("Current device revision is not supported");
		return API_CMS_ERROR_ERROR;
	}
	if (device->dev_info.dev_rev == 1) { /* test 32bit r/w for r1 */
		err = adi_ad9081_device_reg32_access_check(device);
		AD9081_ERROR_RETURN(err);
	}

	/* log laminate and die id */
	err = adi_ad9081_device_laminate_id_get(device, &laminate_id);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_device_die_id_get(device, &die_id);
	AD9081_ERROR_RETURN(err);

	/* ad9081api-621 */
	if (device->dev_info.dev_rev == 2 ||
	    device->dev_info.dev_rev == 3) { /* r1r/r2 */
		err = adi_ad9081_hal_bf_set(
			device, 0x21b2, 0x0104,
			0x1); /* ckt_reset_bypass_en_shadow */
		AD9081_ERROR_RETURN(err);
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_boot_post_clock(adi_ad9081_device_t *device)
{
	int32_t err;
	uint8_t i, boot_done, core_status, clk_switch_done;
	uint32_t rev = 0;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* bypass the boot loader's power up sequence */
	if (device->dev_info.dev_rev == 1) { /* r1 */
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0100,
			0); /* pup1_startup_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0101,
			0); /* pup2_clockenable_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001610, 0x0113,
			1); /* pll_lock_check_disable@alt_boot_reg4 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0102,
			0); /* pup4a_dacenable_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0103,
			0); /* pup4b_adcenable_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0104,
			0); /* pup5_dacdll_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0105,
			1); /* pup6_daccal_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x01001618, 0x0106,
			0); /* pup6_daccalwait_bypass_en@alt_boot_reg6 */
		AD9081_ERROR_RETURN(err);
	}
	if (device->dev_info.dev_rev == 2 ||
	    device->dev_info.dev_rev == 3) { /* r1r/r2 */
		/* @msg3: bit0: pll_lock_check_disable,     bit1: pup1_startup_bypass_en
         *        bit2: pup2_clockenable_bypass_en, bit3: pup4a_dacenable_bypass_en
         *        bit4: pup4b_adcenable_bypass_en,  bit5: pup5_dacdll_bypass_en
         *        bit6: pup6_daccal_bypass_en,      bit7: pup6_daccalwait_bypass_en
         */
		err = adi_ad9081_hal_reg_set(device, 0x3743, 0x01);
		AD9081_ERROR_RETURN(err);
	}

	/* trigger edge interrupt */
	err = adi_ad9081_hal_bf_set(device, REG_UP_CTRL_ADDR,
				    BF_UP_SPI_EDGE_INTERRUPT_INFO, 0x1);
	AD9081_ERROR_RETURN(err);
	if (err = adi_ad9081_hal_bf_wait_to_clear(
		    device, REG_UP_CTRL_ADDR, BF_UP_SPI_EDGE_INTERRUPT_INFO),
	    err != API_CMS_ERROR_OK)
		AD9081_LOG_ERR("up_spi_edge_interrupt bit never cleared");

	/* check boot_done */
	for (i = 0; i < 100; i++) {
		err = adi_ad9081_hal_bf_get(device, 0x3740, 0x0100, &boot_done,
					    1); /* boot_done@msg0 */
		AD9081_ERROR_RETURN(err);
		if (boot_done == 0) {
			err = adi_ad9081_hal_delay_us(device, 1000);
			AD9081_ERROR_RETURN(err);
		} else {
			break;
		}
	}

	/* check core status */
	err = adi_ad9081_hal_reg_get(device, 0x3742, &core_status); /* @msg2 */
	AD9081_ERROR_RETURN(err);
	if (core_status < 0xF0)
		AD9081_LOG_ERR(
			"Boot did not reach spot where it waits for application code");

	/* verify clock switch is done */
	err = adi_ad9081_hal_bf_get(device, 0x3740, 0x0103, &clk_switch_done,
				    1); /* @msg0 */
	AD9081_ERROR_RETURN(err);
	if (clk_switch_done == 0x0)
		AD9081_LOG_ERR("Clock switch not done");

	/* log firmware revision */
	err = adi_ad9081_device_firmware_revision_get(device, &rev);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_device_firmware_patch_revision_get(device, &rev);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_clk_pll_lock_status_get(adi_ad9081_device_t *device,
						  uint8_t *status)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(status);

	/* check pll lock status, bit0 : lock_slow, bit1: lock_fast */
	err = adi_ad9081_hal_reg_get(device, REG_CLK_PLL_STATUS_ADDR, status);
	AD9081_ERROR_RETURN(err);
	*status = *status & 0x03;

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_clk_pll_enable_set(adi_ad9081_device_t *device,
					     uint8_t pll_en)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	err = adi_ad9081_hal_bf_set(device, REG_PLL_BYPASS_ADDR,
				    BF_PLL_BYPASS_INFO, !pll_en);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_set(device, REG_POWERDOWN_REG_0_ADDR,
				     pll_en > 0 ? 0x00 : 0x9f);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_set(device, REG_POWERDOWN_REG_1_ADDR,
				     pll_en > 0 ? 0x00 : 0x1f);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_clk_pll_div_set(adi_ad9081_device_t *device,
					  uint8_t ref_div, uint8_t m_div,
					  uint8_t pll_div, uint8_t fb_div)
{
	int32_t err;
	uint8_t i, pll_lock;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* change defaults */
	err = adi_ad9081_hal_bf_set(device, REG_SLOWV_COMP_HIGHL_REG_0_ADDR,
				    BF_D_SLOWV_COMP_HIGHL_INFO, 0x57F);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_FASTV_COMP_HIGHL_REG_0_ADDR,
				    BF_D_FASTV_COMP_HIGHL_INFO, 0x2FF);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_BIAS_REG_1_ADDR,
				    BF_D_BIAS_POLY_TRIM_INFO, 0x1F);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_BIAS_REG_0_ADDR,
				    BF_D_BIAS_FIXED_TRIM_INFO, 0x20);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_CHARGEPUMP_REG_0_ADDR,
				    BF_D_CP_CURRENT_INFO, 0x13);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_VCM_CONTROL_REG_ADDR,
				    BF_D_VCM_F_CONTROL_INFO, 0x0C);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_VCM_CONTROL_REG_ADDR,
				    BF_D_VCM_C_CONTROL_INFO, 0x00);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_INPUT_MISC_REG_ADDR,
				    BF_D_PFD_DELAY_INFO, 0x01);
	AD9081_ERROR_RETURN(err);

	/* reset dividers */
	err = adi_ad9081_hal_reg_set(device, REG_RESET_REG_ADDR, 0x1f);
	AD9081_ERROR_RETURN(err);

	/* release dividers */
	err = adi_ad9081_hal_reg_set(device, REG_RESET_REG_ADDR, 0x00);
	AD9081_ERROR_RETURN(err);

	/* set dividers */
	err = adi_ad9081_hal_bf_set(device, REG_INPUT_MISC_REG_ADDR,
				    BF_D_REFIN_DIV_INFO, ref_div - 1);
	AD9081_ERROR_RETURN(err);
	if (ref_div == 3) {
		err = adi_ad9081_hal_bf_set(device, REG_RESET_REG_ADDR,
					    BF_D_RESET_REF_DIV_INFO, 0x1);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(device, REG_RESET_REG_ADDR,
					    BF_D_RESET_REF_DIV_INFO, 0x0);
		AD9081_ERROR_RETURN(err);
	}
	err = adi_ad9081_hal_bf_set(device, REG_PLL_CLK_DIV_ADDR,
				    BF_PLL_DIVIDEFACTOR_INFO, pll_div - 1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_DIVIDER_REG_ADDR,
				    BF_D_DIVIDE_CONTROL_INFO, m_div);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_VCO_CAL_LOCK_REG_ADDR,
				    BF_D_CONTROL_HS_FB_DIV_INFO, fb_div);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_VCO_CAL_MOMCAP_REG_1_ADDR,
				    BF_D_IMPALA_TEMP_INFO, 0x2);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_VCO_CAL_CONTROL_REG_0_ADDR,
				    BF_D_IMPALA_CAL_CONTROL_INFO, 0x3D60);
	AD9081_ERROR_RETURN(err);

	/* reset cal, try to lock pll */
	err = adi_ad9081_hal_bf_set(device, REG_RESET_REG_ADDR,
				    BF_D_CAL_RESET_INFO, 0x1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_RESET_REG_ADDR,
				    BF_D_CAL_RESET_INFO, 0x0);
	AD9081_ERROR_RETURN(err);

	/* check pll lock status */
	for (i = 0; i < 50; i++) {
		err = adi_ad9081_hal_delay_us(device, 20000);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_device_clk_pll_lock_status_get(device,
								&pll_lock);
		AD9081_ERROR_RETURN(err);
		if (pll_lock == 0x3)
			break;
	}
	if (pll_lock != 0x3) {
		AD9081_ERROR_REPORT(API_CMS_ERROR_PLL_NOT_LOCKED, pll_lock,
				    "PLL not locked");
		return API_CMS_ERROR_PLL_NOT_LOCKED;
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_clk_pll_startup(adi_ad9081_device_t *device,
					  uint64_t dac_clk_hz,
					  uint64_t ref_clk_hz)
{
	int32_t err;
	uint64_t vco_clk_hz, pfd_clk_hz;
	uint8_t ref_div = 1, n_div = 1, m_div = 1, pll_div = 1, fb_div = 1;
	uint8_t i, total_feedback;
	uint8_t n_div_vals[] = { 5, 7, 8, 11 };
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* calculate pll div */
	if ((dac_clk_hz >= 5800000000ULL) && (dac_clk_hz < 12600000000ULL)) {
		pll_div = 1;
	} else if ((dac_clk_hz >= 3000000000ULL) &&
		   (dac_clk_hz < 5800000000ULL)) {
		pll_div = 2;
	} else if ((dac_clk_hz > 1250000000ULL) &&
		   (dac_clk_hz < 3000000000ULL)) {
		pll_div = 4;
	} else {
		AD9081_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, dac_clk_hz,
				    "Invalid DAC clock.");
	}
	vco_clk_hz = dac_clk_hz * pll_div;

	/* calculate ref div */
	if (ref_clk_hz < 1000000000ULL) {
		ref_div = 1;
	} else if ((ref_clk_hz >= 1000000000ULL) &&
		   (ref_clk_hz < 2000000000ULL)) {
		ref_div = 2;
	} else if ((ref_clk_hz >= 2000000000ULL) &&
		   (ref_clk_hz < 3000000000ULL)) {
		ref_div = 3;
	} else if ((ref_clk_hz >= 3000000000ULL) &&
		   (ref_clk_hz < 4000000000ULL)) {
		ref_div = 4;
	} else {
		AD9081_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, ref_clk_hz,
				    "Ref clock is too high.");
	}

	/* calculate m/n div */
#ifdef __KERNEL__
	pfd_clk_hz = div_u64(ref_clk_hz, ref_div);
	total_feedback = (uint8_t)div64_u64(vco_clk_hz, pfd_clk_hz);
#else
	pfd_clk_hz = ref_clk_hz / ref_div;
	total_feedback = (uint8_t)(vco_clk_hz / pfd_clk_hz);
#endif
	for (i = 0; i < 4; i++) {
		if ((total_feedback % n_div_vals[i]) == 0) {
			n_div = n_div_vals[i];
			m_div = total_feedback / n_div;
			if ((m_div == 1) && (ref_clk_hz > 80000000ULL)) {
				ref_div = 2;
				m_div = 2;
			}
			break;
		}
	}
	if (i >= 4) {
		AD9081_LOG_ERR("Cannot find any settings to lock PLL.");
		return API_CMS_ERROR_INVALID_PARAM;
	}

	/* calculate fb div */
	if (n_div == 5)
		fb_div = 0;
	else if (n_div == 7)
		fb_div = 1;
	else if (n_div == 8)
		fb_div = 2;
	else if (n_div == 11)
		fb_div = 3;

	/* enable pll */
	err = adi_ad9081_device_clk_pll_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* set internal and external dividers */
	err = adi_ad9081_device_clk_pll_div_set(device, ref_div, m_div, pll_div,
						fb_div);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_clk_up_div_set(adi_ad9081_device_t *device,
					 uint64_t dac_clk_hz)
{
	int32_t err;
	uint8_t in_div_spi, cdiv, sdiv, pdiv, mdiv;
	uint64_t in_clk, cclk, sclk;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* setup uP clock div */
	in_div_spi = (dac_clk_hz > 6000000000ULL) ? 1 : 0;
	in_clk = dac_clk_hz >> ((dac_clk_hz > 6000000000ULL) ? 3 : 2);
#ifdef __KERNEL__
	cdiv = (uint8_t)div64_u64((in_clk + 500000000ULL - 1),
				  500000000ULL); /* ceil(in_clk / 500e6) */
#else
	cdiv = (uint8_t)((in_clk + 500000000ULL - 1) /
			 500000000ULL); /* ceil(in_clk / 500e6) */
#endif
	cdiv = cdiv > 31 ? 31 : cdiv;
	cdiv = cdiv < 2 ? 2 : cdiv;
#ifdef __KERNEL__
	cclk = div_u64(in_clk, cdiv);
	sdiv = (uint8_t)div64_u64((cclk + 250000000ULL - 1),
				  250000000ULL); /* ceil(cclk / 250e6) */
#else
	cclk = (in_clk / cdiv);
	sdiv = (uint8_t)((cclk + 250000000ULL - 1) /
			 250000000ULL); /* ceil(cclk / 250e6) */
#endif
	sdiv = sdiv > 4 ? 4 : sdiv;
	sdiv = sdiv < 2 ? 2 : sdiv;
#ifdef __KERNEL__
	sclk = div_u64(cclk, sdiv);
	pdiv = (uint8_t)div64_u64((sclk + 125000000ULL - 1),
				  125000000ULL); /* ceil(sclk / 125e6) */
#else
	sclk = (cclk / sdiv);
	pdiv = (uint8_t)((sclk + 125000000ULL - 1) /
			 125000000ULL); /* ceil(sclk / 125e6) */
#endif
	pdiv = pdiv > 4 ? 4 : pdiv;
	pdiv = pdiv < 2 ? 2 : pdiv;
#ifdef __KERNEL__
	mdiv = (uint8_t)div64_u64(((in_clk >> 1) + 50000000ULL - 1),
				  50000000ULL); /* ceil((in_clk / 2) / 50e6) */
#else
	mdiv = (uint8_t)(((in_clk >> 1) + 50000000ULL - 1) /
			 50000000ULL); /* ceil((in_clk / 2) / 50e6) */
#endif
	mdiv = mdiv < 1 ? 1 : mdiv;
	err = adi_ad9081_hal_bf_set(device, REG_SPI_ENABLE_DAC_ADDR,
				    BF_SPI_EN_D2ACENTER_INFO, 0x1);
	AD9081_ERROR_RETURN(err);
	if (device->dev_info.dev_rev == 1) { /* r1 */
		err = adi_ad9081_hal_reg_set(
			device, 0x01001614,
			(in_div_spi << 31) | (cdiv << 24) | (mdiv << 16) |
				(sdiv << 12) | (pdiv << 8) |
				cdiv); /* @alt_boot_reg5 */
		AD9081_ERROR_RETURN(err);
	}
	if (device->dev_info.dev_rev == 2 ||
	    device->dev_info.dev_rev == 3) { /* r1r/r2 */
		err = adi_ad9081_hal_bf_set(
			device, 0x21b3, 0x0500,
			cdiv); /* cdiv_firm_user@up_clk_div_1 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x21b3, 0x0107,
			in_div_spi); /* uc_clk_sel_firm_user@up_clk_div_1 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x21b4, 0x0300,
			pdiv); /* pdiv_firm_user@up_clk_div_2 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x21b4, 0x0304,
			sdiv); /* sdiv_firm_user@up_clk_div_2 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x21b5, 0x0800,
			mdiv); /* mdiv_firm_user@up_clk_div_3 */
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_bf_set(
			device, 0x21b3, 0x0106,
			1); /* crp_change_en_user@up_clk_div_1 */
		AD9081_ERROR_RETURN(err);
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_clk_config_set(adi_ad9081_device_t *device,
					 uint64_t dac_clk_hz,
					 uint64_t adc_clk_hz,
					 uint64_t ref_clk_hz, uint8_t pll_en)
{
	int32_t err;
	uint8_t adc_clk_div;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	if (pll_en > 0) {
		AD9081_INVALID_PARAM_WARN(ref_clk_hz >
					  AD9081_REF_CLK_FREQ_HZ_MAX);
		AD9081_INVALID_PARAM_WARN(ref_clk_hz <
					  AD9081_REF_CLK_FREQ_HZ_MIN);
	}
	AD9081_INVALID_PARAM_WARN(adc_clk_hz > AD9081_ADC_CLK_FREQ_HZ_MAX);
	AD9081_INVALID_PARAM_WARN(adc_clk_hz < AD9081_ADC_CLK_FREQ_HZ_MIN);
	AD9081_INVALID_PARAM_WARN(dac_clk_hz > AD9081_DAC_CLK_FREQ_HZ_MAX);
	AD9081_INVALID_PARAM_WARN(dac_clk_hz < AD9081_DAC_CLK_FREQ_HZ_MIN);
	AD9081_INVALID_PARAM_RETURN(adc_clk_hz == 0);

	/* save clock settings */
	device->dev_info.dev_freq_hz = ref_clk_hz;
	device->dev_info.dac_freq_hz = dac_clk_hz;
	device->dev_info.adc_freq_hz = adc_clk_hz;

	/* check internal status */
	err = adi_ad9081_device_boot_pre_clock(device);
	AD9081_ERROR_RETURN(err);

	/* enable dac digital logic */
	err = adi_ad9081_dac_digital_logic_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* enable dac spi regs access */
	err = adi_ad9081_dac_spi_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* power up analog clock receiver */
	err = adi_ad9081_device_aclk_receiver_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* bypass pll */
	err = adi_ad9081_device_clk_pll_enable_set(device, 0);
	AD9081_ERROR_RETURN(err);

	/* startup pll */
	if (pll_en == 0) {
		dac_clk_hz = ref_clk_hz;
	} else {
		err = adi_ad9081_device_clk_pll_startup(device, dac_clk_hz,
							ref_clk_hz);
		AD9081_ERROR_RETURN(err);
	}

	/* set adc clk div */
#ifdef __KERNEL__
	adc_clk_div = (uint8_t)div64_u64(dac_clk_hz, adc_clk_hz);
#else
	adc_clk_div = (uint8_t)(dac_clk_hz / adc_clk_hz);
#endif
	if ((adc_clk_hz * adc_clk_div) != dac_clk_hz) {
		AD9081_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, adc_clk_hz,
				    "Cannot generate required adc clock.");
		return API_CMS_ERROR_INVALID_PARAM;
	}
	if ((adc_clk_div == 0) ||
	    (adc_clk_div >
	     4)) { /* Fadc needs to be 1/2/3/4 factor of device clock */
		AD9081_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, adc_clk_hz,
				    "Cannot generate required adc clock.");
		return API_CMS_ERROR_INVALID_PARAM;
	}
	err = adi_ad9081_adc_clk_div_set(device, adc_clk_div);
	AD9081_ERROR_RETURN(err);

	/* set up clk div */
	err = adi_ad9081_device_clk_up_div_set(device, dac_clk_hz);
	AD9081_ERROR_RETURN(err);

	/* check internal status */
	err = adi_ad9081_device_boot_post_clock(device);
	AD9081_ERROR_RETURN(err);

	/* enable dac spi regs access again, as firmware may change paging value after being triggerred in _post_clock() */
	err = adi_ad9081_dac_spi_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* enable adc clk */
	err = adi_ad9081_adc_clk_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* enable adc clock out */
	adi_ad9081_adc_clk_out_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_aclk_receiver_enable_set(adi_ad9081_device_t *device,
						   uint8_t enable)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_INVALID_PARAM_RETURN(enable > 1);

	err = adi_ad9081_hal_bf_set(device, REG_ACLK_CTRL_ADDR,
				    BF_ACLK_POWERDOWN_INFO, !enable);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_spi_config(adi_ad9081_device_t *device)
{
	uint8_t reg_val = 0x00;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	reg_val |= ((device->hal_info.sdo == SPI_SDIO) ? 0x00 : 0x18);
	reg_val |= ((device->hal_info.msb == SPI_MSB_FIRST) ? 0x00 : 0x42);
	reg_val |= ((device->hal_info.addr_inc == SPI_ADDR_DEC_AUTO) ? 0x00 :
								       0x24);

	return adi_ad9081_hal_reg_set(device, REG_SPI_INTFCONFA_ADDR, reg_val);
}

int32_t adi_ad9081_device_hw_open(adi_ad9081_device_t *device)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	if (err = adi_ad9081_hal_hw_open(device), err != API_CMS_ERROR_OK) {
		return err;
	}
	AD9081_LOG_FUNC();

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_hw_close(adi_ad9081_device_t *device)
{
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	return adi_ad9081_hal_hw_close(device);
}

int32_t adi_ad9081_device_power_status_check(adi_ad9081_device_t *device)
{
	int32_t err;
	uint8_t reg8, power_on;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* check power status */
	power_on = 1;
	err = adi_ad9081_hal_bf_get(device, REG_DAC_SUPPLY_MONITOR_ADDR,
				    BF_AVDD_DAC23_MON2_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_DAC_SUPPLY_MONITOR_ADDR,
				    BF_AVDD_DAC01_MON2_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_DAC_SUPPLY_MONITOR_ADDR,
				    BF_DVDD_DAC23_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_DAC_SUPPLY_MONITOR_ADDR,
				    BF_DVDD_DAC01_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_DAC_SUPPLY_MONITOR_ADDR,
				    BF_DAVDD_DAC23_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_DAC_SUPPLY_MONITOR_ADDR,
				    BF_DAVDD_DAC01_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_CLOCK_SUPPLY_MONITOR_ADDR,
				    BF_DACPLLVDD_MON2_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_CLOCK_SUPPLY_MONITOR_ADDR,
				    BF_LS_CLOCK_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_CLOCK_SUPPLY_MONITOR_ADDR,
				    BF_HS_CLOCK_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_CLOCK_SUPPLY_MONITOR_ADDR,
				    BF_REF_UP_CLOCK_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC0_SUPPLY_MONITOR_ADDR,
				    BF_ADC0_CLK_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC0_SUPPLY_MONITOR_ADDR,
				    BF_ADC0_CORE_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC0_SUPPLY_MONITOR_ADDR,
				    BF_ADC0_BUF_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC0_SUPPLY_MONITOR_ADDR,
				    BF_ADC0_REFADC_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC0_SUPPLY_MONITOR_ADDR,
				    BF_ADC0_REF_MON2_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC1_SUPPLY_MONITOR_ADDR,
				    BF_ADC1_CLK_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC1_SUPPLY_MONITOR_ADDR,
				    BF_ADC1_CORE_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC1_SUPPLY_MONITOR_ADDR,
				    BF_ADC1_BUF_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC1_SUPPLY_MONITOR_ADDR,
				    BF_ADC1_REFADC_MON1_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	err = adi_ad9081_hal_bf_get(device, REG_ADC1_SUPPLY_MONITOR_ADDR,
				    BF_ADC1_REF_MON2_INFO, &reg8, 1);
	AD9081_ERROR_RETURN(err);
	power_on &= reg8;
	if (power_on == 0) {
		err = adi_ad9081_hal_log_write(
			device, ADI_CMS_LOG_ERR,
			"some power supplies are not on.");
		AD9081_ERROR_RETURN(err);
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_reg8_access_check(adi_ad9081_device_t *device)
{
	int32_t err;
	uint8_t reg8, data8;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* 8bit reg read/write test */
	err = adi_ad9081_hal_reg_get(device, REG_PAGEINDX_DAC_CHAN_ADDR, &reg8);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_set(device, REG_PAGEINDX_DAC_CHAN_ADDR, 0x5a);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_get(device, REG_PAGEINDX_DAC_CHAN_ADDR,
				     &data8);
	AD9081_ERROR_RETURN(err);
	if (data8 != 0x5a) {
		err = adi_ad9081_hal_log_write(
			device, ADI_CMS_LOG_ERR,
			"8bit r/w test failed. Write %.2x but readback is %.2x.",
			0x5a, data8);
		AD9081_ERROR_RETURN(err);
		return API_CMS_ERROR_TEST_FAILED;
	}
	err = adi_ad9081_hal_reg_set(device, REG_PAGEINDX_DAC_CHAN_ADDR, 0xa5);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_get(device, REG_PAGEINDX_DAC_CHAN_ADDR,
				     &data8);
	AD9081_ERROR_RETURN(err);
	if (data8 != 0xa5) {
		err = adi_ad9081_hal_log_write(
			device, ADI_CMS_LOG_ERR,
			"8bit r/w test failed. Write %.2x but readback is %.2x.",
			0xa5, data8);
		AD9081_ERROR_RETURN(err);
		return API_CMS_ERROR_TEST_FAILED;
	}
	err = adi_ad9081_hal_reg_set(device, REG_PAGEINDX_DAC_CHAN_ADDR, reg8);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_reg32_access_check(adi_ad9081_device_t *device)
{
	int32_t err;
	uint32_t reg32, data32;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	err = adi_ad9081_hal_reg_get(device, 0x01001300, (uint8_t *)&reg32);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_set(device, 0x01001300, 0x55aa55aa);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_get(device, 0x01001300, (uint8_t *)&data32);
	AD9081_ERROR_RETURN(err);
	if (data32 != 0x55aa55aa) {
		err = adi_ad9081_hal_log_write(
			device, ADI_CMS_LOG_ERR,
			"32bit r/w test failed. Write %.8x but readback is %.8x.",
			0x55aa55aa, data32);
		AD9081_ERROR_RETURN(err);
		return API_CMS_ERROR_TEST_FAILED;
	}
	err = adi_ad9081_hal_reg_set(device, 0x01001300, 0xaa55aa55);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_get(device, 0x01001300, (uint8_t *)&data32);
	AD9081_ERROR_RETURN(err);
	if (data32 != 0xaa55aa55) {
		err = adi_ad9081_hal_log_write(
			device, ADI_CMS_LOG_ERR,
			"32bit r/w test failed. Write %.8x but readback is %.8x.",
			0xaa55aa55, data32);
		AD9081_ERROR_RETURN(err);
		return API_CMS_ERROR_TEST_FAILED;
	}
	err = adi_ad9081_hal_reg_set(device, 0x01001300, 0x11223344);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_reg_get(device, 0x01001300, (uint8_t *)&data32);
	AD9081_ERROR_RETURN(err);
	if (data32 != 0x11223344) {
		err = adi_ad9081_hal_log_write(
			device, ADI_CMS_LOG_ERR,
			"32bit r/w test failed. Write %.8x but readback is %.8x.",
			0x11223344, data32);
		AD9081_ERROR_RETURN(err);
		return API_CMS_ERROR_TEST_FAILED;
	}
	err = adi_ad9081_hal_reg_set(device, 0x01001300, reg32);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_init(adi_ad9081_device_t *device)
{
	int32_t err;
	uint32_t endian_test_val = 0x11223344;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* log API information */
	err = adi_ad9081_hal_log_write(
		device, ADI_CMS_LOG_MSG, "api v%d.%d.%d commit %s for ad%x ",
		ad9081_api_revision[0], ad9081_api_revision[1],
		ad9081_api_revision[2], "74e60ae", AD9081_ID);
	AD9081_ERROR_RETURN(err);

	/* get host cpu endian mode */
	if (*(uint8_t *)&endian_test_val == 0x44)
		AD9081_LOG_MSG("host is using little endian mode.");
	else
		AD9081_LOG_MSG("host is using big endian mode.");

	/* configure spi pol/mode */
	err = adi_ad9081_device_spi_config(device);
	AD9081_ERROR_RETURN(err);

	/* test 8bit reg r/w */
	err = adi_ad9081_device_reg8_access_check(device);
	AD9081_ERROR_RETURN(err);

	/* check power status */
	err = adi_ad9081_device_power_status_check(device);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_spi_register_set(adi_ad9081_device_t *device,
					   uint16_t addr, uint8_t data)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	err = adi_ad9081_hal_reg_set(device, addr, data);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_spi_register_get(adi_ad9081_device_t *device,
					   uint16_t addr, uint8_t *data)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(data);

	err = adi_ad9081_hal_reg_get(device, addr, data);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_cbusjrx_register_get(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t *data,
					       uint8_t lane)
{
	return adi_ad9081_hal_cbusjrx_reg_get(device, addr, data, lane);
}

int32_t adi_ad9081_device_cbusjrx_register_set(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t data,
					       uint8_t lane)
{
	return adi_ad9081_hal_cbusjrx_reg_set(device, addr, data, lane);
}

int32_t adi_ad9081_device_cbusjtx_register_get(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t *data,
					       uint8_t lane)
{
	return adi_ad9081_hal_cbusjtx_reg_get(device, addr, data, lane);
}

int32_t adi_ad9081_device_cbusjtx_register_set(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t data,
					       uint8_t lane)
{
	return adi_ad9081_hal_cbusjtx_reg_set(device, addr, data, lane);
}

int32_t adi_ad9081_device_cbuspll_register_get(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t *data)
{
	return adi_ad9081_hal_cbuspll_reg_get(device, addr, data);
}

int32_t adi_ad9081_device_cbuspll_register_set(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t data)
{
	return adi_ad9081_hal_cbuspll_reg_set(device, addr, data);
}

int32_t adi_ad9081_device_chip_id_get(adi_ad9081_device_t *device,
				      adi_cms_chip_id_t *chip_id)
{
	int32_t err;
	uint8_t reg_val = 0x0;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(chip_id);

	err = adi_ad9081_hal_reg_get(device, REG_CHIP_TYPE_ADDR, &reg_val);
	AD9081_ERROR_RETURN(err);
	chip_id->chip_type = reg_val;

	err = adi_ad9081_hal_reg_get(device, REG_PROD_ID_LSB_ADDR, &reg_val);
	AD9081_ERROR_RETURN(err);
	chip_id->prod_id = reg_val;
	err = adi_ad9081_hal_reg_get(device, REG_PROD_ID_MSB_ADDR, &reg_val);
	AD9081_ERROR_RETURN(err);
	chip_id->prod_id |= (reg_val << 8);

	err = adi_ad9081_hal_reg_get(device, REG_CHIP_GRADE_ADDR, &reg_val);
	AD9081_ERROR_RETURN(err);
	chip_id->prod_grade = (reg_val >> 4);
	chip_id->dev_revision = (reg_val & 0x0F);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_reset(adi_ad9081_device_t *device,
				adi_ad9081_reset_e operation)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_INVALID_PARAM_RETURN(operation > AD9081_HARD_RESET_AND_INIT);

	/* do reset */
	if ((operation == AD9081_SOFT_RESET) ||
	    (operation == AD9081_SOFT_RESET_AND_INIT)) {
		err = adi_ad9081_hal_reg_set(device, REG_SPI_INTFCONFA_ADDR,
					     0x81);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_reg_set(device, REG_SPI_INTFCONFA_ADDR,
					     0x00);
		AD9081_ERROR_RETURN(err);
	} else if ((operation == AD9081_HARD_RESET) ||
		   (operation == AD9081_HARD_RESET_AND_INIT)) {
		err = adi_ad9081_hal_reset_pin_ctrl(device, 0);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_delay_us(device, 100000);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_reset_pin_ctrl(device, 1);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_hal_delay_us(device, 100000);
		AD9081_ERROR_RETURN(err);
	}

	/* do init */
	if ((operation == AD9081_SOFT_RESET_AND_INIT) ||
	    (operation == AD9081_HARD_RESET_AND_INIT)) {
		err = adi_ad9081_device_init(device);
		AD9081_ERROR_RETURN(err);
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_api_revision_get(adi_ad9081_device_t *device,
					   uint8_t *rev_major,
					   uint8_t *rev_minor, uint8_t *rev_rc)
{
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(rev_major);
	AD9081_NULL_POINTER_RETURN(rev_minor);
	AD9081_NULL_POINTER_RETURN(rev_rc);

	*rev_major = ad9081_api_revision[0];
	*rev_minor = ad9081_api_revision[1];
	*rev_rc = ad9081_api_revision[2];

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_firmware_revision_get(adi_ad9081_device_t *device,
						uint32_t *rev)
{
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(rev);

	if (device->dev_info.dev_rev == 1) { /* r1 */
		return adi_ad9081_hal_reg_get(device, 0x01001578,
					      (uint8_t *)rev);
	}
	if (device->dev_info.dev_rev == 2) { /* r1r */
		return adi_ad9081_hal_reg_get(device, 0x2126, (uint8_t *)rev);
	}
	if (device->dev_info.dev_rev == 3) { /* r2 */
		return adi_ad9081_hal_reg_get(device, 0x2130, (uint8_t *)rev);
	}

	return API_CMS_ERROR_OK;
}

int32_t
adi_ad9081_device_firmware_patch_revision_get(adi_ad9081_device_t *device,
					      uint32_t *rev)
{
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(rev);

	if (device->dev_info.dev_rev == 1) { /* r1 */
		return adi_ad9081_hal_reg_get(device, 0x0100156c,
					      (uint8_t *)rev);
	}
	if (device->dev_info.dev_rev == 2) { /* r1r */
		return adi_ad9081_hal_reg_get(device, 0x2127, (uint8_t *)rev);
	}
	if (device->dev_info.dev_rev == 3) { /* r2 */
		return adi_ad9081_hal_reg_get(device, 0x2131, (uint8_t *)rev);
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_laminate_id_get(adi_ad9081_device_t *device,
					  uint8_t *id)
{
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(id);

	return adi_ad9081_hal_reg_get(device, 0x1e0d, (uint8_t *)id);
}

int32_t adi_ad9081_device_die_id_get(adi_ad9081_device_t *device, uint8_t *id)
{
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_NULL_POINTER_RETURN(id);

	/* [6:0]: 1/2/3 - R1, 4 - R1R */
	return adi_ad9081_hal_reg_get(device, 0x1e0e, (uint8_t *)id);
}

int32_t adi_ad9081_device_deinit(adi_ad9081_device_t *device)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* hardware reset */
	err = adi_ad9081_device_reset(device, AD9081_HARD_RESET);
	AD9081_ERROR_RETURN(err);

	/* software reset */
	err = adi_ad9081_device_reset(device, AD9081_SOFT_RESET);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_direct_loopback_set(adi_ad9081_device_t *device,
					      uint8_t mode, uint8_t mapping)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	err = adi_ad9081_hal_bf_set(device, REG_CLOCKING_CTRL_ADDR,
				    BF_DIRECT_LOOPBACK_MODE_INFO, mode);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_LOOPBACK_CB_CTRL_ADDR,
				    BF_LOOPBACK_CB_CTRL_INFO, mapping);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_startup_tx_or_nco_test(
	adi_ad9081_device_t *device, uint8_t main_interp, uint8_t chan_interp,
	uint8_t dac_chan[4], int64_t main_shift[4], int64_t chan_shift[8],
	adi_cms_jesd_param_t *jesd_param, uint8_t enable_nco_test)
{
	int32_t err;
	uint8_t i, links;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_INVALID_PARAM_RETURN(device->dev_info.dac_freq_hz == 0);
	AD9081_INVALID_PARAM_RETURN(main_interp == 0);
	AD9081_INVALID_PARAM_RETURN(chan_interp == 0);
	if (enable_nco_test == 0) {
		AD9081_NULL_POINTER_RETURN(jesd_param);
		AD9081_INVALID_PARAM_RETURN(jesd_param->jesd_l == 0);
		AD9081_INVALID_PARAM_RETURN(jesd_param->jesd_jesdv > 2);
	}

	/* enable tx */
	err = adi_ad9081_dac_tx_enable_set(device, AD9081_DAC_ALL, 1);
	AD9081_ERROR_RETURN(err);

	/* power up dac */
	err = adi_ad9081_dac_power_up_set(device, AD9081_DAC_ALL, 1);
	AD9081_ERROR_RETURN(err);

	/* startup dac dll */
	err = adi_ad9081_dac_dll_startup(device, AD9081_DAC_ALL);
	AD9081_ERROR_RETURN(err);

	/* disable tx */
	err = adi_ad9081_dac_tx_enable_set(device, AD9081_DAC_ALL, 0);
	AD9081_ERROR_RETURN(err);

	/* set default current */
	err = adi_ad9081_dac_fsc_set(device, AD9081_DAC_ALL, 26000);
	AD9081_ERROR_RETURN(err);

	/* setup interpolation */
	err = adi_ad9081_dac_interpolation_set(device, main_interp,
					       chan_interp);
	AD9081_ERROR_RETURN(err);

	/* configure jrx links */
	if (enable_nco_test == 0) {
		links = (jesd_param->jesd_duallink > 0) ? AD9081_LINK_ALL :
							  AD9081_LINK_0;
		err = adi_ad9081_jesd_rx_link_config_set(device, links,
							 jesd_param);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_jesd_rx_bring_up(device, links, 0xff);
		AD9081_ERROR_RETURN(err);
		err = adi_ad9081_jesd_rx_sysref_enable_set(
			device, jesd_param->jesd_subclass > 0 ? 1 : 0);
		AD9081_ERROR_RETURN(err);
	}

	/* set xbar (used channels by each DAC) */
	for (i = 0; i < 4; i++) {
		err = adi_ad9081_dac_xbar_set(device, AD9081_DAC_0 << i,
					      dac_chan[i]);
		AD9081_ERROR_RETURN(err);
	}

	/* set main nco */
	for (i = 0; i < 4; i++) {
		err = adi_ad9081_dac_duc_nco_set(device, AD9081_DAC_0 << i,
						 AD9081_DAC_CH_NONE,
						 main_shift[i]);
		AD9081_ERROR_RETURN(err);
	}

	/* set channel nco */
	if (chan_interp != 1) {
		for (i = 0; i < 8; i++) {
			err = adi_ad9081_dac_duc_nco_set(device,
							 AD9081_DAC_NONE,
							 AD9081_DAC_CH_0 << i,
							 chan_shift[i]);
			AD9081_ERROR_RETURN(err);
		}
	}

	/* disable soft off/on for pa protection */
	err = adi_ad9081_dac_soft_off_gain_enable_set(device, AD9081_DAC_ALL,
						      0);
	AD9081_ERROR_RETURN(err);

	/* enable tx */
	err = adi_ad9081_dac_tx_enable_set(device, AD9081_DAC_ALL, 1);
	AD9081_ERROR_RETURN(err);

	/* set shuffle */
	err = adi_ad9081_dac_shuffle_enable_set(device, AD9081_DAC_ALL, 1);
	AD9081_ERROR_RETURN(err);

	/* set data xor */
	err = adi_ad9081_dac_data_xor_set(device, AD9081_DAC_ALL, 1);
	AD9081_ERROR_RETURN(err);

	/* enable irq */
	err = adi_ad9081_dac_irqs_enable_set(device, 0x0030cccc00);
	AD9081_ERROR_RETURN(err);

	/* one shot sync */
	err = adi_ad9081_jesd_oneshot_sync(device);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_startup_tx(adi_ad9081_device_t *device,
				     uint8_t main_interp, uint8_t chan_interp,
				     uint8_t dac_chan[4], int64_t main_shift[4],
				     int64_t chan_shift[8],
				     adi_cms_jesd_param_t *jesd_param)
{
	int32_t err;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* configure datapath */
	err = adi_ad9081_device_startup_tx_or_nco_test(device, main_interp,
						       chan_interp, dac_chan,
						       main_shift, chan_shift,
						       jesd_param, 0);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int32_t
adi_ad9081_device_startup_nco_test(adi_ad9081_device_t *device,
				   uint8_t main_interp, uint8_t chan_interp,
				   uint8_t dac_chan[4], int64_t main_shift[4],
				   int64_t chan_shift[8], uint16_t dc_offset)
{
	int32_t err;
	uint8_t i;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* configure datapath */
	err = adi_ad9081_device_startup_tx_or_nco_test(device, main_interp,
						       chan_interp, dac_chan,
						       main_shift, chan_shift,
						       NULL, 1);
	AD9081_ERROR_RETURN(err);

	/* workaround to enable datapath output */
	err = adi_ad9081_hal_bf_set(device, REG_JESD_MODE_ADDR,
				    BF_TX_JESD_MODE_INFO, 16); /* not paged */
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_hal_bf_set(device, REG_JRX_DL_204C_0_ADDR,
				    BF_JRX_DL_204C_ENABLE_INFO,
				    1); /* not paged */
	AD9081_ERROR_RETURN(err);

	/* enable test mode */
	if (chan_interp == 1) {
		for (i = 0; i < 4; i++) {
			err = adi_ad9081_dac_duc_main_dc_test_tone_en_set(
				device, AD9081_DAC_0 << i, 1);
			AD9081_ERROR_RETURN(err);
			err = adi_ad9081_dac_duc_main_dc_test_tone_offset_set(
				device, AD9081_DAC_0 << i, dc_offset);
			AD9081_ERROR_RETURN(err);
		}
		for (i = 0; i < 8; i++) {
			err = adi_ad9081_dac_dc_test_tone_en_set(
				device, AD9081_DAC_CH_0 << i, 0);
			AD9081_ERROR_RETURN(err);
		}
	} else {
		for (i = 0; i < 4; i++) {
			err = adi_ad9081_dac_duc_main_dc_test_tone_en_set(
				device, AD9081_DAC_0 << i, 0);
			AD9081_ERROR_RETURN(err);
		}
		for (i = 0; i < 8; i++) {
			err = adi_ad9081_dac_dc_test_tone_en_set(
				device, AD9081_DAC_CH_0 << i, 1);
			AD9081_ERROR_RETURN(err);
			err = adi_ad9081_dac_dc_test_tone_offset_set(
				device, AD9081_DAC_CH_0 << i, dc_offset);
			AD9081_ERROR_RETURN(err);
		}
	}

	return API_CMS_ERROR_OK;
}

int32_t adi_ad9081_device_startup_rx(adi_ad9081_device_t *device, uint8_t cddcs,
				     uint8_t fddcs, int64_t cddc_shift[4],
				     int64_t fddc_shift[8], uint8_t cddc_dcm[4],
				     uint8_t fddc_dcm[8], uint8_t cc2r_en[4],
				     uint8_t fc2r_en[8],
				     adi_cms_jesd_param_t jesd_param[2],
				     adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2])
{
	int32_t err;
	uint8_t links;
	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	/* disable adc clock before setting up adc.
     * changing adc dividers while clock is on glitches digital clocks and causes
     * unwanted phase mis-alignment in data path
     */
	err = adi_ad9081_adc_clk_enable_set(device, 0);
	AD9081_ERROR_RETURN(err);

	/* power up adc */
	err = adi_ad9081_adc_power_up_set(device, AD9081_ADC_ALL, 1);
	AD9081_ERROR_RETURN(err);

	/* configure coarse and fine ddc */
	err = adi_ad9081_adc_config(device, cddcs, fddcs, cddc_shift,
				    fddc_shift, cddc_dcm, fddc_dcm, cc2r_en,
				    fc2r_en);
	AD9081_ERROR_RETURN(err);

	/* configure jtx links */
	links = jesd_param[0].jesd_duallink > 0 ? AD9081_LINK_ALL :
						  AD9081_LINK_0;
	if ((links & AD9081_LINK_0) > 0) {
		err = adi_ad9081_jesd_tx_link_config_set(device, AD9081_LINK_0,
							 &jesd_param[0]);
		AD9081_ERROR_RETURN(err);
	}
	if ((links & AD9081_LINK_1) > 0) {
		err = adi_ad9081_jesd_tx_link_config_set(device, AD9081_LINK_1,
							 &jesd_param[1]);
		AD9081_ERROR_RETURN(err);
	}
	err = adi_ad9081_jesd_tx_bring_up(device, links, 0xff, jesd_conv_sel);
	AD9081_ERROR_RETURN(err);

	/* enable adc clock after adc setup and synchronize */
	err = adi_ad9081_adc_clk_enable_set(device, 1);
	AD9081_ERROR_RETURN(err);

	/* one shot sync */
	err = adi_ad9081_jesd_oneshot_sync(device);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

/*! @} */