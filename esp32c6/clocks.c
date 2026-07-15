//go:build esp32c6

/* WiFi/PHY power and clock control for the ESP32-C6.
 *
 * Unlike the C3/S3 (RTC_CNTL + APB_CTRL), the C6 gates the radio through the
 * PMU and the MODEM_SYSCON / MODEM_LPCON blocks.  Power domains themselves
 * are handled by the PMU, so there is no explicit power-domain or isolation
 * sequencing here (esp_wifi_bt_power_domain_on is a no-op in ESP-IDF on this
 * chip, and esp-hal's reset_mac is empty).
 *
 * The register sequence mirrors esp-hal's esp32c6 radio_clocks.rs
 * (init_clocks + wifi_clock_enable + enable_phy), which is itself a
 * flattened version of ESP-IDF's modem_clock_module_enable().
 */

#include <stdint.h>
#include <stdbool.h>

#include "soc/reg_base.h"
#include "hal/pmu_ll.h"
#include "hal/modem_syscon_ll.h"
#include "hal/modem_lpcon_ll.h"

/* The vendored reg_base.h does not carry the modem block bases; values are
 * from ESP-IDF soc/esp32c6 (and match TinyGo's device/esp/esp32c6.go). */
#define ESPRADIO_PMU           ((pmu_dev_t *)DR_REG_PMU_BASE)
#define ESPRADIO_MODEM_SYSCON  ((modem_syscon_dev_t *)0x600A9800u)
#define ESPRADIO_MODEM_LPCON   ((modem_lpcon_dev_t *)0x600AF000u)

static volatile uint32_t s_clock_refcnt;

static void espradio_c6_clocks_init_once(void) {
    pmu_dev_t *pmu = ESPRADIO_PMU;
    modem_syscon_dev_t *syscon = ESPRADIO_MODEM_SYSCON;
    modem_lpcon_dev_t *lpcon = ESPRADIO_MODEM_LPCON;

    /* Modem clock-gate codes per PMU power mode, then latch them. */
    pmu_ll_hp_set_icg_modem(pmu, PMU_MODE_HP_SLEEP, 0);
    pmu_ll_hp_set_icg_modem(pmu, PMU_MODE_HP_MODEM, 1);
    pmu_ll_hp_set_icg_modem(pmu, PMU_MODE_HP_ACTIVE, 2);
    pmu_ll_imm_update_dig_icg_modem_code(pmu, true);
    pmu_ll_imm_update_dig_icg_switch(pmu, true);

    /* Clock-gate state maps for the modem power states. */
    syscon->clk_conf_power_st.clk_modem_apb_st_map = 6;
    syscon->clk_conf_power_st.clk_modem_peri_st_map = 4;
    syscon->clk_conf_power_st.clk_wifi_st_map = 6;
    syscon->clk_conf_power_st.clk_bt_st_map = 6;
    syscon->clk_conf_power_st.clk_fe_st_map = 6;
    syscon->clk_conf_power_st.clk_zb_st_map = 6;

    lpcon->clk_conf_power_st.clk_lp_apb_st_map = 6;
    lpcon->clk_conf_power_st.clk_i2c_mst_st_map = 6;
    lpcon->clk_conf_power_st.clk_coex_st_map = 6;
    lpcon->clk_conf_power_st.clk_wifipwr_st_map = 6;

    /* WiFi power clock source selection. */
    lpcon->wifi_lp_clk_conf.clk_wifipwr_lp_sel_osc_slow = 1;
    lpcon->wifi_lp_clk_conf.clk_wifipwr_lp_sel_osc_fast = 1;
    lpcon->wifi_lp_clk_conf.clk_wifipwr_lp_sel_xtal32k = 1;
    lpcon->wifi_lp_clk_conf.clk_wifipwr_lp_sel_xtal = 1;
    lpcon->wifi_lp_clk_conf.clk_wifipwr_lp_div_num = 0;

    modem_lpcon_ll_enable_wifipwr_clock(lpcon, true);
}

static void espradio_c6_wifi_clocks(bool en) {
    modem_syscon_dev_t *syscon = ESPRADIO_MODEM_SYSCON;
    modem_lpcon_dev_t *lpcon = ESPRADIO_MODEM_LPCON;

    modem_syscon_ll_enable_wifi_apb_clock(syscon, en);
    modem_syscon_ll_enable_wifi_mac_clock(syscon, en);
    modem_syscon_ll_enable_fe_apb_clock(syscon, en);
    modem_syscon_ll_enable_fe_cal_160m_clock(syscon, en);
    modem_syscon_ll_enable_fe_160m_clock(syscon, en);
    modem_syscon_ll_enable_fe_80m_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_160x1_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_80x1_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_40x1_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_80x_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_40x_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_80m_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_44m_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_40m_clock(syscon, en);
    modem_syscon_ll_enable_wifibb_22m_clock(syscon, en);

    modem_lpcon_ll_enable_wifipwr_clock(lpcon, en);
    modem_lpcon_ll_enable_coex_clock(lpcon, en);

    /* PHY: I2C master clock, sourced from 160 MHz. */
    lpcon->clk_conf.clk_i2c_mst_en = en;
    lpcon->i2c_mst_clk_conf.clk_i2c_mst_sel_160m = en;
}

void espradio_hal_init_clocks_go(void) {
    if (__sync_fetch_and_add(&s_clock_refcnt, 1u) != 0u) {
        return;
    }
    espradio_c6_clocks_init_once();
    espradio_c6_wifi_clocks(true);
}

void espradio_hal_disable_clocks_go(void) {
    uint32_t cur = s_clock_refcnt;
    if (cur == 0u) {
        return;
    }
    if (__sync_sub_and_fetch(&s_clock_refcnt, 1u) != 0u) {
        return;
    }
    espradio_c6_wifi_clocks(false);
}

/* The C6 has no RTC_CNTL WiFi isolation controls; the PMU manages domain
 * isolation on its own. */
void espradio_hal_wifi_rtc_enable_iso_go(void) {
}

void espradio_hal_wifi_rtc_disable_iso_go(void) {
}

/* OSI _wifi_reset_mac hook — deliberately EMPTY on the C6, matching esp-hal
 * (which drives these same blobs successfully).
 *
 * The blob calls this from wifi_hw_start(), during esp_wifi_start — i.e.
 * AFTER RX DMA has already been armed at init time (wifi_lmac_init →
 * ic_init → wDev_Rxbuf_Init → hal_mac_rx_set_base).  A real MODEM_SYSCON
 * WiFi-MAC reset pulse here wipes the RX descriptor-list base register and
 * the blob does not re-arm it under our cooperative scheduling, so the MAC
 * then receives frames but drops every one with "rx buffer full" (HW RX
 * full#1 == rx_end, rx_suc == 0).  IDF's wrapper does pulse the reset, but
 * its start path differs; esp-hal's empty version is the safe match for
 * this port. */
void espradio_hal_reset_wifi_mac_go(void) {
}

/* Referenced by libphy (phy_get_xtal_freq).  The ESP32-C6 always runs from
 * a 40 MHz crystal; ESP-IDF returns the frequency in MHz. */
uint32_t rtc_clk_xtal_freq_get(void) {
    return 40u;
}
