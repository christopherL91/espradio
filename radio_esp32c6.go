//go:build esp32c6

package espradio

/*
#cgo CFLAGS: -Iblobs/include
#cgo CFLAGS: -Iblobs/include/esp32c6
#cgo CFLAGS: -Iblobs/include/local
#cgo CFLAGS: -Iblobs/headers
#cgo CFLAGS: -DCONFIG_SOC_WIFI_NAN_SUPPORT=0
#cgo CFLAGS: -DESPRADIO_PHY_PATCH_ROMFUNCS=0
#cgo LDFLAGS: -Lblobs/libs/esp32c6 -lcoexist -lcore -lmesh -lnet80211 -lespnow -lregulatory -lphy -lpp -lwpa_supplicant

#include "include.h"
*/
import "C"

import (
	"runtime/interrupt"
	"unsafe"

	_ "tinygo.org/x/espradio/esp32c6"
)

// ─── Hardware init ───────────────────────────────────────────────────────────

// CPU interrupt number for WiFi MAC. On RISC-V, interrupt 1 is valid.
const wifiCPUInterrupt = 1

func initHardware() error {
	// The PMU manages the modem power domain on the C6
	// (esp_wifi_bt_power_domain_on is a no-op in ESP-IDF), and the modem
	// clocks are enabled by espradio_hal_init_clocks_go() from Enable().
	//
	// What we DO have to do: zero the WiFi ROM's BSS. On the C6, pp,
	// net80211 and coexist live in the mask ROM and keep their state at
	// fixed addresses near the top of HP SRAM (esp32c6.rom.pp.ld:
	// g_osi_funcs_p, pTxRx, rate-scheduler tables, ...). That region is
	// outside our linker image (reserved, see targets/esp32c6.ld), and with
	// no second-stage bootloader nothing else clears it — after a soft
	// reset it holds whatever the previous firmware left there, and the
	// blob skips critical init when it finds e.g. g_osi_funcs_p != 0.
	// Range: first pp symbol (esp_wifi_cert_tx_nss, 0x4087fcec) through the
	// last coexist symbol (coex_env_ptr+4, 0x4087ffc8); phy_param_rom at
	// 0x4087fce8 is ROM-owned and left alone.
	const romWifiBssStart = 0x4087fcec
	const romWifiBssEnd = 0x4087ffc8
	for addr := uintptr(romWifiBssStart); addr < romWifiBssEnd; addr += 4 {
		*(*uint32)(unsafe.Pointer(addr)) = 0
	}
	return nil
}

// Systimer ticks, same rate as the ESP32-C3.
const ticksPerSecond = 16_000_000

// The C6 has 512KB HP SRAM.  Its blob generation (IDF 5.3-era, 802.11ax)
// allocates noticeably more through the OSI allocators than the C3's: the
// wifi NVS shadow alone is ~4.4KB, the STA interface pair ~1.9KB, plus HE/TWT
// state and the static RX buffer pool (10 × ~1.6KB).  48KB was borderline and
// arena OOM makes the blob silently skip creating critical objects, so give
// the C6 more headroom.
const arenaPoolSize = 80 * 1024

// ESP32-C6 (RISC-V): call the blob's WiFi ISR directly from the
// hardware interrupt handler, same as the ESP32-C3.
func wifiISRHandler(interrupt.Interrupt) {
	C.espradio_call_wifi_isr()
	kickSched()
}
