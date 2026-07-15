//go:build esp32c6

package esp32c6

// #cgo CFLAGS: -I../blobs/include
// #cgo CFLAGS: -I../blobs/include/esp32c6
// #cgo CFLAGS: -I../blobs/include/local
// #cgo CFLAGS: -I../blobs/headers
// #cgo CFLAGS: -I..
// #cgo CFLAGS: -DCONFIG_SOC_WIFI_NAN_SUPPORT=0
// #cgo CFLAGS: -DESPRADIO_PHY_PATCH_ROMFUNCS=0
import "C"

import (
	"device/esp"
	"unsafe"
)

// Clock and power control lives in clocks.c: on the ESP32-C6 the modem is
// gated through the PMU + MODEM_SYSCON/MODEM_LPCON blocks, and the vendored
// ESP-IDF ll headers give named access to those registers.  This file only
// provides the eFuse MAC read, which is easiest through device/esp.

//export espradio_hal_read_mac_go
func espradio_hal_read_mac_go(mac *C.uchar, iftype C.uint) C.int {
	if mac == nil {
		return -1
	}

	w0 := esp.EFUSE.GetRD_MAC_SPI_SYS_0()
	w1 := esp.EFUSE.GetRD_MAC_SPI_SYS_1_MAC_1()

	m := (*[6]byte)(unsafe.Pointer(mac))
	m[0] = byte((w1 >> 8) & 0xff)
	m[1] = byte(w1 & 0xff)
	m[2] = byte((w0 >> 24) & 0xff)
	m[3] = byte((w0 >> 16) & 0xff)
	m[4] = byte((w0 >> 8) & 0xff)
	m[5] = byte(w0 & 0xff)

	if iftype != 0 {
		m[0] |= 0x02
		m[5] = byte(uint32(m[5]) + uint32(iftype))
	}

	return 0
}
