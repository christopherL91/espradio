package main

import (
	"time"
	"unsafe"

	"tinygo.org/x/espradio"
)

func reg(addr uintptr) uint32 { return *(*uint32)(unsafe.Pointer(addr)) }

// ESP32-C6 interrupt matrix: one word per source at 0x60010000.
// Source 0 = WIFI_MAC, 2 = WIFI_PWR.
const (
	intmtxBase = uintptr(0x60010000)
	mtxWifiMac = intmtxBase + 0*4
	mtxWifiPwr = intmtxBase + 2*4
)

func writeReg(addr uintptr, v uint32) { *(*uint32)(unsafe.Pointer(addr)) = v }

func dumpIntState(tag string) {
	println(tag,
		"EN:", reg(0x20001000),
		"TYPE:", reg(0x20001004),
		"EIP:", reg(0x2000100C),
		"PRI1:", reg(0x20001014),
		"THRESH:", reg(0x20001090),
		"MTX_MAC:", reg(mtxWifiMac),
		"MTX_PWR:", reg(mtxWifiPwr))
}

func arenaStats(tag string) {
	used, capacity := espradio.ArenaStats()
	println(tag, "arena used:", used, "/", capacity)
}

// isrRate measures WiFi ISR invocations over the given window.  The total
// includes schedOnce's soft polls (dominated by scheduler churn); "hw" is
// actual hardware interrupt deliveries, which is the number that matters
// when hunting a storming interrupt source.
func isrRate(tag string, d time.Duration) uint32 {
	start := espradio.DebugISRCount()
	hwStart := espradio.DebugHWISRCount()
	time.Sleep(d)
	n := espradio.DebugISRCount() - start
	hw := espradio.DebugHWISRCount() - hwStart
	println(tag, "over", int64(d/time.Millisecond), "ms: hw:", hw, "total(+soft):", n)
	return hw
}

// dumpIntMatrix lists every peripheral interrupt source routed to a CPU
// interrupt, so a storming source that the blob routed behind our back
// (via ROM intr_matrix_set) is visible.  ESP32-C6 has 77 sources.
func dumpIntMatrix() {
	for src := uintptr(0); src < 77; src++ {
		v := reg(intmtxBase + src*4)
		if v != 0 {
			println("  intmtx src", int(src), "-> cpu int", v)
		}
	}
}

// hex32 renders v as fixed-width hex with no library calls (strconv and
// printf-family formatting are exactly what's under suspicion when memory
// or varargs handling is broken).
func hex32(v uint32) string {
	const digits = "0123456789abcdef"
	var b [10]byte
	b[0], b[1] = '0', 'x'
	for i := 0; i < 8; i++ {
		b[2+i] = digits[(v>>(28-4*uint(i)))&0xf]
	}
	return string(b[:])
}

// diagNames must match the ESPRADIO_DIAG_* enum order in espradio.h.
var diagNames = [16]string{
	"rc", "osiVer", "osiMagic", "osiPtr", "gOsiPtr", "cfgMagic",
	"coexPre", "coexInit", "arenaUsed", "arenaCap",
	"cfgAddr", "gd0", "gd1", "wpaPre", "wpaPost", "vaTest",
}

// dumpDiagForever prints the RAM-captured init diagnostics slowly and
// repeatedly: the USB-Serial-JTAG console drops/interleaves bytes around
// reset and under load, so a single fast print often arrives mangled.
// One value per line, fixed-width, repeated forever — a lossy console
// eventually delivers every line intact.
func dumpDiagForever(reason string) {
	for {
		w := espradio.DebugInitDiagWords()
		println("ENABLE FAILED:", reason)
		for i, name := range diagNames {
			time.Sleep(100 * time.Millisecond)
			println("D:", name, hex32(w[i]))
		}
		time.Sleep(200 * time.Millisecond)
		println("diaglog:", espradio.DebugInitDiagLog())
		time.Sleep(3 * time.Second)
	}
}

func main() {
	time.Sleep(time.Second)
	println("initializing radio...")
	err := espradio.Enable(espradio.Config{Logging: espradio.LogLevelNone})
	if err != nil {
		dumpDiagForever(err.Error())
	}
	arenaStats("after-enable")
	println("starting radio...")
	err = espradio.Start()
	if err != nil {
		// Keep going: the failed-start state is exactly what we want to
		// inspect (int state, ISR storm rate, whether scan errors out).
		println("could not start radio:", err.Error())
	}
	arenaStats("after-start")
	dumpIntState("after-start")
	dumpIntMatrix()

	// Idle hardware ISR rate: >1000/s with nothing happening means a storming
	// interrupt source. Isolate which one by masking the INTMTX routes
	// one at a time (0 = detached from any CPU interrupt).
	if n := isrRate("idle", 500*time.Millisecond); n > 500 {
		savedMac := reg(mtxWifiMac)
		savedPwr := reg(mtxWifiPwr)

		writeReg(mtxWifiPwr, 0)
		isrRate("idle (WIFI_PWR detached)", 500*time.Millisecond)
		writeReg(mtxWifiPwr, savedPwr)

		writeReg(mtxWifiMac, 0)
		isrRate("idle (WIFI_MAC detached)", 500*time.Millisecond)
		writeReg(mtxWifiMac, savedMac)
	}

	for _, ch := range []uint8{1, 6, 11} {
		n, err := espradio.SniffCountOnChannel(ch, 2*time.Second)
		if err != nil {
			println("sniff error ch", ch, err.Error())
		} else {
			println("sniff ch", ch, "packets:", n)
		}
	}
	println("ISR count:", espradio.DebugISRCount())
	for i := 0; i < 2; i++ {
		aps, err := espradio.Scan()
		if err != nil {
			println("could not scan wifi:", err.Error())
			break
		}
		println("scan", i, "found", len(aps), "APs")
		for _, ap := range aps {
			println("AP:", ap.SSID, "RSSI", ap.RSSI)
		}
	}
	dumpIntState("after-scan")
	arenaStats("after-scan")
	println("done; ISR count:", espradio.DebugISRCount())
	for {
		time.Sleep(5 * time.Second)
		// Repeat the captured diagnostics so a late console attach still
		// sees them, even on a "successful" run.
		println("diaglog:", espradio.DebugInitDiagLog())
	}
}
