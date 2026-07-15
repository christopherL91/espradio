package main

import (
	"strconv"
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

// isrRate measures WiFi ISR invocations over the given window.
func isrRate(tag string, d time.Duration) uint32 {
	start := espradio.DebugISRCount()
	time.Sleep(d)
	n := espradio.DebugISRCount() - start
	println(tag, "ISR count over", int64(d/time.Millisecond), "ms:", n)
	return n
}

func hex32(v uint32) string { return "0x" + strconv.FormatUint(uint64(v), 16) }

// dumpDiagForever prints the RAM-captured init diagnostics slowly and
// repeatedly: the USB-Serial-JTAG console drops/interleaves bytes around
// reset and under load, so a single fast print often arrives mangled.
func dumpDiagForever(reason string) {
	for {
		w := espradio.DebugInitDiagWords()
		println("ENABLE FAILED:", reason)
		time.Sleep(200 * time.Millisecond)
		println("diag rc=" + hex32(w[0]) + " osiVer=" + hex32(w[1]) + " osiMagic=" + hex32(w[2]) +
			" cfgMagic=" + hex32(w[5]))
		time.Sleep(200 * time.Millisecond)
		println("diag osiPtr=" + hex32(w[3]) + " gOsiPtr=" + hex32(w[4]) +
			" coexPre=" + hex32(w[6]) + " coexInit=" + hex32(w[7]) +
			" arena=" + strconv.FormatUint(uint64(w[8]), 10) + "/" + strconv.FormatUint(uint64(w[9]), 10))
		time.Sleep(200 * time.Millisecond)
		println("diaglog:", espradio.DebugInitDiagLog())
		time.Sleep(3 * time.Second)
	}
}

func main() {
	time.Sleep(time.Second)
	println("initializing radio...")
	err := espradio.Enable(espradio.Config{Logging: espradio.LogLevelInfo})
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

	// Idle ISR rate: >1000/s with nothing happening means a storming
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
