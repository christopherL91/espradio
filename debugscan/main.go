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
	dumpIntMatrix()

	// MAC liveness: the TSF microsecond counter only advances when the MAC
	// engine is actually running.  Frozen TSF with clean init = the MAC is
	// clock-gated, held in reset, or unpowered — and explains zero hardware
	// interrupts (not even TX-complete) better than any RF-level theory.
	t0 := espradio.DebugTSF()
	time.Sleep(100 * time.Millisecond)
	t1 := espradio.DebugTSF()
	println("TSF:", int64(t0), "->", int64(t1), "delta(us):", int64(t1-t0))

	// WiFi MAC RX DMA state (block at 0x600A4000).  hal_mac_rx_set_base
	// writes the RX descriptor-list base at +0x84; +0x80 is the reload
	// control.  A zero base means the blob never armed RX DMA at all; a
	// non-zero base pointing outside HP-SRAM (0x40800000..0x4087FFFF) means
	// the descriptors are somewhere the MAC DMA cannot reach.
	const macBase = uintptr(0x600A4000)
	rxBase := reg(macBase + 0x84)
	println("MAC RX dscr base:", hex32(rxBase), "reload/en:", hex32(reg(macBase+0x80)),
		"cur/next:", hex32(reg(macBase+0x88)))
	// The MAC stores the descriptor address with the top nibble stripped
	// (addr & 0xFFFFFF); HP-SRAM is 0x40800000..0x4087FFFF, so the real
	// descriptor address is 0x40000000 | (reg & 0xFFFFFF).
	descAddr := uintptr(0x40000000 | (rxBase & 0x00FFFFFF))
	println("MAC RX dscr real addr:", hex32(uint32(descAddr)))
	// Walk up to 12 descriptors following the next pointer.  Each WiFi RX
	// descriptor is 3 words: [0]=flags (bit31 = owner: 1=HW-owned/available),
	// [1]=buffer pointer, [2]=next pointer.  All-SW-owned (bit31=0) from the
	// first descriptor means the ring was handed to hardware already
	// consumed, which is exactly the "rx buffer full from frame #1" symptom.
	seen := map[uintptr]bool{}
	for i := 0; i < 12; i++ {
		if descAddr < 0x40800000 || descAddr >= 0x40880000 || seen[descAddr] {
			break
		}
		seen[descAddr] = true
		w0 := reg(descAddr)
		w1 := reg(descAddr + 4)
		w2 := reg(descAddr + 8)
		owner := (w0 >> 31) & 1
		println("  dsc", i, "@", hex32(uint32(descAddr)),
			"flags:", hex32(w0), "owner(hw=1):", owner,
			"buf:", hex32(w1), "next:", hex32(w2))
		descAddr = uintptr(0x40000000 | (w2 & 0x00FFFFFF))
	}

	// Full MAC control-register block (0x600A4000..0x600A40FC).  RX aborts at
	// clock rate with a valid HW-owned ring, so the RX-datapath enable/filter
	// must be wrong at a register we haven't named — dump the whole block so
	// it can be compared against the documented reset/enabled values.
	// Dump through +0x200 so the 11ax RX-buffer-queue registers written by
	// mac_last_rxbuf_init (0x600A4120..0x600A416C) are captured too — on the
	// C6 those, not just the legacy +0x84 base, may gate RX DMA.
	println("MAC block 0x600A4000 (offset: w0 w1 w2 w3):")
	for off := uintptr(0); off < 0x200; off += 0x10 {
		b := macBase + off
		println("  +"+hex32(uint32(off))[6:],
			hex32(reg(b)), hex32(reg(b+4)), hex32(reg(b+8)), hex32(reg(b+0xc)))
	}

	// EXPERIMENT: force the MAC to re-read the RX descriptor list.  Bit 0 of
	// 0x600A4080 is the "descriptor reload" trigger (hal_mac_rx_set_dscr_reload
	// sets it).  If the MAC latched an empty/stale ring at the blob's
	// set_base time — before our cooperatively-scheduled descriptor writes
	// were visible to the MAC DMA — the ring never gets picked up and every
	// frame reports "buffer full" despite a valid HW-owned ring.  Poking the
	// reload bit now (descriptors long since in SRAM) should make the DMA
	// re-fetch and start cycling.
	println("EXPERIMENT: triggering RX descriptor reload (0x600A4080 |= 1)")
	writeReg(0x600A4080, reg(0x600A4080)|1)
	time.Sleep(50 * time.Millisecond)
	println("  after reload: reload/en:", hex32(reg(0x600A4080)),
		"cur/next:", hex32(reg(0x600A4088)), "last:", hex32(reg(0x600A408C)))

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

	// EXPERIMENT 2: continuous RX descriptor reload during the sniff.  If the
	// DMA never loads the ring because the periodic reload the ISR refill path
	// (wDev_AppendRxBlocks) normally issues never runs, then hammering the
	// reload bit from a goroutine — which yields to the blob during its
	// Sleep — should make frames start landing.  A goroutine pulses the reload
	// bit every 2ms; SniffCountOnChannel's internal sleeps let it run.
	reloadPulsing := true
	go func() {
		for reloadPulsing {
			writeReg(0x600A4080, reg(0x600A4080)|1)
			time.Sleep(2 * time.Millisecond)
		}
	}()

	for _, ch := range []uint8{1, 6, 11} {
		n, err := espradio.SniffCountOnChannel(ch, 2*time.Second)
		if err != nil {
			println("sniff error ch", ch, err.Error())
		} else {
			println("sniff(reload) ch", ch, "packets:", n,
				"cur:", hex32(reg(0x600A4088)), "last:", hex32(reg(0x600A408C)))
		}
	}
	reloadPulsing = false
	println("hw ISRs after sniff:", espradio.DebugHWISRCount())
	for i := 0; i < 2; i++ {
		aps, err := espradio.Scan()
		if err != nil {
			println("could not scan wifi:", err.Error())
			break
		}
		println("scan", i, "found", len(aps), "APs; hw ISRs:", espradio.DebugHWISRCount())
		for _, ap := range aps {
			println("AP:", ap.SSID, "RSSI", ap.RSSI)
		}
	}
	// TX/RX counters straight from the blob: zero TX attempts means frames
	// never reach the MAC (queue/coex/scheduling); TX attempts without
	// completions means the MAC engine never finishes anything.
	espradio.DebugStatisDump()
	for i := 0; i < 10; i++ { // let the log drain through schedOnce
		time.Sleep(50 * time.Millisecond)
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
