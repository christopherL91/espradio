//go:build esp32c6

#include <stdint.h>
#include "espradio.h"
#include "soc/interrupts.h"
#include "soc/reg_base.h"

/* ---- Interrupt controller registers (ESP32-C6) ----
 *
 * The C6 uses the PLIC (Platform-Level Interrupt Controller) machine-mode
 * registers at DR_REG_PLIC_MX_BASE as its CPU interrupt controller — NOT the
 * INTPRI block at 0x600C5000, which is a vestigial backward-compatibility
 * alias not wired to the CPU.  Same conclusion as TinyGo's
 * runtime/interrupt/interrupt_esp32c6.go, which this file must cooperate
 * with. */

#define ESPRADIO_PLIC_ENABLE_REG   (*(volatile uint32_t *)(DR_REG_PLIC_MX_BASE + 0x00u))
#define ESPRADIO_PLIC_TYPE_REG     (*(volatile uint32_t *)(DR_REG_PLIC_MX_BASE + 0x04u))
#define ESPRADIO_PLIC_CLEAR_REG    (*(volatile uint32_t *)(DR_REG_PLIC_MX_BASE + 0x08u))
#define ESPRADIO_PLIC_PRI_REG(n)   (*(volatile uint32_t *)(DR_REG_PLIC_MX_BASE + 0x10u + (uint32_t)(n) * 4u))

/* Interrupt matrix: one 32-bit register per peripheral source at
 * DR_REG_INTMTX_BASE + 4*source, holding the target CPU interrupt number
 * (0 = disabled, 1-31 = active).  Written directly instead of through the
 * intr_matrix_set ROM function so routing cannot depend on ROM behavior. */
#define ESPRADIO_INTMTX_MAP(source)  (*(volatile uint32_t *)(DR_REG_INTMTX_BASE + (uint32_t)(source) * 4u))

/* The CPU interrupt number used for all WiFi peripheral sources.
 * TinyGo registers its handler on this interrupt via interrupt.New(). */
#define ESPRADIO_WIFI_CPU_INT  1u

/* TinyGo routes ETS_GPIO_INTR_SOURCE to this CPU interrupt
 * (cpuInterruptFromPin in machine_esp32c6.go).  Restored after every
 * schedOnce() so that blob ROM calls (e.g. direct intr_matrix_set) cannot
 * permanently steal the GPIO source. */
#define ESPRADIO_GPIO_CPU_INT  6u

/* Pre-wire WiFi peripheral interrupt sources to the WiFi CPU interrupt.
 * Must be called before esp_wifi_init so routing is in place before the
 * blob enables the peripheral-side interrupts. */
void espradio_prewire_wifi_interrupts(void) {
    /* Routing to the CPU happens in espradio_wifi_int_to_level(), which
     * runs AFTER esp_wifi_init: a freshly clocked MAC/PWR block can hold
     * asserted interrupt status that nothing can acknowledge until the
     * blob's handlers exist, and the resulting storm starves the CPU
     * during the blob's own init.

     * The C6 blob registers its handlers via set_isr() under both n=0 and
     * n=1 (the Rust esp-wifi port maps both to the same handler).  Mark all
     * slots the blob might use so espradio_call_wifi_isr() services them;
     * unregistered slots stay NULL and are skipped. */
    espradio_mark_wifi_isr_slot(0);
    espradio_mark_wifi_isr_slot(1);
    espradio_mark_wifi_isr_slot(2);
}

extern void espradio_mark_wifi_isr_slot(int32_t n);


/* No-op: routing is already configured by espradio_prewire_wifi_interrupts().
 * Letting the blob touch the interrupt matrix at arbitrary times interferes
 * with TinyGo's interrupt controller state.  The Rust esp-wifi does the same
 * (no-op set_intr).  Record the blob's requested intr_num as a WiFi ISR slot. */
void espradio_set_intr(int32_t cpu_no, uint32_t intr_source, uint32_t intr_num, int32_t intr_prio) {
    (void)cpu_no;
    (void)intr_source;
    (void)intr_prio;
    espradio_mark_wifi_isr_slot((int32_t)intr_num);
}

void espradio_clear_intr(uint32_t intr_source, uint32_t intr_num) {
    (void)intr_source;
    (void)intr_num;
}

/* Enable/disable CPU interrupts via the PLIC enable register instead of
 * ROM functions (ets_isr_unmask / ets_isr_mask) which may have side effects
 * that conflict with TinyGo's interrupt controller setup. */
void espradio_ints_on(uint32_t mask) {
    ESPRADIO_PLIC_ENABLE_REG |= mask;
}

void espradio_ints_off(uint32_t mask) {
    ESPRADIO_PLIC_ENABLE_REG &= ~mask;
}

/* Switch the WiFi CPU interrupt from edge (TinyGo's Enable() default) to
 * level type.  Must be called AFTER esp_wifi_init() so the blob's ISR
 * handlers are registered and can acknowledge the peripheral when a level
 * interrupt fires.
 *
 * Sequence: disable → clear latched edge → switch to level → fence → re-enable.
 */
void espradio_wifi_int_to_level(void) {
    /* Route the WiFi peripheral sources now that the blob's ISR handlers
     * are registered and can acknowledge them. */
    ESPRADIO_INTMTX_MAP(ETS_WIFI_MAC_INTR_SOURCE) = ESPRADIO_WIFI_CPU_INT;
    ESPRADIO_INTMTX_MAP(ETS_WIFI_PWR_INTR_SOURCE) = ESPRADIO_WIFI_CPU_INT;

    ESPRADIO_PLIC_ENABLE_REG &= ~(1u << ESPRADIO_WIFI_CPU_INT);
    ESPRADIO_PLIC_CLEAR_REG  |=  (1u << ESPRADIO_WIFI_CPU_INT);
    ESPRADIO_PLIC_CLEAR_REG  &= ~(1u << ESPRADIO_WIFI_CPU_INT);
    ESPRADIO_PLIC_TYPE_REG   &= ~(1u << ESPRADIO_WIFI_CPU_INT);
    __asm__ volatile ("fence" ::: "memory");
    ESPRADIO_PLIC_ENABLE_REG |=  (1u << ESPRADIO_WIFI_CPU_INT);
}

/* Raise WiFi CPU interrupt priority above the PLIC threshold (5, set by
 * TinyGo's runtime init) so delivery is unambiguous regardless of whether
 * the comparison is > or >= threshold. */
void espradio_wifi_int_raise_priority(void) {
    ESPRADIO_PLIC_PRI_REG(ESPRADIO_WIFI_CPU_INT) = 6u;
    __asm__ volatile ("fence" ::: "memory");
}

/* PLIC enable snapshot taken at the start of schedOnce(), before any blob
 * code runs.  espradio_wifi_unmask() ORs this back so that bits cleared by
 * blob OS-adapter ints_off or ROM calls during processing are restored —
 * e.g. bit 6 which TinyGo uses for GPIO. */
static volatile uint32_t s_intenable_snapshot;

void espradio_snapshot_intenable(void) {
    s_intenable_snapshot = ESPRADIO_PLIC_ENABLE_REG;
}

/* No-op on RISC-V: PS.INTLEVEL does not exist. */
void espradio_lower_intlevel(void) {
}

/* Called at the end of espradio_call_wifi_isr().  In level-triggered
 * mode, mask the WiFi CPU interrupt via the enable register to prevent
 * re-entry if the hardware line is still asserted after the blob ISR ran.
 * The bottom-half (schedOnce) unmasks after processing queued work. */
void espradio_wifi_isr_post_mask(void) {
    if ((ESPRADIO_PLIC_TYPE_REG & (1u << ESPRADIO_WIFI_CPU_INT)) == 0) {
        ESPRADIO_PLIC_ENABLE_REG &= ~(1u << ESPRADIO_WIFI_CPU_INT);
    }
}

void espradio_wifi_unmask(void) {
    /* Restore any TinyGo-owned PLIC enable bits that blob code may have
     * cleared, then ensure the WiFi CPU interrupt is enabled. */
    ESPRADIO_PLIC_ENABLE_REG |= s_intenable_snapshot | (1u << ESPRADIO_WIFI_CPU_INT);

    /* Re-route GPIO source → TinyGo's CPU interrupt in case blob ROM code
     * (direct intr_matrix_set calls inside the binary) corrupted it during
     * schedOnce() processing. */
    ESPRADIO_INTMTX_MAP(ETS_GPIO_INTR_SOURCE) = ESPRADIO_GPIO_CPU_INT;

    /* Re-fire GPIO CPU interrupt if it was registered and its PLIC enable
     * bit was cleared during schedOnce (blob ets_isr_mask or ints_off).
     * GPIO is level-triggered so toggling the ENABLE bit causes the
     * controller to re-sample the level and assert the interrupt if the
     * GPIO source is still pending. */
    if (s_intenable_snapshot & (1u << ESPRADIO_GPIO_CPU_INT)) {
        ESPRADIO_PLIC_ENABLE_REG &= ~(1u << ESPRADIO_GPIO_CPU_INT);
        __asm__ volatile ("fence" ::: "memory");
        ESPRADIO_PLIC_ENABLE_REG |=  (1u << ESPRADIO_GPIO_CPU_INT);
    }
}
