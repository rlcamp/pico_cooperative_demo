/* simple demo which uses sevonpend and runs a few cooperative tasks that can each behave
 as if they were the only thing running, simply by replacing __wfe() with yield() */
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

#include "cortex_m_cooperative_multitasking.h"

#include <stdio.h>
#include <string.h>

static unsigned wakes = 0;

/* override the default version of this function to also increment a counter */
void sleep_until_event(void) {
    __dsb();
    __wfe();
    wakes++;
}

/* begin cooperative multitasking + low power sleep friendly versions of sdk funcs */

void uart_puts_with_yield(uart_inst_t * uart, const char * buf) {
    /* block in low power yield until all bytes have been copied into the tx fifo */
    /* enable the tx non-full interrupt, but leave disabled in nvic (expects sevonpend) */
    hw_set_bits(&uart_get_hw(uart)->imsc, 1U << UART_UARTIMSC_TXIM_LSB);
    irq_set_enabled(UART_IRQ_NUM(uart), false);

    /* set fifo threshold to raise interrupt when fifo is non full */
    hw_write_masked(&uart_get_hw(uart)->ifls, 0 << UART_UARTIFLS_TXIFLSEL_LSB, 0b100);

    /* loop over outgoing bytes, adding each to the fifo as soon as there is room for it */
    for (const char * cursor = buf; *cursor != '\0'; cursor++) {
        /* while uart fifo is full, yield until the next interrupt becomes pending */
        while (uart_get_hw(uart)->fr & UART_UARTFR_TXFF_BITS)
            yield();

        /* clear the interrupt that woke us up */
        irq_clear(UART_IRQ_NUM(uart));

        /* write the byte to the fifo */
        uart_get_hw(uart)->dr = *cursor;
    }

    /* disable tx non-full interrupt */
    hw_clear_bits(&uart_get_hw(uart)->imsc, 1U << UART_UARTIMSC_TXIM_LSB);
}

void uart_tx_wait_blocking_with_yield(uart_inst_t * uart) {
    /* block in busy yield until all bytes in the tx fifo have been transmitted */
    while (uart_get_hw(uart)->fr & UART_UARTFR_BUSY_BITS) {
        /* since we are not waiting for an interrupt-accompanied condition, we must
         inhibit the wfe call within yield, but can still allow yield to do other things */
        __sev();
        yield();
    }
}

static volatile unsigned char uart_tx_locked = 0;

static void uart_tx_lock(void) {
    while (uart_tx_locked) yield();
    uart_tx_locked = 1;
}

static void uart_tx_unlock(void) {
    uart_tx_locked = 0;
    __sev();
}

/* end cooperative multitasking + low power sleep friendly versions of sdk funcs */

static void pwm_task(void) {
    const unsigned period_in_12MHz_ticks = 30000;

    gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);
    const unsigned slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);

    pwm_clear_irq(slice_num);

    /* enable interrupt for pwm wrap, but leave it disabled in nvic */
    pwm_set_irq_enabled(slice_num, true);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), false);

    /* set up the pwm to tick at 12 MHz (assuming sys is 48 MHz) and overflow at 400 Hz */
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 4);
    pwm_config_set_wrap(&config, period_in_12MHz_ticks - 1);
    pwm_init(slice_num, &config, true);

    unsigned phase = 0;

    /* loop once per pwm overflow */
    while (1) {
        /* run other tasks or low power sleep until next pwm overflow interrupt */
        while (!(pwm_hw->intr & 1U << slice_num))
            yield();

        /* acknowledge and clear the interrupt in both pwm and nvic */
        pwm_clear_irq(slice_num);
        irq_clear(PWM_DEFAULT_IRQ_NUM());

        /* make a triangle wave from 0 to 173 and back */
        const unsigned linear = phase >= 173 ? 346 - phase : phase;
        phase = (phase + 1) % 346;

        /* square the triangle wave to generate values between 0 and 30000 */
        pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, linear * linear);
    }
}

static void uart_rx_task(void) {
    /* wake up when either fifo reaches 1/8 full, or when fifo is nonempty and a timeout elapses */
    hw_set_bits(&uart_get_hw(uart0)->imsc, 1U << UART_UARTIMSC_RXIM_LSB | 1U << UART_UARTIMSC_RTIM_LSB);
    irq_set_enabled(UART_IRQ_NUM(uart0), false);
    hw_write_masked(&uart_get_hw(uart0)->ifls, 0 << UART_UARTIFLS_RXIFLSEL_LSB, 0);

    size_t icur = 0;
    char linebuf[82];

    /* loop on characters from uart */
    while (1) {
        /* sleep until either fifo is 1/8 full, or fifo is nonempty and times out */
        while (uart_get_hw(uart0)->fr & UART_UARTFR_RXFE_BITS)
            yield();

        /* clear both of the above possible reasons for having awoke */
        uart_get_hw(uart0)->icr = UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS;
        irq_clear(UART_IRQ_NUM(uart0));

        /* either of the two reasons mean we can can read at least one byte */
        const unsigned char byte = uart_get_hw(uart0)->dr;

        /* only advance the cursor if not already full */
        if (icur < sizeof(linebuf)) linebuf[icur++] = byte;

        /* this will only be non-NULL when it points to a complete, properly terminated line */
        const char * line = NULL;

        /* we treat either line ending identically, and ignore duplicates/empty lines */
        if ('\r' == byte || '\n' == byte) {
            /* if overflow occurred, discard the partial line */
            if (byte != linebuf[icur - 1]) icur = 1;

            /* overwrite whichever line ending we got with a zero termination */
            linebuf[icur - 1] = '\0';

            /* reset cursor */
            icur = 0;

            /* if the line ending in this newline was nonempty, return it */
            if (linebuf[0] != '\0') line = linebuf;
        }

        /* if we got a complete line... */
        if (line) {
            static char buf[128];
            snprintf(buf, sizeof(buf), "%% %s\r\n", line);

            uart_tx_lock();
            uart_puts_with_yield(uart0, buf);
            uart_tx_unlock();
        }
    }
}

int main(void) {
    /* this is not a computationally heavy demo - we could run at 12 MHz but it
     requires more code and for this demo we just want to run at a defined rate */
    set_sys_clock_48mhz();

    /* enable sevonpend, so that we don't need nearly-empty ISRs and can wake
     directly into the waiting application code */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    /* enable uart */
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    /* use a really slow baud rate to illustrate what is happening */
    uart_init(uart0, 300);
    uart_puts_with_yield(uart0, "\r\nhello\r\n");
    uart_tx_wait_blocking_with_yield(uart0);

    static struct __attribute((aligned(8))) {
        /* this needs to be enough to accommodate the deepest call stack needed
         by a child task, PLUS any interrupt handlers if we are not using the
         msp/psp switch to provide interrupt handlers with their own dedicated
         call stack. this is probably still overkill */
        unsigned char stack[2048 - 16];

        struct child_context child;
    } child_pwm, child_uart_rx;

    child_start(&child_pwm.child, pwm_task);
    child_start(&child_uart_rx.child, uart_rx_task);

    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);

    /* enable interrupt for alarm, but leave it disabled in nvic */
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    const unsigned interval = 5000000;

    /* first tick will be 1/4 interval from now */
    timer_hw->alarm[alarm_num] = timer_hw->timerawl + interval / 4;

    unsigned wakes_prior = wakes;

    while (1) {
        /* run other tasks or low power sleep until next alarm interrupt */
        while (!(timer_hw->intr & (1U << alarm_num)))
            yield();

        /* acknowledge and clear the interrupt in both timer and nvic */
        hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
        irq_clear(hardware_alarm_get_irq_num(alarm_num));

        /* increment and rearm the alarm */
        timer_hw->alarm[alarm_num] += interval;

        const unsigned wakes_now = *(volatile unsigned *)&wakes;
        const unsigned wakes_elapsed = wakes_now - wakes_prior;
        wakes_prior = wakes_now;

        static char buf[128];
        snprintf(buf, sizeof(buf),
                 "this is how many wakes there have been since last alarm: %u\r\n", wakes_elapsed);

        uart_tx_lock();
        uart_puts_with_yield(uart0, buf);

        /* this will busy loop (yield, with wfe inhibited) while the last up to 32 bytes of
         the above are transmitted. you may or may not actually need this, it is safe to
         modify buf without doing so */
//        uart_tx_wait_blocking_with_yield(uart0);
        uart_tx_unlock();
    }
}
