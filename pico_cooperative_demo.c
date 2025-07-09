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
#include <ctype.h>

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

/* end cooperative multitasking + low power sleep friendly versions of sdk funcs */

/* this sort of lock is only needed if multiple tasks want to interact with a shared
 resource (such as a uart) using functions that themselves internally call yield() */

static volatile unsigned char uart_tx_locked = 0;

static void uart_tx_lock(void) {
    while (uart_tx_locked) yield();
    uart_tx_locked = 1;
}

static void uart_tx_unlock(void) {
    uart_tx_locked = 0;
    __sev();
}

static void pwm_led_task(void) {
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

static const uint32_t table[] = {
    ['A'] = 0b10111000,
    ['B'] = 0b111010101000,
    ['C'] = 0b11101011101000,
    ['D'] = 0b1110101000,
    ['E'] = 0b1000,
    ['F'] = 0b101011101000,
    ['G'] = 0b111011101000,
    ['H'] = 0b1010101000,
    ['I'] = 0b101000,
    ['J'] = 0b1110111011101000,
    ['K'] = 0b111010111000,
    ['L'] = 0b101110101000,
    ['M'] = 0b1110111000,
    ['N'] = 0b11101000,
    ['O'] = 0b11101110111000,
    ['P'] = 0b10111011101000,
    ['Q'] = 0b1110111010111000,
    ['R'] = 0b1011101000,
    ['S'] = 0b10101000,
    ['T'] = 0b111000,
    ['U'] = 0b1010111000,
    ['V'] = 0b101010111000,
    ['W'] = 0b101110111000,
    ['X'] = 0b11101010111000,
    ['Y'] = 0b1110101110111000,
    ['Z'] = 0b11101110101000,
    ['1'] = 0b10111011101110111000,
    ['2'] = 0b101011101110111000,
    ['3'] = 0b1010101110111000,
    ['4'] = 0b10101010111000,
    ['5'] = 0b101010101000,
    ['6'] = 0b11101010101000,
    ['7'] = 0b1110111010101000,
    ['8'] = 0b111011101110101000,
    ['9'] = 0b11101110111011101000,
    ['0'] = 0b1110111011101110111000,
    ['+'] = 0b1011101011101000,
    ['-'] = 0b111010101010111000,
    ['?'] = 0b101011101110101000,
    ['/'] = 0b1110101011101000,
    ['.'] = 0b10111010111010111000,
    [','] = 0b1110111010101110111000,
    ['\''] = 0b1110101011101000,
    [')'] = 0b1110101110111010111000,
    ['('] = 0b111010111011101000,
    [':'] = 0b11101110111010101000,
};

static void pwm_piezo_morse_task(void) {
    /* generate quiet linear-period-modulated chirps from 2818 to 3548 Hz with piezo */
    gpio_set_function(28, GPIO_FUNC_PWM);
    const unsigned slice_num = pwm_gpio_to_slice_num(28);

    /* tone period in 12 MHz ticks */
    const unsigned waveform_period = 4258;

    /* get another timer, enable interrupt for alarm, but leave it disabled in nvic */
    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    const unsigned dot_length_microseconds = 80000;

    /* first tick will be one interval from now */
    timer_hw->alarm[alarm_num] = timer_hw->timerawl + dot_length_microseconds;

    /* set up the pwm to tick at 12 MHz (assuming sys is 48 MHz) */
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 4);
    pwm_config_set_wrap(&config, waveform_period - 1);
    pwm_init(slice_num, &config, true);

    static const char * sentence = "everything is fine. nothing is ruined. ";

    while (1) {
        /* loop over letters within the sentence */
        for (const char * letter = sentence; *letter != '\0'; letter++) {
            const unsigned index = toupper(*letter);
            const unsigned bits = index < sizeof(table) / sizeof(table[0]) ? table[index] : 0;
            unsigned ibit = bits ? 32 - __builtin_clz(bits) : 6;

            /* loop over pixels within the letter */
            for (; ibit; ibit--) {
                /* change duty cycle to either quiet-ish or off */
                pwm_set_gpio_level(28, (bits >> ibit) & 1 ? 30 : 0);

                /* run other tasks or low power sleep until next alarm interrupt */
                while (!(timer_hw->intr & (1U << alarm_num)))
                    yield();

                /* acknowledge and clear the interrupt in both timer and nvic */
                hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
                irq_clear(hardware_alarm_get_irq_num(alarm_num));

                /* increment and rearm the alarm */
                timer_hw->alarm[alarm_num] += dot_length_microseconds;
            }
        }
    }
}

static void pwm_chirp_task(void) {
    /* generate quiet linear-period-modulated chirps from 2818 to 3548 Hz with piezo */
    gpio_set_function(28, GPIO_FUNC_PWM);
    const unsigned slice_num = pwm_gpio_to_slice_num(28);

    /* these are all in 12 MHz ticks */
    const unsigned period_initial = 4258;
    const unsigned period_final = 3382;
    const unsigned decrement = 1;

    pwm_clear_irq(slice_num);

    /* enable interrupt for pwm wrap, but leave it disabled in nvic */
    pwm_set_irq_enabled(slice_num, true);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), false);

    /* set up the pwm to tick at 12 MHz (assuming sys is 48 MHz) */
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 4);
    pwm_config_set_wrap(&config, period_initial - 1);
    pwm_init(slice_num, &config, true);

    /* the piezo is very loud if we set this to period/2 */
    pwm_set_gpio_level(28, 5);

    while (1)
        for (unsigned period = period_initial; period > period_final; period -= decrement) {
            /* change the pwm period for the next cycle on every cycle */
            pwm_hw->slice[slice_num].top = period - decrement - 1;

            /* run other tasks or low power sleep until next pwm overflow interrupt */
            while (!(pwm_hw->intr & 1U << slice_num))
                yield();

            /* acknowledge and clear the interrupt in both pwm and nvic */
            pwm_clear_irq(slice_num);
            irq_clear(PWM_DEFAULT_IRQ_NUM());
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

static void button_task(void) {
    unsigned gpio = 2;

    gpio_init(gpio);
    gpio_pull_up(gpio);
    gpio_set_input_hysteresis_enabled(gpio, true);

    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, false);
    gpio_acknowledge_irq(gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
    while (1) {
        unsigned events;
        while (1) {
            events = ((io_bank0_hw->intr[gpio >> 3U]) >> (4U * gpio)) & 0xF;
            if (events & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)) {
                gpio_acknowledge_irq(gpio, events);
                irq_clear(IO_IRQ_BANK0);
                break;
            }

            yield();
        }

        uart_tx_lock();
        uart_puts_with_yield(uart0, events & GPIO_IRQ_EDGE_FALL ? "falling edge\r\n" : "rising edge\r\n");
        uart_tx_unlock();
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
    } child_pwm_led, child_uart_rx, child_pwm_chirp, child_button;

    child_start(&child_pwm_led.child, pwm_led_task);
    child_start(&child_pwm_chirp.child, pwm_piezo_morse_task);
    child_start(&child_uart_rx.child, uart_rx_task);
    child_start(&child_button.child, button_task);

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
