
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>


void delay(uint32_t n) {
    for (volatile int i = 0; i < n; i++)
        __asm__("nop");
}

int main(void) {

    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM5);

    //rcc_periph_reset_pulse(RST_TIM5);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                                                      GPIO_TIM5_CH1);
    gpio_set(GPIOA, GPIO0);

    timer_enable_preload(TIM5);
    //TIM_CR1(TIM5) |= TIM_CR1_ARPE;

    timer_enable_oc_preload(TIM5, TIM_OC1);
    //TIM_CCMR1(TIM5) |= TIM_CCMR1_OC1PE;

    timer_set_oc_mode(TIM5, TIM_OC1, TIM_OCM_PWM1);
    //TIM_CCMR1(TIM5) &= ~TIM_CCMR1_CC1S_MASK;
    //TIM_CCMR1(TIM5) |= TIM_CCMR1_CC1S_OUT;
    //TIM_CCMR1(TIM5) &= ~TIM_CCMR1_OC1M_MASK;
    //TIM_CCMR1(TIM5) |= TIM_CCMR1_OC1M_PWM1;

    timer_continuous_mode(TIM5);
    //TIM_CR1(TIM5) &= ~TIM_CR1_OPM;

    timer_set_counter(TIM5, 0);
    //TIM_CNT(TIM5) = 0;

    int prescale = (rcc_apb1_frequency / (1000 * 1000)) - 1;
    timer_set_prescaler(TIM5, prescale);
    //TIM_PSC(TIM5) = prescale;

    int period = (1000 * 1000 / 100) - 1;
    timer_set_period(TIM5, period);
    //TIM_ARR(TIM5) = period;

    int percent = 50;
    timer_set_oc_value(TIM5, TIM_OC1, period * percent / 100);
    //TIM_CCR1(TIM5) = period * percent / 100;

    timer_enable_oc_output(TIM5, TIM_OC1);
    //TIM_CCER(TIM5) |= TIM_CCER_CC1E;

    timer_enable_counter(TIM5);
    //TIM_CR1(TIM5) |= TIM_CR1_CEN;

    while (1) {
        for (int i = 200; i < 800; i += 1) {
            delay(1000000 * 0.02);
            timer_set_oc_value(TIM5, TIM_OC1, period * percent / 100)
            //TIM_CCR1(TIM5) = period * i/1000;
        }
        for (int i = 800; i > 200; i -= 1) {
            delay(1000000 * 0.02);
            timer_set_oc_value(TIM5, TIM_OC1, period * percent / 100)
            //TIM_CCR1(TIM5) = period * i/1000;
        }
    };

    return 0;
}
