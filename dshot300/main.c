/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>


static void timer_gpio_setup(uint32_t gpio_group, uint32_t gpio_af, uint32_t gpio_pin) {
    gpio_mode_setup(gpio_group, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_pin);
    gpio_set_af(gpio_group, gpio_af, gpio_pin);
    gpio_set_output_options(gpio_group, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpio_pin);
}

static void timer_channel_disable(uint32_t timer, uint32_t channel) {
    timer_disable_oc_output(timer, channel);
    switch(channel) {
        case TIM_OC1:
            timer_disable_irq(timer, TIM_DIER_CC1DE);
            break;
        case TIM_OC2:
            timer_disable_irq(timer, TIM_DIER_CC2DE);
            break;
        case TIM_OC3:
            timer_disable_irq(timer, TIM_DIER_CC3DE);
            break;
        case TIM_OC4:
            timer_disable_irq(timer, TIM_DIER_CC4DE);
            break;
    }
}

static void timer_channel_enable(uint32_t timer, uint32_t channel) {
    timer_enable_oc_output(timer, channel);
    switch(channel) {
        case TIM_OC1:
            timer_enable_irq(timer, TIM_DIER_CC1DE);
            break;
        case TIM_OC2:
            timer_enable_irq(timer, TIM_DIER_CC2DE);
            break;
        case TIM_OC3:
            timer_enable_irq(timer, TIM_DIER_CC3DE);
            break;
        case TIM_OC4:
            timer_enable_irq(timer, TIM_DIER_CC4DE);
            break;
    };
}

static void timer_channel_init(uint32_t timer, uint32_t channel) {
    timer_disable_oc_output(timer, channel);
    timer_set_oc_value(timer, channel, 0);
    timer_disable_oc_clear(timer, channel);
    timer_disable_oc_preload(timer, channel);
    timer_set_oc_slow_mode(timer, channel);
    timer_set_oc_mode(timer, channel, TIM_OCM_PWM1);

    timer_set_oc_polarity_high(timer, channel);
    timer_set_oc_idle_state_set(timer, channel);
    timer_enable_oc_output(timer, channel);
}

#define DSHOT600 472
#define DSHOT300 236


static void timer_init(uint32_t timer, uint32_t bitrate) {

    int prescale = 0;
    int period = 65536 / bitrate - 1;

    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_disable_preload(timer);
    timer_continuous_mode(timer);
    timer_set_prescaler(timer, prescale);
    timer_set_period(timer, period);
    timer_set_repetition_counter(timer, 0);
    timer_enable_break_main_output(timer);

    timer_enable_counter(timer);
}

void timer_setup(uint32_t timer, uint32_t bitrate) {
    int period = 65536 / bitrate - 1;
    timer_set_period(timer, period);
}


static void timer_disable(uint32_t timer) {
    timer_disable_counter(timer);
}

static void timer_enable(uint32_t timer) {
    timer_enable_counter(timer);
}

static uint32_t timer_get_period(uint32_t timer) {
    return TIM_ARR(timer);
}


static void dma_init(uint32_t dma, uint32_t stream, uint32_t channel) {
    dma_stream_reset(dma, stream);
    dma_set_priority(dma, stream, DMA_SxCR_PL_HIGH);

    dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_32BIT);
    dma_set_peripheral_size(dma, stream, DMA_SxCR_PSIZE_16BIT);
    dma_enable_memory_increment_mode(dma, stream);
    dma_disable_peripheral_increment_mode(dma, stream);
    dma_enable_circular_mode(dma, stream);
    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

    dma_channel_select(dma, stream, channel);
}

static void dma_setup(uint32_t dma, uint32_t stream, uint32_t* src, uint32_t size, uint32_t timer) {
    dma_set_peripheral_address(dma, stream, (uint32_t)&(TIM_CCR1(timer)));
    dma_set_memory_address(dma, stream, (uint32_t)src);
    dma_set_number_of_data(dma, stream, size);
}

static void dma_disable(uint32_t dma, uint32_t stream) {
    dma_disable_stream(dma, stream);
}

static void dma_enable(uint32_t dma, uint32_t stream) {
    dma_enable_stream(dma, stream);
}


void delay(uint32_t n) {
    for (volatile int i = 0; i < n; i++)
        __asm__("nop");
}

#define BUFFER_SIZE 30

void dshot_encode(uint16_t command, uint32_t* buffer, uint32_t period) {
    uint32_t dshot0 = period * 39 / 100;
    uint32_t dshot1 = period * 71 / 100;

    uint16_t payload = command << 1;
    uint8_t crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F;

    for (int i = 0; i < 16; i++) buffer[i] = dshot1;

    int offset = 0;
    for (int i = 0; i < 11; i++) {
        if ((command & ((uint16_t)1 << (10 - i))) > 0) {
            buffer[i] = dshot1;
        } else {
            buffer[i] = dshot0;
        }
    }
    buffer[11] = dshot0;

    offset = 12;
    for (int i = 0; i < 4; i++) {
        if ((crc & (((uint16_t)1) << (3 - i))) > 0) {
            buffer[i + offset] = dshot1;
        } else {
            buffer[i + offset] = dshot0;
        }
    }
    offset = 16;
    for (int i = 0; i < BUFFER_SIZE - offset; i++) {
        buffer[i + offset] = 0x00;
    }
}


int main(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_TIM4);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_GPIOB);

    uint32_t timer = TIM4;
    uint32_t timer_channel = TIM_OC1;

    uint32_t dma = DMA1;
    uint32_t dma_stream = DMA_STREAM0;
    uint32_t dma_channel = DMA_SxCR_CHSEL_2;



    uint32_t buffer[BUFFER_SIZE];
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer[i] = 0;
    }


    dma_init(dma, dma_stream, dma_channel);
    dma_disable(dma, dma_stream);
    dma_setup(dma, dma_stream, buffer, BUFFER_SIZE * 2, timer);
    dma_enable(dma, dma_stream);

    timer_disable(timer);
    timer_init(timer, DSHOT600);

    uint32_t gpio_group = GPIOB;
    uint32_t gpio_af = GPIO_AF2;
    uint32_t gpio_pin = GPIO6;

    timer_channel_disable(timer, timer_channel);
    timer_gpio_setup(gpio_group, gpio_af, gpio_pin);

    timer_channel_init(timer, timer_channel);
    timer_channel_enable(timer, timer_channel);
    timer_enable(timer);


    uint32_t period = timer_get_period(timer);

    delay(1 * 1000 * 1000);

    uint16_t command    = 0;
    dshot_encode(command, buffer, period);

    delay(120 * 1000 * 1000);

    while (1) {
        for (uint16_t i = 48; i < 2048 / 6; i++) {
            delay(100 * 1000);
            command = i;
            dshot_encode(command, buffer, period);
        }

        for (uint16_t i = 2047 / 6; i > 48; i--) {
            delay(100 * 1000);
            command    = i;
            dshot_encode(command, buffer, period);
        }
    };

    return 0;
}
