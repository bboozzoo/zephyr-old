/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief Driver for Reset & Clock Control of STM32F10x family processor.
 *
 * Based on reference manual:
 *   STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx and STM32F107xx
 *   advanced ARM ® -based 32-bit MCUs
 *
 * Chapter 27: Universal synchronous asynchronous receiver
 *             transmitter (USART)
 */

#include <nanokernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <board.h>
#include <init.h>
#include <uart.h>
#include <clock_control.h>

#include <sections.h>
#include <clock_control/stm32f10x_clock_control.h>
#include <pinmux/pinmux_stm32f10x.h>
#include <pinmux.h>

/*  27.6.1 Status register (USART_SR) */
union __sr {
	uint32_t val;
	struct {
		uint32_t pe :1 __packed;
		uint32_t fe :1 __packed;
		uint32_t ne :1 __packed;
		uint32_t ore :1 __packed;
		uint32_t idle :1 __packed;
		uint32_t rxne :1 __packed;
		uint32_t tc :1 __packed;
		uint32_t txe :1 __packed;
		uint32_t lbd :1 __packed;
		uint32_t cts :1 __packed;
		uint32_t rsvd__10_15 : 6 __packed;
		uint32_t rsvd__16_31 : 16 __packed;
	} bit;
};

/* 27.6.2 Data register (USART_DR) */
union __dr {
	uint32_t val;
	struct {
		uint32_t dr :8 __packed;
		uint32_t rsvd__9_31 :24 __packed;
	} bit;
};

/* 27.6.3 Baud rate register (USART_BRR) */
union __brr {
	uint32_t val;
	struct {
		uint32_t fraction :4 __packed;
		uint32_t mantissa :12 __packed;
		uint32_t rsvd__16_31 :16 __packed;
	} bit;
};

/* 27.6.4 Control register 1 (USART_CR1) */
union __cr1 {
	uint32_t val;
	struct {
		uint32_t sbk :1 __packed;
		uint32_t rwu :1 __packed;
		uint32_t re :1 __packed;
		uint32_t te :1 __packed;
		uint32_t idleie :1 __packed;
		uint32_t rnxeie :1 __packed;
		uint32_t tcie :1 __packed;
		uint32_t txeie :1 __packed;
		uint32_t peie :1 __packed;
		uint32_t ps :1 __packed;
		uint32_t pce :1 __packed;
		uint32_t wake :1 __packed;
		uint32_t m :1 __packed;
		uint32_t ue :1 __packed;
		uint32_t rsvd__14_15 :2 __packed;
		uint32_t rsvd__16_31 :16 __packed;
	} bit;
};

/* 27.6.5 Control register 2 (USART_CR2) */
union __cr2 {
	uint32_t val;
	struct {
		uint32_t addr :4 __packed;
		uint32_t rsvd__4 :1 __packed;
		uint32_t lbdl :1 __packed;
		uint32_t lbdie :1 __packed;
		uint32_t rsvd__7 :1 __packed;
		uint32_t lbcl :1 __packed;
		uint32_t cpha :1 __packed;
		uint32_t cpol :1 __packed;
		uint32_t clken :1 __packed;
		uint32_t stop :2 __packed;
		uint32_t linen :1 __packed;
		uint32_t rsvd__15_31 :17 __packed;
	} bit;
};

/* 27.6.6 Control register 3 (USART_CR3) */
union __cr3 {
	uint32_t val;
	struct {
		uint32_t eie :1 __packed;
		uint32_t iren :1 __packed;
		uint32_t irlp :1 __packed;
		uint32_t hdsel :1 __packed;
		uint32_t nack :1 __packed;
		uint32_t scen :1 __packed;
		uint32_t dmar :1 __packed;
		uint32_t dmat :1 __packed;
		uint32_t rtse :1 __packed;
		uint32_t ctse :1 __packed;
		uint32_t ctsie :1 __packed;
		uint32_t rsvd__11_31 :21 __packed;
	} bit;
};

/* 27.6.7 Guard time and prescaler register (USART_GTPR) */
union __gtpr {
	uint32_t val;
	struct {
		uint32_t psc :8 __packed;
		uint32_t gt :8 __packed;
		uint32_t rsvd__16_31 :16 __packed;
	} bit;
};

/* 27.6.8 USART register map */
struct __uart {
	union __sr sr;
	union __dr dr;
	union __brr brr;
	union __cr1 cr1;
	union __cr2 cr2;
	union __cr3 cr3;
	union __gtpr gtpr;
};

struct uart_stm32f10x_config {
	struct uart_device_config uconf;
	uint32_t baud_rate;
	clock_control_subsys_t clock_subsys;

};

struct uart_stm32f10x_data {
	struct device *clock;
};

/* convenience defines */
#define DEV_CFG(dev)							\
	((struct uart_stm32f10x_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)							\
	((struct uart_stm32f10x_data * const)(dev)->driver_data)
#define UART_STRUCT(dev)					\
	((volatile struct __uart *)(DEV_CFG(dev))->uconf.base)

static struct uart_driver_api uart_stm32f10x_driver_api;

/**
 * @brief set baud rate
 *
 */
static void set_baud_rate(struct device *dev, uint32_t rate)
{
	volatile struct __uart *uart = UART_STRUCT(dev);
	struct uart_stm32f10x_data *data = DEV_DATA(dev);
	struct uart_stm32f10x_config *cfg = DEV_CFG(dev);

	/* Baud rate is controlled through BRR register. The values
	 * written into the register depend on the clock driving the
	 * peripheral, this can be either PCLK1 or PCLK2. Ask
	 * clock_control for the current clock rate of our
	 * peripheral. */
	uint32_t clock;
	stm32f10x_clock_control_get_subsys_rate(data->clock,
						cfg->clock_subsys,
						&clock);

	/* baud rate calculation:
	 *
	 *     baud rate = f_clk / (16 * usartdiv)
	 *
	 * Example (USART1, PCLK2 @ 36MHz, 9600bps):
	 *
	 *    f_clk == PCLK2,
	 *    usartdiv = 234.375,
	 *    mantissa = 234,
	 *    fracion = 6 (0.375 * 16)
	 */

	uint32_t div = clock / rate;
	uint32_t mantissa = div >> 4;
	uint32_t fraction = div & 0xf;

	uart->brr.bit.mantissa = mantissa;
	uart->brr.bit.fraction = fraction;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */

static int uart_stm32f10x_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct __uart *uart = UART_STRUCT(dev);

	/* check if RXNE is set */
	if (!uart->sr.bit.rxne)
		return -1;

	/* read character */
	*c = (unsigned char)uart->dr.bit.dr;

	return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_stm32f10x_poll_out(struct device *dev,
					unsigned char c)
{
	volatile struct __uart *uart = UART_STRUCT(dev);

	/* wait for TXE to be set */
	while (!uart->sr.bit.txe);

	uart->dr.bit.dr = c;
	return c;
}

/**
 * @brief setup MCU pins
 *
 * FIXME: this should really take a port related parameter
 */
static void setup_port(struct uart_stm32f10x_data *data) {
	struct device *pindev = device_get_binding(PINMUX_NAME);

	pinmux_pin_set(pindev, STM32PIN(STM32_PORTA, 9),
		PIN_CONFIG_AF_PUSH_PULL);

	pinmux_pin_set(pindev, STM32PIN(STM32_PORTA, 10),
		PIN_CONFIG_BIAS_HIGH_IMPEDANCE);
}

static inline void __uart_stm32f10x_get_clock(struct device *dev)
{
	struct device *clk =
		device_get_binding(STM32F10X_CLOCK_CONTROL_NAME);

	if (clk) {
		struct uart_stm32f10x_data *ddata = dev->driver_data;
		ddata->clock = clk;
	}
}
/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 * NOTE: hardcided to setup USART1 only
 *
 * @param dev UART device struct
 *
 * @return DEV_OK
 */
static int uart_stm32f10x_init(struct device *dev)
{
	volatile struct __uart *uart = UART_STRUCT(dev);

        __uart_stm32f10x_get_clock(dev);

        struct uart_stm32f10x_data *data = DEV_DATA(dev);
	struct uart_stm32f10x_config *cfg = DEV_CFG(dev);

	/* enable clock */
        clock_control_on(data->clock, cfg->clock_subsys);

	/* setup pins */
	setup_port(data);

	/* FIXME: hardcoded, clear stop bits */
	uart->cr2.bit.stop = 0;

	uart->cr1.val = 0;
	/* FIXME: hardcoded, 8n1 */
	uart->cr1.bit.m = 0;
	uart->cr1.bit.pce = 0;

	/* FIXME: hardcoded, disable hardware flow control */
	uart->cr3.bit.ctse = 0;
	uart->cr3.bit.rtse = 0;

	set_baud_rate(dev, cfg->baud_rate);

	/* enable TX/RX */
	uart->cr1.bit.te = 1;
	uart->cr1.bit.re = 1;

	/* enable */
	uart->cr1.bit.ue = 1;

	dev->driver_api = &uart_stm32f10x_driver_api;
	return DEV_OK;
}

static struct uart_driver_api uart_stm32f10x_driver_api = {
	.poll_in = uart_stm32f10x_poll_in,
	.poll_out = uart_stm32f10x_poll_out,
};

#define UART_ADDR USART1_ADDR
static struct uart_stm32f10x_config uart_stm32f10x_dev_cfg_0 = {
	.uconf = {
		.base = (uint8_t *)UART_ADDR,
		.sys_clk_freq = CONFIG_UART_STM32F10X_CLK_FREQ,
	},
	.baud_rate = CONFIG_UART_STM32F10X_BAUD_RATE,
        .clock_subsys = UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_USART1),
};

static struct uart_stm32f10x_data uart_stm32f10x_dev_data_0;

DEVICE_INIT(uart_stm32f10x_0, CONFIG_UART_STM32F10X_NAME, &uart_stm32f10x_init,
	    &uart_stm32f10x_dev_data_0, &uart_stm32f10x_dev_cfg_0,
	    PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
