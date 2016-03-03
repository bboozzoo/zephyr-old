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
 * @brief Driver for UART port on STM32F10x family processor.
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
#include <clock_control/stm32_clock_control.h>
#include <pinmux/pinmux_stm32.h>
#ifdef CONFIG_SOC_STM32F1X
#include <pinmux_stm32f1.h>
#endif
#include <pinmux.h>
#include "uart_stm32.h"

struct pinconf {
	uint32_t pin;		/* STM32PIN() encoded pin */
	uint32_t func;		/* pin func */
};

struct uart_stm32_config {
	struct uart_device_config uconf;
	clock_control_subsys_t clock_subsys;
	struct pinconf pin_tx;
	struct pinconf pin_rx;
};

struct uart_stm32_data {
	uint32_t baud_rate;
	struct device *clock;
};

/* convenience defines */
#define DEV_CFG(dev)							\
	((struct uart_stm32_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)							\
	((struct uart_stm32_data * const)(dev)->driver_data)
#define UART_STRUCT(dev)					\
	((volatile struct uart_stm32 *)(DEV_CFG(dev))->uconf.base)

static struct uart_driver_api uart_stm32_driver_api;

/**
 * @brief set baud rate
 *
 */
static void set_baud_rate(struct device *dev, uint32_t rate)
{
	volatile struct uart_stm32 *uart = UART_STRUCT(dev);
	struct uart_stm32_data *data = DEV_DATA(dev);
	struct uart_stm32_config *cfg = DEV_CFG(dev);
	uint32_t div, mantissa, fraction;
	uint32_t clock;

	/* Baud rate is controlled through BRR register. The values
	 * written into the register depend on the clock driving the
	 * peripheral. Ask clock_control for the current clock rate of
	 * our peripheral.
	 */
	clock_control_get_rate(data->clock, cfg->clock_subsys, &clock);

	/* baud rate calculation:
	 *
	 *     baud rate = f_clk / (16 * usartdiv)
	 *
	 * Example (STM32F10x, USART1, PCLK2 @ 36MHz, 9600bps):
	 *
	 *    f_clk == PCLK2,
	 *    usartdiv = 234.375,
	 *    mantissa = 234,
	 *    fracion = 6 (0.375 * 16)
	 */

	div = clock / rate;
	mantissa = div >> 4;
	fraction = div & 0xf;

	uart->brr.bit.mantissa = mantissa;
	uart->brr.bit.fraction = fraction;
}

static int uart_stm32_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct uart_stm32 *uart = UART_STRUCT(dev);

	/* check if RXNE is set */
	if (!uart->sr.bit.rxne) {
		return -1;
	}

	/* read character */
	*c = (unsigned char)uart->dr.bit.dr;

	return 0;
}

static unsigned char uart_stm32_poll_out(struct device *dev,
					unsigned char c)
{
	volatile struct uart_stm32 *uart = UART_STRUCT(dev);

	/* wait for TXE to be set */
	while (!uart->sr.bit.txe) {
	}

	uart->dr.bit.dr = c;
	return c;
}

/**
 * @brief setup MCU pins
 *
 * FIXME: this should really take a port related parameter
 */
static void setup_port(struct uart_stm32_data *data,
		       struct uart_stm32_config *cfg)
{
	struct device *pindev = device_get_binding(STM32_PINMUX_NAME);

	pinmux_pin_set(pindev, cfg->pin_tx.pin,
		cfg->pin_tx.func);

	pinmux_pin_set(pindev, cfg->pin_rx.pin,
		cfg->pin_rx.func);
}

static inline void __uart_stm32_get_clock(struct device *dev)
{
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	if (clk) {
		struct uart_stm32_data *ddata = dev->driver_data;

		ddata->clock = clk;
	}
}
/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return DEV_OK
 */
static int uart_stm32_init(struct device *dev)
{
	volatile struct uart_stm32 *uart = UART_STRUCT(dev);

	__uart_stm32_get_clock(dev);

	struct uart_stm32_data *data = DEV_DATA(dev);
	struct uart_stm32_config *cfg = DEV_CFG(dev);

	/* enable clock */
	clock_control_on(data->clock, cfg->clock_subsys);

	/* setup pins */
	setup_port(data, cfg);

	/* FIXME: hardcoded, clear stop bits */
	uart->cr2.bit.stop = 0;

	uart->cr1.val = 0;
	/* FIXME: hardcoded, 8n1 */
	uart->cr1.bit.m = 0;
	uart->cr1.bit.pce = 0;

	/* FIXME: hardcoded, disable hardware flow control */
	uart->cr3.bit.ctse = 0;
	uart->cr3.bit.rtse = 0;

	set_baud_rate(dev, data->baud_rate);

	/* enable TX/RX */
	uart->cr1.bit.te = 1;
	uart->cr1.bit.re = 1;

	/* enable */
	uart->cr1.bit.ue = 1;

	dev->driver_api = &uart_stm32_driver_api;
	return DEV_OK;
}

static struct uart_driver_api uart_stm32_driver_api = {
	.poll_in = uart_stm32_poll_in,
	.poll_out = uart_stm32_poll_out,
};

#ifdef CONFIG_UART_STM32_PORT_0

static struct uart_stm32_config uart_stm32_dev_cfg_0 = {
	.uconf = {
		.base = (uint8_t *)USART1_ADDR,
	},
#ifdef CONFIG_SOC_STM32F1X
	.clock_subsys = UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_USART1),
	.pin_tx = {
		.pin = STM32_PIN_PA9,
		.func = STM32F1_PINMUX_FUNC_PA9_USART1_TX, /* USART1_TX */
	},
	.pin_rx = {
		.pin = STM32_PIN_PA10,
		.func = STM32F1_PINMUX_FUNC_PA10_USART1_RX, /* USART1_RX */
	},
#endif
};

static struct uart_stm32_data uart_stm32_dev_data_0 = {
	.baud_rate = CONFIG_UART_STM32_PORT_0_BAUD_RATE,
};

DEVICE_INIT(uart_stm32_0, CONFIG_UART_STM32_PORT_0_NAME, &uart_stm32_init,
	    &uart_stm32_dev_data_0, &uart_stm32_dev_cfg_0,
	    PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#endif	/* CONFIG_UART_STM32_PORT_0 */

#ifdef CONFIG_UART_STM32_PORT_1

static struct uart_stm32_config uart_stm32_dev_cfg_1 = {
	.uconf = {
		.base = (uint8_t *)USART2_ADDR,
	},
#ifdef CONFIG_SOC_STM32F1X
	.clock_subsys = UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_USART2),
	.pin_tx = {
		.pin = STM32_PIN_PA2,
		.func = STM32F1_PINMUX_FUNC_PA2_USART2_TX, /* USART2_TX */
	},
	.pin_rx = {
		.pin = STM32_PIN_PA3,
		.func = STM32F1_PINMUX_FUNC_PA3_USART2_RX, /* USART2_RX */
	},
#endif
};

static struct uart_stm32_data uart_stm32_dev_data_1 = {
	.baud_rate = CONFIG_UART_STM32_PORT_1_BAUD_RATE,
};

DEVICE_INIT(uart_stm32_1, CONFIG_UART_STM32_PORT_1_NAME, &uart_stm32_init,
	    &uart_stm32_dev_data_1, &uart_stm32_dev_cfg_1,
	    PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#endif	/* CONFIG_UART_STM32_PORT_1 */

#ifdef CONFIG_UART_STM32_PORT_2

static struct uart_stm32_config uart_stm32_dev_cfg_2 = {
	.uconf = {
		.base = (uint8_t *)USART3_ADDR,
	},
#ifdef CONFIG_SOC_STM32F1X
	.clock_subsys = UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_USART3),
	.pin_tx = {
		.pin = STM32_PIN_PB10,
		.func = STM32F1_PINMUX_FUNC_PB10_USART3_TX, /* USART3_TX */
	},
	.pin_rx = {
		.pin = STM32_PIN_PB11,
		.func = STM32F1_PINMUX_FUNC_PB11_USART3_RX, /* USART3_RX */
	},
#endif
};

static struct uart_stm32_data uart_stm32_dev_data_2 = {
	.baud_rate = CONFIG_UART_STM32_PORT_2_BAUD_RATE,
};

DEVICE_INIT(uart_stm32_2, CONFIG_UART_STM32_PORT_2_NAME, &uart_stm32_init,
	    &uart_stm32_dev_data_2, &uart_stm32_dev_cfg_2,
	    PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#endif	/* CONFIG_UART_STM32_PORT_2 */
