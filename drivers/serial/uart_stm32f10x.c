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
 * @brief Driver for UART on STM32F10x family processor.
 *
 * (used uart_atmel_sam3.c as template)
 */
#include <nanokernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <board.h>
#include <init.h>
#include <uart.h>
#include <sections.h>
#include <rcc.h>
#include <gpio.h>

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

union __dr {
	uint32_t val;
	struct {
		uint32_t dr :8 __packed;
		uint32_t rsvd__9_31 :24 __packed;
	} bit;
};

union __brr {
	uint32_t val;
	struct {
		uint32_t fraction :4 __packed;
		uint32_t mantissa :12 __packed;
		uint32_t rsvd__16_31 :16 __packed;
	} bit;
};

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

union __gtpr {
	uint32_t val;
	struct {
		uint32_t psc :8 __packed;
		uint32_t gt :8 __packed;
		uint32_t rsvd__16_31 :16 __packed;
	} bit;
};

struct __uart {
	union __sr sr;
	union __dr dr;
	union __brr brr;
	union __cr1 cr1;
	union __cr2 cr2;
	union __cr3 cr3;
	union __gtpr gtpr;
};

struct uart_stm32f10x_dev_data {
	uint32_t baud_rate;
};

/* convenience defines */
#define DEV_CFG(dev) \
	((struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_stm32f10x_data_t * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile struct __uart *)(DEV_CFG(dev))->base)

static struct uart_driver_api uart_stm32f10x_driver_api;

/**
 * @brief set baud rate
 *
 */
static void set_baud_rate(struct device *dev, uint32_t rate)
{
	volatile struct __uart *uart = UART_STRUCT(dev);

	/* get clock frequency */
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
static void setup_port(void) {
	volatile struct stm32f10x_gpio *gpio = (struct stm32f10x_gpio *)(GPIOA_BASE);

	/* pin 9, output, alternate function push-pull */
	gpio->crh &= ~(0xf << 4);
	gpio->crh |= (0x2 << 6) | (0x1 << 4);


	/* pin 10, input */
	gpio->crh &= ~(0xf << 8);
	gpio->crh |= (0x1 << 10);
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

	/* TODO: find out which UART port we're using */

	/* FIXME: hardcoded USART1 */

	/* enable clock */
	volatile struct stm32f10x_rcc *rcc = (struct stm32f10x_rcc*)(RCC_BASE);
	/* USART1, APB2 clock */
	rcc->apb2enr |= 1 << 14;

	/* GPIO port A, APB2 clock */
	rcc->apb2enr |= 1 << 2;

	/* AF, APB2 clock */
	rcc->apb2enr |= 1 << 0;

	/* setup pins */
	setup_port();

	/* clear stop bits */
	uart->cr2.bit.stop = 0;

	uart->cr1.val = 0;
	/* 8n1 */
	uart->cr1.bit.m = 0;
	uart->cr1.bit.pce = 0;

	/* disable hardware flow control */
	uart->cr3.bit.ctse = 0;
	uart->cr3.bit.rtse = 0;

	/* setup 9600 */
	/* TODO: setup baud rate */

	/* assuming PCLK2 (USART1) is clocked at 36MHz */

	/* baud rate calculation:
	 *
	 *     baud rate = f_clk / (16 * usartdiv)
	 *
	 * for USART1, f_clk == PCLK2, for 9600, usartdiv = 234.375,
	 * hence mantissa = 234, fracion = 6 (0.375 * 16)
	 */

	uart->brr.bit.mantissa = 234;
	uart->brr.bit.fraction = 6;

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
static struct uart_device_config uart_stm32f10x_dev_cfg_0 = {
	.base = (uint8_t *)UART_ADDR,
	.sys_clk_freq = CONFIG_UART_STM32F10X_CLK_FREQ,
};

static struct uart_stm32f10x_dev_data uart_stm32f10x_dev_data_0 = {
	.baud_rate = CONFIG_UART_STM32F10X_BAUD_RATE,
};

DEVICE_INIT(uart_stm32f10x_0, CONFIG_UART_STM32F10X_NAME, &uart_stm32f10x_init,
            &uart_stm32f10x_dev_data_0, &uart_stm32f10x_dev_cfg_0,
            PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
