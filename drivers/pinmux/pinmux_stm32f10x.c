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
 * @brief
 *
 * Based on reference manual:
 *   STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx and STM32F107xx
 *   advanced ARM ® -based 32-bit MCUs
 *
 * Chapter 9: General-purpose and alternate-function I/Os
 *            (GPIOs and AFIOs)
 */

#include <nanokernel.h>
#include <device.h>
#include <soc.h>
#include <soc_registers.h>
#include <gpio.h>
#include "pinmux.h"
#include <pinmux.h>
#include <pinmux/pinmux_stm32f10x.h>
#include <clock_control/stm32_clock_control.h>
#include <misc/util.h>

/**
 * @brief helper to extract IO port number from STM32PIN() encoded
 * value
 */
#define PORT(__pin) \
	(__pin >> 4)

/**
 * @brief helper to extract IO pin number from STM32PIN() encoded
 * value
 */
#define PIN(__pin) \
	(__pin & 0xf)

uint32_t __func_to_mode(int func)
{
	switch (func) {
	case PIN_CONFIG_ANALOG:
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
		return 0;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_AF_PUSH_PULL:
	case PIN_CONFIG_AF_OPEN_DRAIN:
		return 0x1;
	}
	return 0;
}

uint32_t __func_to_cnf(int func)
{
	switch (func) {
	case PIN_CONFIG_ANALOG:
		return 0x0;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		return 0x1;
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
		return 0x2;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		return 0x0;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		return 0x1;
	case PIN_CONFIG_AF_PUSH_PULL:
		return 0x2;
	case PIN_CONFIG_AF_OPEN_DRAIN:
		return 0x3;
	}
	return 0;
}

/**
 * @brief enable IO port clock
 */
static uint32_t enable_port(uint32_t port)
{
	/* enable port clock */
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	const clock_control_subsys_t ports_to_clock[STM32_PORTS_MAX] = {
		UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPA),
		UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPB),
		UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPC),
		UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPD),
		UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPE),
	};

	if (port >= STM32_PORTS_MAX) {
		return DEV_FAIL;
	}

	return clock_control_on(clk, ports_to_clock[port]);
}

/* Each GPIO port controls 16 pins. Each pin can be configured in
 * one of the following modes:
 * - Input floating
 * - Input pull-up
 * - Input-pull-down
 * - Analog
 * - Output open-drain
 * - Output push-pull
 * - Alternate function push-pull
 * - Alternate function open-drain
 */
static uint32_t pinmux_stm32f10x_set(struct device *dev,
				     uint32_t pin, uint8_t func)
{
	uint32_t cnf, mode;

	/* make sure to enable port clock first */
	if (enable_port(PORT(pin)) != DEV_OK) {
		return DEV_FAIL;
	}

	/* determine IO port registers location */
	uint32_t offset = PORT(pin) * GPIO_REG_SIZE;
	uint8_t *port_base = (uint8_t *)(GPIO_PORTS_BASE + offset);
	volatile struct stm32f10x_gpio *gpio =
		(struct stm32f10x_gpio *)(port_base);

	/* determine port pin number */
	int ppin = PIN(pin);
	/* pins are configured in CRL (0-7) and CRH (8-15)
	 * registers
	 */
	volatile uint32_t *reg = &gpio->crl;

	if (ppin > 7) {
		reg = &gpio->crh;
		ppin -= 8;
	}

	/* each port is configured by 2 registers:
	 * CNFy[1:0]: Port x configuration bits
	 * MODEy[1:0]: Port x mode bits
	 *
	 * memory layout is repeated for every port:
	 *   |  CNF  |  MODE |
	 *   | [0:1] | [0:1] |
	 */
	cnf = __func_to_cnf(func);
	mode = __func_to_mode(func);

	/* clear bits */
	*reg &= ~(0xf << (ppin * 4));
	/* set bits */
	*reg |= (cnf << (ppin * 4 + 2) | mode << (ppin * 4));

	return DEV_OK;
}

static uint32_t pinmux_stm32f10x_get(struct device *dev,
				     uint32_t pin, uint8_t *func)
{
	return DEV_NO_SUPPORT;
}

static struct pinmux_driver_api pinmux_stm32f10x_api = {
	.set = pinmux_stm32f10x_set,
	.get = pinmux_stm32f10x_get,
};

int pinmux_stm32f10x_init(struct device *port)
{
	port->driver_api = &pinmux_stm32f10x_api;
	return DEV_OK;
}

static struct pinmux_config pinmux_stm32f10x_cfg = {
	.base_address = GPIO_PORTS_BASE,
};

/**
 * @brief device init
 *
 * Device priority set to 2, so that we come after clock_control and
 * before any other devices
 */
DEVICE_INIT(pinmux_stm32f10x, STM32_PINMUX_NAME, &pinmux_stm32f10x_init,
	    NULL, &pinmux_stm32f10x_cfg,
	    PRIMARY, CONFIG_PINMUX_STM32F10X_DEVICE_INITIALIZATION_PRIORITY);
