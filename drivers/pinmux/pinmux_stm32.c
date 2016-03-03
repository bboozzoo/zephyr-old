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
 * A common driver for STM32 pinmux. Each SoC must implement a SoC
 * specific part of the driver.
 */

#include <nanokernel.h>
#include <device.h>
#include <soc.h>
#include "pinmux.h"
#include <pinmux.h>
#include <pinmux/pinmux_stm32.h>
#include <clock_control/stm32_clock_control.h>

/**
 * @brief enable IO port clock
 */
static uint32_t enable_port(uint32_t port)
{
	clock_control_subsys_t subsys = stm32_get_port_clock(port);
	/* enable port clock */
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	return clock_control_on(clk, subsys);
}

static uint32_t pinmux_stm32_set(struct device *dev,
				 uint32_t pin, uint8_t func)
{
	int config;

	/* make sure to enable port clock first */
	if (enable_port(STM32_PORT(pin)) != DEV_OK) {
		return DEV_FAIL;
	}

	/* determine config for alternat function */
	config = stm32_get_pin_config(pin, func);

	return stm32_pin_configure(pin, config);
}

static uint32_t pinmux_stm32_get(struct device *dev,
				 uint32_t pin, uint8_t *func)
{
	return DEV_NO_SUPPORT;
}

static struct pinmux_driver_api pinmux_stm32_api = {
	.set = pinmux_stm32_set,
	.get = pinmux_stm32_get,
};

int pinmux_stm32_init(struct device *port)
{
	port->driver_api = &pinmux_stm32_api;
	return DEV_OK;
}

static struct pinmux_config pinmux_stm32_cfg = {
#ifdef CONFIG_SOC_STM32F1X
	.base_address = GPIO_PORTS_BASE,
#endif
};

/**
 * @brief device init
 *
 * Device priority set to 2, so that we come after clock_control and
 * before any other devices
 */
DEVICE_INIT(pinmux_stm32, STM32_PINMUX_NAME, &pinmux_stm32_init,
	    NULL, &pinmux_stm32_cfg,
	    PRIMARY, CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
