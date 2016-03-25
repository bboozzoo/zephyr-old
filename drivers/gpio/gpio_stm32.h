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

#ifndef _STM32_GPIO_H_
#define _STM32_GPIO_H_

/**
 * @file header for STM32 GPIO
 */


#include <clock_control/stm32_clock_control.h>
#include <pinmux/stm32/pinmux_stm32.h>
#include <gpio.h>

/* GPIO registers - each GPIO port controls 16 pins, see respective
 * user manuals for details
 */
struct stm32_gpio {
#if defined(CONFIG_SOC_STM32F1X)
	uint32_t crl;
	uint32_t crh;
	uint32_t idr;
	uint32_t odr;
	uint32_t bsrr;
	uint32_t brr;
	uint32_t lckr;
#else
#error missing definition of GPIO registers for your SoC target
#endif
};

/* IO pin functions are mostly common across STM32 devices. Notable
 * exception is STM32F1 as these MCUs do not have registers for
 * configuration of pin's alternate function. The configuration is
 * done implicitly by setting specific mode and config in MODE and CNF
 * registers for particular pin.
 */
enum stm32_pin_config_mode {
	STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE = 0,
	STM32_PIN_CONFIG_BIAS_PULL_UP,
	STM32_PIN_CONFIG_BIAS_PULL_DOWN,
	STM32_PIN_CONFIG_ANALOG,
	STM32_PIN_CONFIG_DRIVE_OPEN_DRAIN,
	STM32_PIN_CONFIG_DRIVE_PUSH_PULL,
#ifdef CONFIG_SOC_STM32F1X
	/* account for STM32F1 method of pin alternate function
	 * configuration
	 */
	STM32_PIN_CONFIG_AF_PUSH_PULL,
	STM32_PIN_CONFIG_AF_OPEN_DRAIN,
#else
	STM32_PIN_CONFIG_AF,
#endif
};

/**
 * @brief configuration of GPIO device
 */
struct gpio_stm32_config {
	/* port base address */
	uint32_t *base;
	/* IO port */
	enum stm32_pin_port port;
	/* clock subsystem */
	clock_control_subsys_t clock_subsys;
};

/**
 * @brief driver data
 */
struct gpio_stm32_data {
	/* user ISR cb */
	gpio_callback_t cb;
	/* mask of enabled pins */
	uint32_t enabled_mask;
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param base_addr GPIO port base address
 * @param pin IO pin
 * @param func GPIO mode
 */
int stm32_gpio_configure(uint32_t *base_addr, int pin, int func);

/**
 * @brief enable interrupt source for GPIO pin
 * @param port
 * @param pin
 */
int stm32_gpio_enable_int(int port, int pin);

#endif /* _STM32_GPIO_H_ */
