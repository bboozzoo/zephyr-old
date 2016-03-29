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
 *   STM32F303xB/C/D/E, STM32F303x6/8, STM32F328x8, STM32F358xC,
 *   STM32F398xE advanced ARM ® -based MCUs
 *
 * Chapter 11: General-purpose I/Os (GPIO)
 */

#include <device.h>
#include "soc.h"
#include "soc_registers.h"
#include <gpio.h>
#include <gpio/gpio_stm32.h>

/**
 * @brief map pin function to MODE register value
 */
static uint32_t __func_to_mode(int func)
{
	switch (STM32_MODE(func)) {
	case STM32_PIN_CONFIG_ANALOG:
		return 0x3;
	case STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
	case STM32_PIN_CONFIG_BIAS_PULL_UP:
	case STM32_PIN_CONFIG_BIAS_PULL_DOWN:
		return 0x0;
	case STM32_PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case STM32_PIN_CONFIG_DRIVE_PUSH_PULL:
		return 0x1;
	case STM32_PIN_CONFIG_AF:
		return 0x2;
	}
	return 0;
}

int stm32_gpio_configure(uint32_t *base_addr, int pin, stm32_pin_func_t conf)
{
	volatile struct stm32_gpio *gpio =
		(struct stm32_gpio *)(base_addr);
	int mode, cmode;

	cmode = STM32_MODE(conf);
	mode = __func_to_mode(conf);

	/* clear bits */
	gpio->moder &= ~(0x3 << (pin * 4));
	/* set bits */
	gpio->moder |= mode << (pin * 4);

	if (cmode == STM32_PIN_CONFIG_BIAS_PULL_UP
		|| cmode == STM32_PIN_CONFIG_BIAS_PULL_DOWN
		|| cmode == STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE) {
		/* input modes setup */

		/* configure pin as floating by clearing pupd flags */
		gpio->pupdr &= ~(0x3 << pin);

		if (conf == STM32_PIN_CONFIG_BIAS_PULL_UP) {
			/* enable pull up */
			gpio->pupdr |= 1 << pin;
		} else if (conf == STM32_PIN_CONFIG_BIAS_PULL_DOWN) {
			/* or pull down */
			gpio->pupdr &= ~(2 << pin);
		}
	} else if (cmode == STM32_PIN_CONFIG_AF) {
		/* alternate function setup */
		int af = STM32_AF(conf);
		uint32_t *afr = &gpio->afrl;
		int crpin = pin;

		if (crpin > 7) {
			afr = &gpio->afrh;
			crpin -= 7;
		}

		/* clear AF bits */
		*afr &= ~(0xf << crpin);
		/* set AF */
		*afr |= (af << crpin);
	}

	return 0;
}

int stm32_gpio_enable_int(int port, int pin)
{
#warning add SYSCFG support
	return 0;
}
