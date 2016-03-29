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

#include "soc.h"
#include <device.h>
#include <misc/util.h>
#include <pinmux/stm32/pinmux_stm32.h>
#include <gpio/gpio_stm32.h>
#include <drivers/clock_control/stm32_clock_control.h>

/*
 * Note that STM32F1 is different from other STM32 SoCs with regard to
 * AF configuration. Since the chip has no separate registers for
 * selecting alternate function, the choice is done implicitly by
 * configuring pins in proper I/O mode.
 */
static const stm32_pin_func_t pin_pa9_funcs[] = {
	[STM32F1_PINMUX_FUNC_PA9_USART1_TX - 1] = STM32_AS_MODE(STM32_PIN_CONFIG_AF_PUSH_PULL),
};

static const stm32_pin_func_t pin_pa10_funcs[] = {
	[STM32F1_PINMUX_FUNC_PA9_USART1_TX - 1] = STM32_AS_MODE(STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE),
};

static const stm32_pin_func_t pin_pa2_funcs[] = {
	[STM32F1_PINMUX_FUNC_PA2_USART2_TX - 1] = STM32_AS_MODE(STM32_PIN_CONFIG_AF_PUSH_PULL),
};

static const stm32_pin_func_t pin_pa3_funcs[] = {
	[STM32F1_PINMUX_FUNC_PA3_USART2_RX - 1] = STM32_AS_MODE(STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE),
};

static const stm32_pin_func_t pin_pb10_funcs[] = {
	[STM32F1_PINMUX_FUNC_PB10_USART3_TX - 1] = STM32_AS_MODE(STM32_PIN_CONFIG_AF_PUSH_PULL),
};

static const stm32_pin_func_t pin_pb11_funcs[] = {
	[STM32F1_PINMUX_FUNC_PB11_USART3_RX - 1] = STM32_AS_MODE(STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE),
};

/**
 * @brief pin configuration
 */
static struct stm32_pinmux_conf pins[] = {
	STM32_PIN_CONF(STM32_PIN_PA2, pin_pa2_funcs),
	STM32_PIN_CONF(STM32_PIN_PA3, pin_pa3_funcs),
	STM32_PIN_CONF(STM32_PIN_PA9, pin_pa9_funcs),
	STM32_PIN_CONF(STM32_PIN_PA10, pin_pa10_funcs),
	STM32_PIN_CONF(STM32_PIN_PB10, pin_pb10_funcs),
	STM32_PIN_CONF(STM32_PIN_PB11, pin_pb11_funcs),
};

int stm32_get_pin_config(int pin, int func, stm32_pin_func_t *soc_func)
{
	/* GPIO function is always available, to save space it is not
	 * listed in alternate functions array
	 */
	if (func == STM32_PINMUX_FUNC_GPIO) {
		*soc_func = STM32_PIN_CONFIG_BIAS_HIGH_IMPEDANCE;
		return 0;
	}

	/* analog function is another 'known' setting */
	if (func == STM32_PINMUX_FUNC_ANALOG) {
		*soc_func = STM32_PIN_CONFIG_ANALOG;
		return 0;
	}

	func -= 1;

	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		if (pins[i].pin == pin) {
			if (func > pins[i].nfuncs) {
				return -EINVAL;
			}

			*soc_func = pins[i].funcs[func];
		}
	}
	return -EINVAL;
}

clock_control_subsys_t stm32_get_port_clock(int port)
{
	const clock_control_subsys_t ports_to_clock[STM32_PORTS_MAX] = {
		UINT_TO_POINTER(STM32_CLOCK_SUBSYS_IOPA),
		UINT_TO_POINTER(STM32_CLOCK_SUBSYS_IOPB),
		UINT_TO_POINTER(STM32_CLOCK_SUBSYS_IOPC),
		UINT_TO_POINTER(STM32_CLOCK_SUBSYS_IOPD),
		UINT_TO_POINTER(STM32_CLOCK_SUBSYS_IOPE),
	};

	if (port > STM32_PORTE) {
		return NULL;
	}

	return ports_to_clock[port];
}
