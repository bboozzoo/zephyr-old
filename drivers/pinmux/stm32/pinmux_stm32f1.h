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

#ifndef _STM32F1_PINMUX_H_
#define _STM32F1_PINMUX_H_

/**
 * @file Header for STM32F1 pin multiplexing helper
 */

/**
 * @brief pinmux config wrapper
 *
 * GPIO function is assumed to be always available, as such it's not listed
 * in @funcs array
 */
struct stm32_pinmux_conf {
	uint32_t pin;		 /* pin ID */
	const stm32_pin_func_t *funcs; /* functions array, indexed with
					* (stm32_pin_alt_func - 1)
					*/
	const size_t nfuncs;	 /* number of alternate functions, not
				  * counting GPIO
				  */
};

/**
 * @brief helper to define pins
 */
#define STM32_PIN_CONF(__pin, __funcs) \
	{__pin, __funcs, ARRAY_SIZE(__funcs)}

/* pin alternate function definitions */
#define STM32F1_PINMUX_FUNC_PA9_USART1_TX STM32_PINMUX_FUNC_ALT_1
#define STM32F1_PINMUX_FUNC_PA10_USART1_RX STM32_PINMUX_FUNC_ALT_1

#define STM32F1_PINMUX_FUNC_PA2_USART2_TX STM32_PINMUX_FUNC_ALT_1
#define STM32F1_PINMUX_FUNC_PA3_USART2_RX STM32_PINMUX_FUNC_ALT_1

#define STM32F1_PINMUX_FUNC_PB10_USART3_TX STM32_PINMUX_FUNC_ALT_1
#define STM32F1_PINMUX_FUNC_PB11_USART3_RX STM32_PINMUX_FUNC_ALT_1


#endif /* _STM32F1_PINMUX_H_ */
