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

#ifndef __PINMUX_STM32F10X_H__
#define __PINMUX_STM32F10X_H__

enum pin_port {
	STM32_PORTA = 0,
	STM32_PORTB = 1,
	STM32_PORTC = 2,
	STM32_PORTD = 3,
	STM32_PORTE = 4,
	STM32_PORTF = 5,
	STM32_PORTG = 6,
};

/* pin functions */
enum pin_config_mode {
	PIN_CONFIG_BIAS_HIGH_IMPEDANCE,
	PIN_CONFIG_BIAS_PULL_UP,
	PIN_CONFIG_BIAS_PULL_DOWN,
	PIN_CONFIG_ANALOG,
	PIN_CONFIG_DRIVE_OPEN_DRAIN,
	PIN_CONFIG_DRIVE_PUSH_PULL,
	PIN_CONFIG_AF_PUSH_PULL,
	PIN_CONFIG_AF_OPEN_DRAIN,
};

#define STM32PIN(_port, _pin) \
	(_port << 4 | _pin)

#endif /* __PINMUX_STM32F10X_H__ */
