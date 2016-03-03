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

/**
 * @brief helper for mapping of GPIO flags to SoC specific config
 *
 * @param flags GPIO encoded flags
 * @param out conf SoC specific pin config
 *
 * @return DEV_OK if flags were mapped to SoC pin config
 */
int stm32_gpio_flags_to_conf(int flags, int *conf);

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param base_addr GPIO port base address
 * @param pin IO pin
 * @param func GPIO mode
 */
int stm32_gpio_configure(uint32_t *base_addr, int pin, int func);

/**
 * @brief helper for configuration of GPIO pin output
 *
 * @param base_addr GPIO port base address
 * @param pin IO pin
 * @param value 1, 0
 */
int stm32_gpio_set(uint32_t *base, int pin, int value);

#endif /* _STM32_GPIO_H_ */
