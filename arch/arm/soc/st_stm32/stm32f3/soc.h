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
 * @file SoC configuration macros for the STM32F3 family processors.
 *
 * Based on reference manual:
 *   STM32F303xB/C/D/E, STM32F303x6/8, STM32F328x8, STM32F358xC,
 *   STM32F398xE advanced ARM ® -based MCUs
 *
 * Chapter 3.3: Memory organization
 */


#ifndef _STM32F3_SOC_H_
#define _STM32F3_SOC_H_

/* peripherals start address */
#define PERIPH_BASE           0x40000000

/* use naming consistent with STM32 Peripherals Library */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x8000000)
#define AHB3PERIPH_BASE       (PERIPH_BASE + 0x10000000)
#define AHB4PERIPH_BASE       (PERIPH_BASE + 0x20000000)

/* UART */
#define USART1_ADDR           (APB2PERIPH_BASE + 0x3800)
#define USART2_ADDR           (APB1PERIPH_BASE + 0x4400)
#define USART3_ADDR           (APB1PERIPH_BASE + 0x4800)

/* Reset and Clock Control */
#define RCC_BASE              (AHB1PERIPH_BASE + 0x1000)

#define GPIO_REG_SIZE         0x400
#define GPIOA_BASE            (AHB2PERIPH_BASE)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB2PERIPH_BASE + 0x1C00)
/* base address for where GPIO registers start */
#define GPIO_PORTS_BASE       (GPIOA_BASE)

/* EXTI */
#define EXTI_BASE            (APB2PERIPH_BASE + 0x0400)

/* AFIO */
#define AFIO_BASE            (APB2PERIPH_BASE + 0x0000)

/* IWDG */
#define IWDG_BASE            (APB1PERIPH_BASE + 0x3000)

/* FLASH */
#define FLASH_BASE           (AHB1PERIPH_BASE + 0x2000)

#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

#include "soc_irq.h"

#endif /* !_ASMLANGUAGE */

#endif /* _STM32F3_SOC_H_ */
