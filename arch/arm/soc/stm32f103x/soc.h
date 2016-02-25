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
 * @file SoC configuration macros for the STM32F103 family processors.
 */

#ifndef _STM32F103_SOC_H_
#define _STM32F103_SOC_H_

/* peripherals start address */
#define PERIPH_BASE           0x40000000

/* use naming consistent with STMF10x Peripherals Library */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)
#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

/* uart configuration settings */
#define UART_IRQ_FLAGS 0

#endif /* !_ASMLANGUAGE */

#endif /* _STM32F103_SOC_H_ */
