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
 * @file
 * @brief System/hardware module for STM32F103VE processor
 *
 */

#include <nanokernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <rcc.h>

#ifdef CONFIG_RUNTIME_NMI
extern void _NmiInit(void);
#define NMI_INIT() _NmiInit()
#else
#define NMI_INIT()
#endif

/**
 * @brief Setup various clock on SoC.
 */
static void clock_init(void)
{
	volatile struct stm32f10x_rcc *rcc = (struct stm32f10x_rcc *)(RCC_BASE);

	/* enable HSI clock */
	rcc->cr.bit.hsion = 1;
	/* this should end after one test */
	while (rcc->cr.bit.hsirdy != 1);

	/* aim for 36MHz SYSCLK */

	/* setup PLL first */
	rcc->cr.bit.pllon = 0;

	/* PLL input from HSI/2 = 4MHz */
	rcc->cfgr.bit.pllsrc = 0;

	/* setup PLL for x9 multiplication -> 36MHz */
	rcc->cfgr.bit.pllmul = 0x7;

	/* enable PLL */
	rcc->cr.bit.pllon = 1;

	/* wait for it to become ready */
	while (rcc->cr.bit.pllrdy != 1);

	/* setup AHB prescaler to 0, 36MHz on AHB */
	rcc->cfgr.bit.hpre = 0;

	/* setup APB1, must not exceed 36MHz, prescaler set to 0 */
	rcc->cfgr.bit.ppre1 = 0x0;

	/* setup APB2 to use 36MHz, prescaler set to 0 */
	rcc->cfgr.bit.ppre2 = 0x0;

	/* setup SYSCLK source from PLL */
	rcc->cfgr.bit.sw = 0x2;	/* 0b10 */

	/* wait for SYSCLK to switch to PLL */
	while (rcc->cfgr.bit.sws != 0x2);

	/* done, we should be running at 36MHz, APB1 at 36MHz, APB2 at
	 * 36MHz */
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int stm32f103ve_init(struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	/* Note:
	 * Magic numbers below are obtained by reading the registers
	 * when the SoC was running the SAM-BA bootloader
	 * (with reserved bits set to 0).
	 */

	key = irq_lock();

	/* Setup the vector table offset register (VTOR),
	 * which is located at the beginning of flash area.
	 */
	_scs_relocate_vector_table((void *)CONFIG_FLASH_BASE_ADDRESS);

	/* Clear all faults */
	_ScbMemFaultAllFaultsReset();
	_ScbBusFaultAllFaultsReset();
	_ScbUsageFaultAllFaultsReset();

	_ScbHardFaultAllFaultsReset();

	/* Setup master clock */
	clock_init();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(stm32f103ve_init, PRIMARY, 0);
