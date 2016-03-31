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

#include <device.h>

/**
 * @brief helper for returning ENOTSUP error
 */
static int __notsupp(void)
{
	return -ENOTSUP;
}

static int stm32_ef_latency_set(int clock)
{
	volatile struct stm32_flash *flash =
		(struct stm32_flash *)(FLASH_BASE);
	int latency;

	if (clock <= 24000000) {
		latency = STM32_FLASH_LATENCY_0;
	} else if (clock <= 48000000) {
		latency = STM32_FLASH_LATENCY_1;
	} else if (clock <= 72000000) {
		latency = STM32_FLASH_LATENCY_2;
	}

	flash->acr.bit.latency = latency;

	return 0;
}

static int stm32_ef_cmd(struct device *dev, uint32_t cmd, uint32_t p)
{
	switch (cmd) {
	case STM32_FLASH_CMD_LATENCY_FOR_CLOCK_SET:
		return stm32_ef_latency_set(p);
	default:
		break;
	}
	return -ENOTSUP;
}

static struct flash_driver_api stm32_ef_api = {
	.write = (flash_api_write) __notsupp,
	.read  = (flash_api_read) __notsupp,
	.erase = (flash_api_erase) __notsupp,
	.write_protection = (flash_api_write_protection) __notsupp,
	.cmd = stm32_ef_cmd,
};

static int stm32_ef_init(struct device *dev)
{
	dev->driver_api = &stm32_ef_api;
	return 0;
}

/* initialize the device as early as possible */
DEVICE_INIT(stm32_embedded_flash, STM32_FLASH_NAME, stm32_ef_init,
	NULL, NULL, PRIMARY, 0);
