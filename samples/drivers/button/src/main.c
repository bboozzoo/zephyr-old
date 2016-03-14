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

#include <zephyr.h>
#include <device.h>
#include <gpio.h>

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#define PORT "GPIOB"

void button_pressed(struct device *gpiob, uint32_t pin)
{
	PRINT("button pressed at %d\n", sys_tick_get_32());
}

void main(void)
{
	struct device *gpiob;

	gpiob = device_get_binding(PORT);

	gpio_pin_configure(gpiob, 15,
			GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE
			| GPIO_PUD_PULL_UP);
	gpio_set_callback(gpiob, button_pressed);
	gpio_pin_enable_callback(gpiob, 15);

	while (1) {
		int val = 0;

		gpio_pin_read(gpiob, 15, &val);
		PRINT("GPIO val: %d\n", val);
		task_sleep(MSEC(500));
	}
}
