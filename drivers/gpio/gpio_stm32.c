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

#include <nanokernel.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <clock_control/stm32_clock_control.h>
#include <pinmux/pinmux_stm32.h>
#include <pinmux.h>
#include <gpio/gpio_stm32.h>
#include <misc/util.h>

/**
 * @brief Common GPIO driver for STM32 MCUs. Each SoC must implemet a
 * SoC specific part.
 */

/** * @brief configuration of GPIO device
 */
struct gpio_stm32_config {
	/* port base address */
	uint32_t *base;
	/* IO port */
	enum stm32_pin_port port;
	/* clock subsystem */
	clock_control_subsys_t clock_subsys;
};

/**
 * @brief Configure pin or port
 */
static int gpio_stm32_config(struct device *dev, int access_op,
			     uint32_t pin, int flags)
{
	if (access_op != GPIO_ACCESS_BY_PIN)
		return DEV_NO_SUPPORT;

	struct device *pindev = device_get_binding(STM32_PINMUX_NAME);
	struct gpio_stm32_config *cfg = dev->config->config_info;
	int pincfg;
	int map_res;

	/* figure out if we can map the requested GPIO
	 * configuration
	 */
	map_res = stm32_gpio_flags_to_conf(flags, &pincfg);
	if (map_res != DEV_OK)
		return map_res;

	/* configure GPIO mode on pin */
	pinmux_pin_set(pindev, STM32PIN(cfg->port, pin),
		       STM32_PINMUX_FUNC_GPIO);

	return stm32_gpio_configure(cfg->base, pin, pincfg);
}

/**
 * @brief Set the pin or port output
 */
static int gpio_stm32_write(struct device *dev, int access_op,
			   uint32_t pin, uint32_t value)
{
	if (access_op != GPIO_ACCESS_BY_PIN)
		return DEV_NO_SUPPORT;

	struct gpio_stm32_config *cfg = dev->config->config_info;

	return stm32_gpio_set(cfg->base, pin, value);
}

/**
 * @brief Read the pin or port status
 */
static int gpio_stm32_read(struct device *dev, int access_op,
			   uint32_t pin, uint32_t *value)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32_set_callback(struct device *dev,
				   gpio_callback_t callback)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32_enable_callback(struct device *dev,
				      int access_op, uint32_t pin)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32_disable_callback(struct device *dev,
				       int access_op, uint32_t pin)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32_suspend_port(struct device *dev)
{
	ARG_UNUSED(dev);

	return DEV_INVALID_OP;
}

static int gpio_stm32_resume_port(struct device *dev)
{
	ARG_UNUSED(dev);

	return DEV_INVALID_OP;
}

static struct gpio_driver_api gpio_stm32_driver = {
	.config = gpio_stm32_config,
	.write = gpio_stm32_write,
	.read = gpio_stm32_read,
	.set_callback = gpio_stm32_set_callback,
	.enable_callback = gpio_stm32_enable_callback,
	.disable_callback = gpio_stm32_disable_callback,
	.suspend = gpio_stm32_suspend_port,
	.resume = gpio_stm32_resume_port,

};

/**
 * @brief Initialize GPIO port
 *
 * Perform basic initialization of a GPIO port. The code will
 * enable the clock for corresponding peripheral.
 *
 * @param dev GPIO device struct
 *
 * @return DEV_OK
 */
static int gpio_stm32_init(struct device *device)
{
	struct gpio_stm32_config *cfg = device->config->config_info;

	/* enable clock for subsystem */
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	clock_control_on(clk, cfg->clock_subsys);

	device->driver_api = &gpio_stm32_driver;
	return DEV_OK;
}

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr, __port, __clock) \
static struct gpio_stm32_config gpio_stm32_cfg_## __suffix = {		\
	.base = (uint32_t *)__base_addr,				\
	.port = __port,							\
	.clock_subsys = UINT_TO_POINTER(__clock),			\
};									\
DEVICE_INIT(gpio_stm32_## __suffix,					\
	__name,								\
	gpio_stm32_init,						\
	NULL,								\
	&gpio_stm32_cfg_## __suffix,					\
	SECONDARY,							\
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE)

#ifdef CONFIG_GPIO_STM32_PORTA
GPIO_DEVICE_INIT("GPIOA", a, GPIOA_BASE, STM32_PORTA,
#ifdef CONFIG_SOC_STM32F1X
		STM32F10X_CLOCK_SUBSYS_IOPA
#endif
	);
#endif /* CONFIG_GPIO_STM32_PORTA */

#ifdef CONFIG_GPIO_STM32_PORTB
GPIO_DEVICE_INIT("GPIOB", b, GPIOB_BASE, STM32_PORTB,
#ifdef CONFIG_SOC_STM32F1X
		STM32F10X_CLOCK_SUBSYS_IOPB
#endif
	);
#endif /* CONFIG_GPIO_STM32_PORTB */

#ifdef CONFIG_GPIO_STM32_PORTC
GPIO_DEVICE_INIT("GPIOC", c, GPIOC_BASE, STM32_PORTC,
#ifdef CONFIG_SOC_STM32F1X
		STM32F10X_CLOCK_SUBSYS_IOPC
#endif
);
#endif /* CONFIG_GPIO_STM32_PORTC */

#ifdef CONFIG_GPIO_STM32_PORTD
GPIO_DEVICE_INIT("GPIOD", d, GPIOD_BASE, STM32_PORTD,
#ifdef CONFIG_SOC_STM32F1X
		STM32F10X_CLOCK_SUBSYS_IOPD
#endif
	);
#endif /* CONFIG_GPIO_STM32_PORTD */

#ifdef CONFIG_GPIO_STM32_PORTE
GPIO_DEVICE_INIT("GPIOE", e, GPIOE_BASE, STM32_PORTE
#ifdef CONFIG_SOC_STM32F1X
		STM32F10X_CLOCK_SUBSYS_IOPE
#endif
	);
#endif /* CONFIG_GPIO_STM32_PORTE */
