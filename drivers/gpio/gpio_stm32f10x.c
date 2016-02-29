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

/**
 * @brief configuration of GPIO device
 */
struct gpio_stm32f10x_config {
	/* port base address */
	uint8_t *base;
};

/**
 * @brief Configure pin or port
 */
static int gpio_stm32f10x_config(struct device *dev, int access_op,
			    uint32_t pin, int flags)
{
	return DEV_NO_SUPPORT;
}

/**
 * @brief Set the pin or port output
 */
static int gpio_stm32f10x_write(struct device *dev, int access_op,
			   uint32_t pin, uint32_t value)
{
	return DEV_NO_SUPPORT;
}

/**
 * @brief Read the pin or port status
 */
static int gpio_stm32f10x_read(struct device *dev, int access_op,
				       uint32_t pin, uint32_t *value)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32f10x_set_callback(struct device *dev,
				  gpio_callback_t callback)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32f10x_enable_callback(struct device *dev,
				     int access_op, uint32_t pin)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32f10x_disable_callback(struct device *dev,
				      int access_op, uint32_t pin)
{
	return DEV_INVALID_OP;
}

static int gpio_stm32f10x_suspend_port(struct device *dev)
{
	ARG_UNUSED(dev);

	return DEV_INVALID_OP;
}

static int gpio_stm32f10x_resume_port(struct device *dev)
{
	ARG_UNUSED(dev);

	return DEV_INVALID_OP;
}

static struct gpio_driver_api stm32f10x_gpio_driver = {
	.config = gpio_stm32f10x_config,
	.write = gpio_stm32f10x_write,
	.read = gpio_stm32f10x_read,
	.set_callback = gpio_stm32f10x_set_callback,
	.enable_callback = gpio_stm32f10x_enable_callback,
	.disable_callback = gpio_stm32f10x_disable_callback,
	.suspend = gpio_stm32f10x_suspend_port,
	.resume = gpio_stm32f10x_resume_port,

};

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr)	\
static struct gpio_stm32f10x_config gpio_stm32f10x_cfg_## __suffix = { \
	.base = (uint8_t *)__base_addr,				      \
};								      \
DEVICE_INIT(gpio_stm32f10x_## __suffix,				      \
	__name,							      \
	gpio_stm32f10x_init,					      \
	NULL,							      \
	&gpio_stm32f10x_cfg_## __suffix,			      \
	SECONDARY,						      \
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE)

#ifdef CONFIG_GPIO_STM32F10X_PORTA
GPIO_DEVICE_INIT("GPIOA", a, GPIOA_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTA */

#ifdef CONFIG_GPIO_STM32F10X_PORTB
GPIO_DEVICE_INIT("GPIOB", b, GPIOB_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTB */

#ifdef CONFIG_GPIO_STM32F10X_PORTC
GPIO_DEVICE_INIT("GPIOC", c, GPIOC_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTC */

#ifdef CONFIG_GPIO_STM32F10X_PORTD
GPIO_DEVICE_INIT("GPIOD", d, GPIOD_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTD */

#ifdef CONFIG_GPIO_STM32F10X_PORTE
GPIO_DEVICE_INIT("GPIOE", e, GPIOE_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTE */

#ifdef CONFIG_GPIO_STM32F10X_PORTF
GPIO_DEVICE_INIT("GPIOF", f, GPIOF_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTF */

#ifdef CONFIG_GPIO_STM32F10X_PORTG
GPIO_DEVICE_INIT("GPIOG", g, GPIOG_BASE);
#endif /* CONFIG_GPIO_STM32F10X_PORTG */
