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
#include <clock_control/stm32f10x_clock_control.h>
#include <pinmux/pinmux_stm32f10x.h>
#include <pinmux.h>


/**
 * @brief configuration of GPIO device
 */
struct gpio_stm32f10x_config {
	/* port base address */
	uint8_t *base;
	/* IO port */
	enum pin_port port;
};

/**
 * @brief Configure pin or port
 */
static int gpio_stm32f10x_config(struct device *dev, int access_op,
			    uint32_t pin, int flags)
{
	if (access_op != GPIO_ACCESS_BY_PIN)
		return DEV_NO_SUPPORT;

	struct device *pindev = device_get_binding(PINMUX_NAME);
	struct gpio_stm32f10x_config *cfg = dev->config->config_info;

	int pincfg = 0;
	/* pretend we only support out direction */
	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_OUT)
		pincfg = PIN_CONFIG_DRIVE_PUSH_PULL;
	else
		return DEV_NO_SUPPORT;

	return pinmux_pin_set(pindev, STM32PIN(cfg->port, pin), pincfg);
}

/**
 * @brief Set the pin or port output
 */
static int gpio_stm32f10x_write(struct device *dev, int access_op,
			   uint32_t pin, uint32_t value)
{
	struct gpio_stm32f10x_config *cfg = dev->config->config_info;
	struct stm32f10x_gpio *gpio = (struct stm32f10x_gpio *)cfg->base;

	int pval = 1 << (pin & 0xf);
	if (value)
		gpio->odr |= pval;
	else
		gpio->odr &= ~pval;

	return DEV_OK;
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

static struct gpio_driver_api gpio_stm32f10x_driver = {
	.config = gpio_stm32f10x_config,
	.write = gpio_stm32f10x_write,
	.read = gpio_stm32f10x_read,
	.set_callback = gpio_stm32f10x_set_callback,
	.enable_callback = gpio_stm32f10x_enable_callback,
	.disable_callback = gpio_stm32f10x_disable_callback,
	.suspend = gpio_stm32f10x_suspend_port,
	.resume = gpio_stm32f10x_resume_port,

};

static clock_control_subsys_t __port_to_subsys(uint8_t *base) {
	struct {
		uint8_t *base;
		clock_control_subsys_t subsys;
	} map[] = {
		{(uint8_t *)GPIOA_BASE,
		 UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPA)},
		{(uint8_t *)GPIOB_BASE,
		 UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPB)},
		{(uint8_t *)GPIOC_BASE,
		 UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPC)},
		{(uint8_t *)GPIOD_BASE,
		 UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPD)},
		{(uint8_t *)GPIOE_BASE,
		 UINT_TO_POINTER(STM32F10X_CLOCK_SUBSYS_IOPE)},
	};

	for (int i = 0; i < sizeof(map)/sizeof(map[0]); i++) {
		if (base == map[i].base)
			return map[i].subsys;
	}
	return 0;
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 * NOTE: hardcided to setup USART1 only
 *
 * @param dev UART device struct
 *
 * @return DEV_OK
 */
static int gpio_stm32f10x_init(struct device *device)
{
	struct gpio_stm32f10x_config *cfg = device->config->config_info;

	/* enable clock for subsystem */
	struct device *clk =
		device_get_binding(STM32F10X_CLOCK_CONTROL_NAME);
	clock_control_subsys_t subsys = __port_to_subsys(cfg->base);
	clock_control_on(clk, subsys);

	device->driver_api = &gpio_stm32f10x_driver;
	return DEV_OK;
}

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr, __port)	       \
static struct gpio_stm32f10x_config gpio_stm32f10x_cfg_## __suffix = { \
	.base = (uint8_t *)__base_addr,				      \
	.port = __port,						      \
};								      \
DEVICE_INIT(gpio_stm32f10x_## __suffix,				      \
	__name,							      \
	gpio_stm32f10x_init,					      \
	NULL,			      \
	&gpio_stm32f10x_cfg_## __suffix,			      \
	SECONDARY,						      \
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE)

#ifdef CONFIG_GPIO_STM32F10X_PORTA
GPIO_DEVICE_INIT("GPIOA", a, GPIOA_BASE, STM32_PORTA);
#endif /* CONFIG_GPIO_STM32F10X_PORTA */

#ifdef CONFIG_GPIO_STM32F10X_PORTB
GPIO_DEVICE_INIT("GPIOB", b, GPIOB_BASE, STM32_PORTB);
#endif /* CONFIG_GPIO_STM32F10X_PORTB */

#ifdef CONFIG_GPIO_STM32F10X_PORTC
GPIO_DEVICE_INIT("GPIOC", c, GPIOC_BASE, STM32_PORTC);
#endif /* CONFIG_GPIO_STM32F10X_PORTC */

#ifdef CONFIG_GPIO_STM32F10X_PORTD
GPIO_DEVICE_INIT("GPIOD", d, GPIOD_BASE, STM32_PORTD);
#endif /* CONFIG_GPIO_STM32F10X_PORTD */

#ifdef CONFIG_GPIO_STM32F10X_PORTE
GPIO_DEVICE_INIT("GPIOE", e, GPIOE_BASE, STM32_PORTE);
#endif /* CONFIG_GPIO_STM32F10X_PORTE */

#ifdef CONFIG_GPIO_STM32F10X_PORTF
GPIO_DEVICE_INIT("GPIOF", f, GPIOF_BASE, STM32_PORTF);
#endif /* CONFIG_GPIO_STM32F10X_PORTF */

#ifdef CONFIG_GPIO_STM32F10X_PORTG
GPIO_DEVICE_INIT("GPIOG", g, GPIOG_BASE, STM32_PORTG);
#endif /* CONFIG_GPIO_STM32F10X_PORTG */
