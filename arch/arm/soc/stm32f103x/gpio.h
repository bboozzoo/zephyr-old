#ifndef _STM32F103_GPIO_H_
#define _STM32F103_GPIO_H_

struct stm32f10x_gpio
{
	uint32_t crl;
	uint32_t crh;
	uint32_t idr;
	uint32_t odr;
	uint32_t bsrr;
	uint32_t brr;
	uint32_t lckr;
};

#endif /* _STM32F103_GPIO_H_ */
