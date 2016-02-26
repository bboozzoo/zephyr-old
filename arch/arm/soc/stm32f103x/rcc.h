#ifndef _STM32F103_RCC_H_
#define _STM32F103_RCC_H_

/**
  * @brief Reset and Clock Control
  */

union __rcc_cr {
	uint32_t val;
	struct {
		uint32_t hsion :1 __packed;
		uint32_t hsirdy :1 __packed;
		uint32_t rsvd__2 :1 __packed;
		uint32_t hsitrim :5 __packed;
		uint32_t hsical :8 __packed;
		uint32_t hseon :1 __packed;
		uint32_t hserdy :1 __packed;
		uint32_t hsebyp :1 __packed;
		uint32_t csson :1 __packed;
		uint32_t rsvd__20_23 :4 __packed;
		uint32_t pllon :1 __packed;
		uint32_t pllrdy :1 __packed;
		uint32_t rsvd__26_31 :6 __packed;
	} bit;
};

union __rcc_cfgr {
	uint32_t val;
	struct {
		uint32_t sw :2 __packed;
		uint32_t sws :2 __packed;
		uint32_t hpre :4 __packed;
		uint32_t ppre1 :3 __packed;
		uint32_t ppre2 :3 __packed;
		uint32_t adcpre :2 __packed;
		uint32_t pllsrc :1 __packed;
		uint32_t pllxtpre :1 __packed;
		uint32_t pllmul :4 __packed;
		uint32_t usbpre :1 __packed;
		uint32_t mco :3 __packed;
		uint32_t rsvd__27_31 :5 __packed;
	} bit;
};

struct stm32f10x_rcc
{
	union __rcc_cr cr;
	union __rcc_cfgr cfgr;
	uint32_t cir;
	uint32_t apb2rstr;
	uint32_t apb1rstr;
	uint32_t ahbenr;
	uint32_t apb2enr;
	uint32_t apb1enr;
	uint32_t bdcr;
	uint32_t csr;
};

#endif /* _STM32F103_RCC_H_ */
