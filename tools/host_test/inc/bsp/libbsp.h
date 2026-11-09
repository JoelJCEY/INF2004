#ifndef HOST_BSP_H
#define HOST_BSP_H
#define IO_BANK0_BASE 0x40014000UL
#define GPIO(n) (0x4001c000UL + 4UL + (n) * 4UL)
#define GPIO_PDE (1 << 2)
#define GPIO_PUE (1 << 3)
/* 31 Oct: PWM-timed ultrasonic echo (drv_ultrasonic.c) */
#define GPIO_OD (1 << 7)
#define GPIO_IE (1 << 6)
#define GPIO_CTRL(n) (IO_BANK0_BASE + 4UL + (n) * 8UL)
#define GPIO_CTRL_FUNCSEL_PWM 4
#define PWM_BASE 0x40050000UL
#define PWM_CHx_CSR 0x00
#define PWM_CHx_DIV 0x04
#define PWM_CHx_CTR 0x08
#define PWM_CHx_TOP 0x10
#define PWM_CH_CSR_EN (1 << 0)
#define RESETS_RESET_PWM 0x00004000UL
#define GPIO_MODE_OUT 1
#define GPIO_MODE_IN 2
ER gpio_set_pin(UINT no, UINT mode); ER gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);
#endif
