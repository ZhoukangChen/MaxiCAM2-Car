#include <stdint.h>

/* 中文注释：NUCLEO-F446RE 复位后先使用 16 MHz HSI 时钟。 */
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

#define RCC_AHB1ENR REG32(0x40023830U)
#define GPIOB_MODER REG32(0x40020400U)
#define GPIOB_OTYPER REG32(0x40020404U)
#define GPIOB_OSPEEDR REG32(0x40020408U)
#define GPIOB_PUPDR REG32(0x4002040CU)
#define GPIOB_BSRR REG32(0x40020418U)

/* 中文注释：D3/PB3、D4/PB5、D5/PB4、D6/PB10 接到 AT8236 的四个控制输入。 */
#define MOTOR_INPUT_MASK ((1U << 3) | (1U << 5) | (1U << 4) | (1U << 10))

static void motor_inputs_hold_low(void)
{
    const uint32_t pins[] = {3U, 5U, 4U, 10U};
    uint32_t config_mask = 0U;
    uint32_t pulldown_bits = 0U;
    uint32_t output_mode_bits = 0U;

    /* 中文注释：打开 GPIOB 时钟，并等待寄存器写入生效。 */
    RCC_AHB1ENR |= (1U << 1);
    (void)RCC_AHB1ENR;

    /* 中文注释：先把输出锁存值清零，再配置低速推挽输出，避免切换模式时跳高。 */
    GPIOB_BSRR = MOTOR_INPUT_MASK << 16;

    for (uint32_t i = 0U; i < (uint32_t)(sizeof(pins) / sizeof(pins[0])); i++) {
        const uint32_t shift = pins[i] * 2U;
        config_mask |= 3U << shift;
        pulldown_bits |= 2U << shift;
        output_mode_bits |= 1U << shift;
    }

    /* 中文注释：同时保留内部下拉；在输出模式下，四个脚由推挽低电平主动驱动。 */
    GPIOB_PUPDR = (GPIOB_PUPDR & ~config_mask) | pulldown_bits;
    GPIOB_OTYPER &= ~MOTOR_INPUT_MASK;
    GPIOB_OSPEEDR &= ~config_mask;
    GPIOB_MODER = (GPIOB_MODER & ~config_mask) | output_mode_bits;

    /* 中文注释：再次清零，确保 AIN1、AIN2、BIN1、BIN2 均处于 0。 */
    GPIOB_BSRR = MOTOR_INPUT_MASK << 16;
}

int main(void)
{
    motor_inputs_hold_low();

    /* 中文注释：安全待机，不输出 PWM，不运行电机，也不占用相机串口。 */
    for (;;) {
        __asm volatile("wfi");
    }
}
