#include <stdint.h>

/* 中文注释：复位后使用 NUCLEO-F446RE 默认的 16 MHz HSI 时钟。 */
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

#define RCC_AHB1ENR REG32(0x40023830U)
#define RCC_APB1ENR REG32(0x40023840U)

#define GPIOA_MODER REG32(0x40020000U)
#define GPIOA_AFRL REG32(0x40020020U)

#define GPIOB_MODER REG32(0x40020400U)
#define GPIOB_OTYPER REG32(0x40020404U)
#define GPIOB_OSPEEDR REG32(0x40020408U)
#define GPIOB_PUPDR REG32(0x4002040CU)
#define GPIOB_BSRR REG32(0x40020418U)

#define USART2_SR REG32(0x40004400U)
#define USART2_DR REG32(0x40004404U)
#define USART2_BRR REG32(0x40004408U)
#define USART2_CR1 REG32(0x4000440CU)
#define USART2_CR2 REG32(0x40004410U)
#define USART2_CR3 REG32(0x40004414U)

#define SYST_CSR REG32(0xE000E010U)
#define SYST_RVR REG32(0xE000E014U)
#define SYST_CVR REG32(0xE000E018U)

#define USART_SR_TXE (1U << 7)
#define USART_SR_RXNE (1U << 5)
#define USART_CR1_UE (1U << 13)
#define USART_CR1_TE (1U << 3)
#define USART_CR1_RE (1U << 2)

/* 中文注释：D3/PB3、D4/PB5、D5/PB4、D6/PB10 分别控制 AT8236 的 A/B 两路。 */
#define MOTOR_A_IN1 (1U << 3)
#define MOTOR_A_IN2 (1U << 5)
#define MOTOR_B_IN1 (1U << 4)
#define MOTOR_B_IN2 (1U << 10)
#define MOTOR_INPUT_MASK (MOTOR_A_IN1 | MOTOR_A_IN2 | MOTOR_B_IN1 | MOTOR_B_IN2)

#define COMMAND_BUFFER_SIZE 16U
#define ARM_WINDOW_MS 5000U
#define MOTOR_PULSE_MS 200U

static volatile uint32_t g_milliseconds;
static uint8_t g_armed;
static uint32_t g_arm_deadline;
static char g_command[COMMAND_BUFFER_SIZE];
static uint8_t g_command_length;
static uint8_t g_discard_line;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static void delay_ms(uint32_t duration_ms)
{
    const uint32_t start = g_milliseconds;
    while ((uint32_t)(g_milliseconds - start) < duration_ms) {
        __asm volatile("wfi");
    }
}

static void motor_inputs_safe_low(void)
{
    const uint32_t pins[] = {3U, 5U, 4U, 10U};
    uint32_t config_mask = 0U;
    uint32_t pulldown_bits = 0U;
    uint32_t output_mode_bits = 0U;

    /* 中文注释：打开 GPIOB 时钟，先清除输出锁存值，再配置输出模式。 */
    RCC_AHB1ENR |= (1U << 1);
    (void)RCC_AHB1ENR;
    GPIOB_BSRR = MOTOR_INPUT_MASK << 16;

    for (uint32_t i = 0U; i < (uint32_t)(sizeof(pins) / sizeof(pins[0])); i++) {
        const uint32_t shift = pins[i] * 2U;
        config_mask |= 3U << shift;
        pulldown_bits |= 2U << shift;
        output_mode_bits |= 1U << shift;
    }

    /* 中文注释：四个控制脚使用推挽、低速、内部下拉，并保持输出低电平。 */
    GPIOB_PUPDR = (GPIOB_PUPDR & ~config_mask) | pulldown_bits;
    GPIOB_OTYPER &= ~MOTOR_INPUT_MASK;
    GPIOB_OSPEEDR &= ~config_mask;
    GPIOB_MODER = (GPIOB_MODER & ~config_mask) | output_mode_bits;
    GPIOB_BSRR = MOTOR_INPUT_MASK << 16;
}

static void uart2_init(void)
{
    /* 中文注释：NUCLEO 默认把 USART2 PA2/PA3 接到 ST-LINK 虚拟串口。 */
    RCC_AHB1ENR |= (1U << 0);
    RCC_APB1ENR |= (1U << 17);
    (void)RCC_APB1ENR;

    /* 中文注释：PA2、PA3 设为 AF7，即 USART2_TX、USART2_RX。 */
    GPIOA_MODER &= ~((3U << 4) | (3U << 6));
    GPIOA_MODER |= (2U << 4) | (2U << 6);
    GPIOA_AFRL &= ~((15U << 8) | (15U << 12));
    GPIOA_AFRL |= (7U << 8) | (7U << 12);

    /* 中文注释：16 MHz HSI 下 BRR=139，对应 115200 baud、8N1。 */
    USART2_CR1 = 0U;
    USART2_CR2 = 0U;
    USART2_CR3 = 0U;
    USART2_BRR = 139U;
    USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void systick_init(void)
{
    /* 中文注释：使用 16 MHz HSI 产生 1 ms 系统节拍，用于限定短脉冲时长。 */
    SYST_CSR = 0U;
    SYST_RVR = 16000U - 1U;
    SYST_CVR = 0U;
    SYST_CSR = 7U;
}

static void uart2_write_byte(uint8_t value)
{
    while ((USART2_SR & USART_SR_TXE) == 0U) {
    }
    USART2_DR = value;
}

static void uart2_write_text(const char *text)
{
    while (*text != '\0') {
        uart2_write_byte((uint8_t)*text++);
    }
}

static uint8_t text_equals(const char *left, const char *right)
{
    while (*left != '\0' && *right != '\0') {
        if (*left != *right) return 0U;
        left++;
        right++;
    }
    return (*left == '\0' && *right == '\0') ? 1U : 0U;
}

static void motor_pulse(uint8_t motor_b)
{
    /* 中文注释：每次只启用一路输入；右侧 A 电机实测需反向，使用 AIN2；左侧 B 维持 BIN1。 */
    motor_inputs_safe_low();
    if (motor_b == 0U) {
        GPIOB_BSRR = MOTOR_A_IN2;
    } else {
        GPIOB_BSRR = MOTOR_B_IN1;
    }

    delay_ms(MOTOR_PULSE_MS);
    motor_inputs_safe_low();
}

static void handle_command(const char *command)
{
    if (text_equals(command, "ARM") != 0U) {
        g_armed = 1U;
        g_arm_deadline = g_milliseconds + ARM_WINDOW_MS;
        uart2_write_text("ARMED 5s; send A or B\r\n");
        return;
    }

    if (g_armed != 0U && (int32_t)(g_milliseconds - g_arm_deadline) < 0) {
        g_armed = 0U;
        if (text_equals(command, "A") != 0U) {
            uart2_write_text("A_START 200ms\r\n");
            motor_pulse(0U);
            uart2_write_text("A_DONE; inputs low\r\n");
            return;
        }
        if (text_equals(command, "B") != 0U) {
            uart2_write_text("B_START 200ms\r\n");
            motor_pulse(1U);
            uart2_write_text("B_DONE; inputs low\r\n");
            return;
        }
        uart2_write_text("CANCELLED; inputs low\r\n");
        return;
    }

    g_armed = 0U;
    uart2_write_text("IGNORED; send ARM first\r\n");
}

static void uart2_process_input(void)
{
    if ((USART2_SR & USART_SR_RXNE) == 0U) return;

    const char character = (char)(USART2_DR & 0xFFU);
    if (character == '\r') return;

    if (character == '\n') {
        if (g_discard_line == 0U) {
            g_command[g_command_length] = '\0';
            if (g_command_length > 0U) handle_command(g_command);
        }
        g_command_length = 0U;
        g_discard_line = 0U;
        return;
    }

    if (g_discard_line != 0U) return;
    if (g_command_length >= (COMMAND_BUFFER_SIZE - 1U)) {
        g_command_length = 0U;
        g_discard_line = 1U;
        g_armed = 0U;
        motor_inputs_safe_low();
        uart2_write_text("LINE_TOO_LONG; inputs low\r\n");
        return;
    }
    g_command[g_command_length++] = character;
}

int main(void)
{
    /* 中文注释：先建立安全低电平，再启动串口和定时器。 */
    motor_inputs_safe_low();
    systick_init();
    uart2_init();
    uart2_write_text("MOTOR_TEST_READY; outputs low\r\n");

    for (;;) {
        if (g_armed != 0U && (int32_t)(g_milliseconds - g_arm_deadline) >= 0) {
            g_armed = 0U;
            motor_inputs_safe_low();
            uart2_write_text("ARM_TIMEOUT; inputs low\r\n");
        }
        uart2_process_input();
    }
}
