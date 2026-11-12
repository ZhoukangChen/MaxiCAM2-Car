#include <stdint.h>

/* 中文注释：NUCLEO-F446RE 上电复位后先使用 16 MHz HSI 时钟。 */
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

#define RCC_AHB1ENR REG32(0x40023830U)
#define RCC_APB2ENR REG32(0x40023844U)
#define GPIOA_MODER REG32(0x40020000U)
#define GPIOA_AFRH REG32(0x40020024U)

#define USART1_SR REG32(0x40011000U)
#define USART1_DR REG32(0x40011004U)
#define USART1_BRR REG32(0x40011008U)
#define USART1_CR1 REG32(0x4001100CU)
#define USART1_CR2 REG32(0x40011010U)
#define USART1_CR3 REG32(0x40011014U)

#define USART_SR_TXE (1U << 7)
#define USART_SR_RXNE (1U << 5)
#define USART_CR1_UE (1U << 13)
#define USART_CR1_TE (1U << 3)
#define USART_CR1_RE (1U << 2)

static void uart1_init(void)
{
    /* 中文注释：打开 GPIOA 和 USART1 的时钟。 */
    RCC_AHB1ENR |= (1U << 0);
    RCC_APB2ENR |= (1U << 4);
    (void)RCC_APB2ENR;

    /* 中文注释：PA9、PA10 设为 AF7，分别作为 USART1_TX、USART1_RX。 */
    GPIOA_MODER &= ~((3U << 18) | (3U << 20));
    GPIOA_MODER |= (2U << 18) | (2U << 20);
    GPIOA_AFRH &= ~((15U << 4) | (15U << 8));
    GPIOA_AFRH |= (7U << 4) | (7U << 8);

    /* 中文注释：16 MHz HSI 下 BRR=139，波特率约 115200，8 数据位、无校验、1 停止位。 */
    USART1_CR1 = 0U;
    USART1_CR2 = 0U;
    USART1_CR3 = 0U;
    USART1_BRR = 139U;
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart1_write_byte(uint8_t value)
{
    while ((USART1_SR & USART_SR_TXE) == 0U) {
    }
    USART1_DR = value;
}

static void uart1_write_text(const char *text)
{
    while (*text != '\0') {
        uart1_write_byte((uint8_t)*text++);
    }
}

int main(void)
{
    uart1_init();
    uart1_write_text("NUCLEO_USART1_READY\r\n");

    /* 中文注释：收到相机发来的每个字节后原样回传，用于确认 TX/RX 交叉接线。 */
    for (;;) {
        if ((USART1_SR & USART_SR_RXNE) != 0U) {
            const uint8_t received = (uint8_t)(USART1_DR & 0xFFU);
            uart1_write_byte(received);
        }
    }
}
