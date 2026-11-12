#include <stdint.h>

/* 中文注释：复位后使用 NUCLEO-F446RE 默认的 16 MHz HSI 时钟。 */
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

#define RCC_AHB1ENR REG32(0x40023830U)
#define RCC_APB2ENR REG32(0x40023844U)
#define GPIOA_MODER REG32(0x40020000U)
#define GPIOA_AFRH REG32(0x40020024U)
#define GPIOA_BSRR REG32(0x40020018U)

#define USART1_SR REG32(0x40011000U)
#define USART1_DR REG32(0x40011004U)
#define USART1_BRR REG32(0x40011008U)
#define USART1_CR1 REG32(0x4001100CU)
#define USART1_CR2 REG32(0x40011010U)
#define USART1_CR3 REG32(0x40011014U)

#define SYST_CSR REG32(0xE000E010U)
#define SYST_RVR REG32(0xE000E014U)
#define SYST_CVR REG32(0xE000E018U)

#define USART_SR_TXE (1U << 7)
#define USART_SR_RXNE (1U << 5)
#define USART_CR1_UE (1U << 13)
#define USART_CR1_TE (1U << 3)
#define USART_CR1_RE (1U << 2)

#define RX_BUFFER_SIZE 112U
#define ACK_BUFFER_SIZE 112U
#define FRAME_TIMEOUT_MS 300U

typedef struct {
    int32_t sequence;
    int32_t line_valid;
    int32_t line_dx_permille;
    int32_t heading_millideg;
    int32_t obstacle_valid;
    int32_t obstacle_x_permille;
    int32_t obstacle_w_permille;
    int32_t confidence_permille;
} vision_frame_t;

static volatile uint32_t g_milliseconds;
static volatile uint32_t g_last_valid_ms;
static volatile uint8_t g_have_valid_frame;
static char g_rx_buffer[RX_BUFFER_SIZE];
static uint16_t g_rx_length;
static uint8_t g_discard_until_newline;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static void systick_init(void)
{
    /* 中文注释：16 MHz 时钟除以 16000，产生每毫秒一次的中断。 */
    SYST_CSR = 0U;
    SYST_RVR = 16000U - 1U;
    SYST_CVR = 0U;
    SYST_CSR = 7U;
}

static void uart1_init(void)
{
    /* 中文注释：打开 GPIOA 和 USART1 的时钟。 */
    RCC_AHB1ENR |= (1U << 0);
    RCC_APB2ENR |= (1U << 4);
    (void)RCC_APB2ENR;

    /* 中文注释：PA9、PA10 设为 AF7，分别连接 USART1_TX、USART1_RX。 */
    GPIOA_MODER &= ~((3U << 18) | (3U << 20));
    GPIOA_MODER |= (2U << 18) | (2U << 20);
    GPIOA_AFRH &= ~((15U << 4) | (15U << 8));
    GPIOA_AFRH |= (7U << 4) | (7U << 8);

    /* 中文注释：16 MHz HSI 下 BRR=139，约 115200、8N1。 */
    USART1_CR1 = 0U;
    USART1_CR2 = 0U;
    USART1_CR3 = 0U;
    USART1_BRR = 139U;
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void led_init(void)
{
    /* 中文注释：板载绿色 LD2 接 PA5，高电平点亮。 */
    GPIOA_MODER &= ~(3U << 10);
    GPIOA_MODER |= (1U << 10);
    GPIOA_BSRR = (1U << (5U + 16U));
}

static void led_set(uint8_t on)
{
    if (on != 0U) {
        GPIOA_BSRR = (1U << 5);
    } else {
        GPIOA_BSRR = (1U << (5U + 16U));
    }
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

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static int8_t hex_value(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

static uint8_t parse_hex4(const char *text, uint16_t *value)
{
    uint16_t result = 0U;
    for (uint8_t i = 0U; i < 4U; i++) {
        const int8_t digit = hex_value(text[i]);
        if (digit < 0) return 0U;
        result = (uint16_t)((result << 4) | (uint16_t)digit);
    }
    *value = result;
    return 1U;
}

static uint8_t parse_integer(const char *text, int32_t *value)
{
    uint8_t negative = 0U;
    uint8_t digits = 0U;
    uint32_t magnitude = 0U;

    if (*text == '-') {
        negative = 1U;
        text++;
    } else if (*text == '+') {
        text++;
    }

    while (*text >= '0' && *text <= '9') {
        digits = 1U;
        magnitude = magnitude * 10U + (uint32_t)(*text - '0');
        if (magnitude > 1000000000U) return 0U;
        text++;
    }
    if (digits == 0U || *text != '\0') return 0U;

    *value = negative != 0U ? -(int32_t)magnitude : (int32_t)magnitude;
    return 1U;
}

static uint8_t append_char(char *buffer, uint16_t capacity, uint16_t *length, char c)
{
    if (*length >= capacity) return 0U;
    buffer[*length] = c;
    (*length)++;
    return 1U;
}

static uint8_t append_text(char *buffer, uint16_t capacity, uint16_t *length, const char *text)
{
    while (*text != '\0') {
        if (append_char(buffer, capacity, length, *text++) == 0U) return 0U;
    }
    return 1U;
}

static uint8_t append_i32(char *buffer, uint16_t capacity, uint16_t *length, int32_t value)
{
    char reversed[11];
    uint8_t digits = 0U;
    uint32_t magnitude;

    if (value < 0) {
        if (append_char(buffer, capacity, length, '-') == 0U) return 0U;
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        reversed[digits++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U && digits < sizeof(reversed));

    while (digits > 0U) {
        if (append_char(buffer, capacity, length, reversed[--digits]) == 0U) return 0U;
    }
    return 1U;
}

static void send_response(const char *payload, uint16_t payload_length)
{
    const uint16_t crc = crc16_ccitt((const uint8_t *)payload, payload_length);
    uart1_write_byte('!');
    for (uint16_t i = 0U; i < payload_length; i++) {
        uart1_write_byte((uint8_t)payload[i]);
    }
    uart1_write_byte('*');
    for (int8_t shift = 12; shift >= 0; shift -= 4) {
        const uint8_t digit = (uint8_t)((crc >> shift) & 0x0FU);
        if (digit < 10U) {
            uart1_write_byte((uint8_t)('0' + (int)digit));
        } else {
            uart1_write_byte((uint8_t)('A' + (int)(digit - 10U)));
        }
    }
    uart1_write_text("\r\n");
}

static void send_ok(const vision_frame_t *frame)
{
    char payload[ACK_BUFFER_SIZE];
    uint16_t length = 0U;
    if (append_text(payload, sizeof(payload), &length, "1,") == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->sequence) == 0U ||
        append_text(payload, sizeof(payload), &length, ",OK,") == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->line_valid) == 0U ||
        append_char(payload, sizeof(payload), &length, ',') == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->line_dx_permille) == 0U ||
        append_char(payload, sizeof(payload), &length, ',') == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->heading_millideg) == 0U ||
        append_char(payload, sizeof(payload), &length, ',') == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->obstacle_valid) == 0U ||
        append_char(payload, sizeof(payload), &length, ',') == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->obstacle_x_permille) == 0U ||
        append_char(payload, sizeof(payload), &length, ',') == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->obstacle_w_permille) == 0U ||
        append_char(payload, sizeof(payload), &length, ',') == 0U ||
        append_i32(payload, sizeof(payload), &length, frame->confidence_permille) == 0U) {
        return;
    }
    send_response(payload, length);
}

static void send_error(int32_t sequence, uint8_t error_code)
{
    char payload[40];
    uint16_t length = 0U;
    if (append_text(payload, sizeof(payload), &length, "1,") == 0U ||
        append_i32(payload, sizeof(payload), &length, sequence) == 0U ||
        append_text(payload, sizeof(payload), &length, ",ERR,") == 0U ||
        append_i32(payload, sizeof(payload), &length, error_code) == 0U) {
        return;
    }
    send_response(payload, length);
}

static uint8_t parse_frame(char *line, uint16_t length, vision_frame_t *frame, uint8_t *error_code)
{
    int16_t star = -1;
    uint16_t received_crc = 0U;
    int32_t values[9];

    if (length < 10U || line[0] != '@') {
        *error_code = 1U;
        return 0U;
    }
    for (uint16_t i = 1U; i < length; i++) {
        if (line[i] == '*') {
            if (star >= 0) {
                *error_code = 1U;
                return 0U;
            }
            star = (int16_t)i;
        }
    }
    if (star < 2 || (uint16_t)star + 5U != length ||
        parse_hex4(&line[(uint16_t)star + 1U], &received_crc) == 0U) {
        *error_code = 1U;
        return 0U;
    }

    const uint16_t calculated_crc = crc16_ccitt((const uint8_t *)&line[1], (uint16_t)star - 1U);
    line[(uint16_t)star] = '\0';
    char *cursor = &line[1];

    for (uint8_t field = 0U; field < 9U; field++) {
        char *start = cursor;
        while (*cursor != '\0' && *cursor != ',') cursor++;
        if (field < 8U) {
            if (*cursor != ',') {
                *error_code = 1U;
                return 0U;
            }
            *cursor++ = '\0';
        } else if (*cursor != '\0') {
            *error_code = 1U;
            return 0U;
        }
        if (parse_integer(start, &values[field]) == 0U) {
            *error_code = 1U;
            return 0U;
        }
    }

    frame->sequence = values[1];
    frame->line_valid = values[2];
    frame->line_dx_permille = values[3];
    frame->heading_millideg = values[4];
    frame->obstacle_valid = values[5];
    frame->obstacle_x_permille = values[6];
    frame->obstacle_w_permille = values[7];
    frame->confidence_permille = values[8];

    if (calculated_crc != received_crc) {
        *error_code = 2U;
        return 0U;
    }
    if (values[0] != 1 || frame->sequence < 0 || frame->sequence > 65535 ||
        (frame->line_valid != 0 && frame->line_valid != 1) ||
        frame->line_dx_permille < -1000 || frame->line_dx_permille > 1000 ||
        frame->heading_millideg < -180000 || frame->heading_millideg > 180000 ||
        (frame->obstacle_valid != 0 && frame->obstacle_valid != 1) ||
        frame->obstacle_x_permille < 0 || frame->obstacle_x_permille > 1000 ||
        frame->obstacle_w_permille < 0 || frame->obstacle_w_permille > 1000 ||
        frame->confidence_permille < 0 || frame->confidence_permille > 1000) {
        *error_code = 3U;
        return 0U;
    }

    *error_code = 0U;
    return 1U;
}

static void handle_line(uint16_t length)
{
    vision_frame_t frame = {0};
    uint8_t error_code = 1U;

    if (parse_frame(g_rx_buffer, length, &frame, &error_code) != 0U) {
        g_last_valid_ms = g_milliseconds;
        g_have_valid_frame = 1U;
        send_ok(&frame);
    } else {
        send_error(frame.sequence, error_code);
    }
}

int main(void)
{
    uint8_t led_on = 0U;

    uart1_init();
    led_init();
    systick_init();

    /* 中文注释：协议已启动后先发一条状态消息，随后等待 MaixCAM2 数据帧。 */
    uart1_write_text("PROTO_V1_READY\r\n");

    for (;;) {
        if ((USART1_SR & USART_SR_RXNE) != 0U) {
            const char value = (char)(USART1_DR & 0xFFU);
            if (value == '\r') {
                /* 中文注释：忽略 CR，使用 LF 作为一帧的结束符。 */
            } else if (value == '\n') {
                if (g_discard_until_newline == 0U && g_rx_length > 0U) {
                    handle_line(g_rx_length);
                }
                g_rx_length = 0U;
                g_discard_until_newline = 0U;
            } else if (g_discard_until_newline == 0U) {
                if (g_rx_length < (RX_BUFFER_SIZE - 1U)) {
                    g_rx_buffer[g_rx_length++] = value;
                } else {
                    g_rx_length = 0U;
                    g_discard_until_newline = 1U;
                }
            }
        }

        const uint8_t should_be_on = (g_have_valid_frame != 0U &&
            (uint32_t)(g_milliseconds - g_last_valid_ms) < FRAME_TIMEOUT_MS) ? 1U : 0U;
        if (should_be_on != led_on) {
            led_set(should_be_on);
            led_on = should_be_on;
        }

    }
}
