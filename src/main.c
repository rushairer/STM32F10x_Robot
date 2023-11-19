#include "stm32f10x.h"
#include "stm32f10x_abl_delay.h"
#include "stm32f10x_abl_key.h"
#include "stm32f10x_abl_led.h"
#include "stm32f10x_abl_oled.h"
#include "stm32f10x_abl_serial.h"
#include "stm32f10x_abl_iap.h"
#include "stm32f10x_abl_joystick.h"

void SERIAL_GetInputString(SERIAL_InitTypeDef *Serial, uint8_t *Buf)
{
    uint32_t bytesRead = 0;
    uint8_t byte       = 0;
    do {
        while (1) {
            if (SERIAL_ReceiveByte(Serial, &byte) == 1) {
                break;
            }
        }
        if (byte == '\n') {
            if (Buf[bytesRead - 1] == '\r')
                break;
        }

        if (byte == '\b') /* Backspace */
        {
            if (bytesRead > 0) {
                bytesRead--;
            }
            continue;
        }
        if (bytesRead >= 128) {
            bytesRead = 0;
            continue;
        }
        if ((byte >= 0x20 && byte <= 0x7E) || byte == '\r') {
            Buf[bytesRead++] = byte;
        }
    } while (1);
    Buf[bytesRead - 1] = '\0';
}

int main()
{
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN, ENABLE);
    NVIC_SetVectorTable(0x08000000, 0x3000); // 设置中断向量表
    Delay_ms(200);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    KEY_InitTypeDef key1;
    KEY_Init(&key1, RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_12);

    LED_InitTypeDef ledDefault;
    LedDefault_Init(&ledDefault);
    LED_On(&ledDefault);

    OLED_InitTypeDef oled1;
    Oled1_Init(&oled1);

    SERIAL_InitTypeDef serial;
    Serial1_Init(&serial);

    JOYSTICK_InitTypeDef Joystick1;
    JOYSTICK_Init(
        &Joystick1,
        RCC_APB2Periph_ADC1,
        ADC1,
        RCC_APB2Periph_GPIOA,
        GPIOA,
        GPIO_Pin_0,
        GPIO_Pin_1,
        ADC_Channel_0,
        ADC_Channel_1,
        GPIO_Pin_2);

    OLED_ShowString(&oled1, 35, 20, " ROBOT ", OLED_FONT_SIZE_16, OlED_COLOR_REVERSED);
    OLED_RefreshScreen(&oled1);

    while (1) {

        // Serial DEMO
        // uint8_t cmd[128] = {0};
        // SERIAL_GetInputString(&serial, cmd);
        // OLED_ShowString(&oled1, 0, 50, (char *)cmd, OLED_FONT_SIZE_12, OLED_COLOR_NORMAL);
        // OLED_RefreshScreen(&oled1);

        // JOYSTICK DEMO
        uint16_t xValue, yValue;
        uint8_t Joystick1IsPressed;
        xValue             = JOYSTICK_GetXValue(&Joystick1);
        yValue             = JOYSTICK_GetYValue(&Joystick1);
        Joystick1IsPressed = JOYSTICK_IsPressed(&Joystick1);

        OLED_ShowString(&oled1, 0, 0, "X", OLED_FONT_SIZE_12, OlED_COLOR_REVERSED);
        OLED_ShowNumber(&oled1, 16, 0, xValue, 4, OLED_FONT_SIZE_12, OLED_COLOR_NORMAL);

        OLED_ShowString(&oled1, 60, 0, "Y", OLED_FONT_SIZE_12, OlED_COLOR_REVERSED);
        OLED_ShowNumber(&oled1, 76, 0, yValue, 4, OLED_FONT_SIZE_12, OLED_COLOR_NORMAL);

        OLED_ShowString(&oled1, 0, 50, "IsPressed", OLED_FONT_SIZE_12, OlED_COLOR_REVERSED);
        OLED_ShowNumber(&oled1, 100, 50, Joystick1IsPressed, 2, OLED_FONT_SIZE_12, OLED_COLOR_NORMAL);

        OLED_RefreshScreen(&oled1);

        Delay_us(10);
    }
}
