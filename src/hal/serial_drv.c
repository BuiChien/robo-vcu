#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "serial_drv.h"
#include "common.h"
UART_HandleTypeDef hSerial;

#define SERIAL_TX_PIN                   GPIO_PIN_9
#define SERIAL_RX_PIN                   GPIO_PIN_10
#define SERIAL_TX_PORT                  GPIOA
#define SERIAL_RX_PORT                  GPIOA
#define SERIAL_CLK_GPIO_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define SERIAL_CHANNEL                  USART1
#define SERIAL_CLK_ENABLE               __HAL_RCC_USART1_CLK_ENABLE
#define SERIAL_IRQn                     USART1_IRQn
#define SerialDrv_ISRHandler            USART1_IRQHandler
QueueHandle_t hReceiveQueue;
SerialDrv_DataTypeDef sBuffer;
void SerialDrv_ISRHandler()
{
    uint32_t isrflags   = READ_REG(hSerial.Instance->SR);
    uint32_t cr1its     = READ_REG(hSerial.Instance->CR1);
    uint32_t errorflags;
    uint8_t data = 0;
    /* If no error occurs */
    errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    if (errorflags == RESET)
    {
        /* UART in mode Receiver */
        if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        {
            data = (uint8_t)(hSerial.Instance->DR & 0x1FF);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (data == '\n')
            {
                sBuffer.data[sBuffer.iLength] = 0;
                if (xQueueSendFromISR(hReceiveQueue, &sBuffer, &xHigherPriorityTaskWoken) == pdPASS) 
                {
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
                else
                {
                    Common_Assert(__LINE__, __FILE__);
                }
                sBuffer.iLength = 0;
            }
            else
            {
                sBuffer.data[sBuffer.iLength] = data;
                sBuffer.iLength++;
                if (sBuffer.iLength >= RECEIVE_CMD_BUFFER_SIZE)
                {
                    sBuffer.iLength = 0;
                }
            }
        }
    }
    else
    {
        Common_Assert(__LINE__, __FILE__);
    }
}

void SerialDrv_Init()
{
    GPIO_InitTypeDef  sGpio;
    /* Enable GPIO TX/RX clock */
    SERIAL_CLK_GPIO_ENABLE();
    /* Enable USART_CONSOLE clock */
    SERIAL_CLK_ENABLE();
    sGpio.Pin       = SERIAL_TX_PIN;
    sGpio.Mode      = GPIO_MODE_AF_PP;
    sGpio.Pull      = GPIO_PULLUP;
    sGpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SERIAL_TX_PORT, &sGpio);
    sGpio.Pin       = SERIAL_RX_PIN;
    sGpio.Mode      = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SERIAL_RX_PORT, &sGpio);
    /* UART configured as follows:
            - Word Length = 8 Bits
            - Stop Bit = One Stop bit
            - Parity = None
            - BaudRate = 115200 baud
            - Hardware flow control disabled (RTS and CTS signals) */
    hSerial.Instance        = SERIAL_CHANNEL;
    hSerial.Init.BaudRate   = 115200;
    hSerial.Init.WordLength = UART_WORDLENGTH_8B;
    hSerial.Init.StopBits   = UART_STOPBITS_1;
    hSerial.Init.Parity     = UART_PARITY_NONE;
    hSerial.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    hSerial.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_DeInit(&hSerial);
    if (HAL_UART_Init(&hSerial) == HAL_OK) 
    {
        __HAL_UART_ENABLE_IT(&hSerial, UART_IT_RXNE);
        HAL_NVIC_SetPriority(SERIAL_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(SERIAL_IRQn);
    } 
    else
    {
        Common_Assert(__LINE__, __FILE__);
    }
    hReceiveQueue = xQueueCreate(RECEIVE_CMD_QUEUE_SIZE, RECEIVE_CMD_BUFFER_SIZE);
}

size_t SerialDrv_Send(char *buffer, uint16_t length, int timeout)
{
    size_t count = length;
    if (HAL_UART_Transmit(&hSerial, (uint8_t*)buffer, length, timeout) != HAL_OK)
    {
        count = 0;
    }
    return count;
}

bool SerialDrv_Receive(SerialDrv_DataTypeDef *buffer, int timeout)
{
    bool bRet = false;
    if (buffer != NULL) {
        TickType_t ticks = pdMS_TO_TICKS(timeout);
        bRet = xQueueReceive(hReceiveQueue, buffer, ticks) == pdPASS;
    }
    return bRet;
}
