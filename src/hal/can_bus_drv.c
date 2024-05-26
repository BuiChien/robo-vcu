#include <FreeRTOS.h>
#include <queue.h>
#include "common.h"
#include "can_bus_drv.h"

CAN_HandleTypeDef hcan;
static CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
static uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t txMailbox;
QueueHandle_t hCanRcvQueue;
typedef struct {
  uint32_t baudrate;
  uint16_t prescaler;
  uint8_t time_quanta;
  uint8_t timeseg1;
  uint8_t timeseg2;
} CanBusDrv_BaudrateEntryTypeDef;

static const CanBusDrv_BaudrateEntryTypeDef BAUD_RATE_TABLE_48M[] = {
    {
    1000000, 3, 16, 13, 2
    },
    {
    800000, 4, 15, 12, 2
    },
    {
    500000, 6, 16, 13, 2
    },
    {
    250000, 12, 16, 13, 2
    },
    {
    125000, 24, 16, 13, 2
    },
    {
    100000, 30, 16, 13, 2
    }
};
	
static const CanBusDrv_BaudrateEntryTypeDef BAUD_RATE_TABLE_45M[] = {
    {
    1000000, 3, 15, 12, 2
    },
    {
    500000, 5, 18, 15, 2
    },
    {
    250000, 10, 18, 15, 2
    },
    {
    125000, 20, 18, 15, 2
    },
    {
    100000, 25, 18, 15, 2
    }
};

static void CanBusDrv_calculateBaudrate(CAN_HandleTypeDef *CanHandle, int baud);
static uint32_t CanBusDrv_getAPB1Clock();
static void CanBusDrv_setBaudRateValues(CAN_HandleTypeDef *CanHandle, uint16_t prescaler, uint8_t timeseg1,
                                                                uint8_t timeseg2, uint8_t sjw);

void CanBusDrv_Init() 
{
    GPIO_InitTypeDef sGpio = {0};
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
     PA11     ------> CAN_RX
     PA12     ------> CAN_TX
    */
    sGpio.Pin = GPIO_PIN_11;
    sGpio.Mode = GPIO_MODE_INPUT;
    sGpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &sGpio);

    sGpio.Pin = GPIO_PIN_12;
    sGpio.Mode = GPIO_MODE_AF_PP;
    sGpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &sGpio);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    hcan.Instance = CAN1;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    // Calculate and set baudrate
    CanBusDrv_calculateBaudrate( &hcan, 500000);

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Common_Assert(__LINE__, __FILE__);
    }
    canfil.FilterActivation = CAN_FILTER_ENABLE;
    canfil.FilterBank = 10;  // which filter bank to use from the assigned ones
    canfil.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfil.FilterIdHigh = 0;
    canfil.FilterIdLow = 0;
    canfil.FilterMaskIdHigh = 0;
    canfil.FilterMaskIdLow = 0x0000;
    canfil.FilterMode = CAN_FILTERMODE_IDMASK;
    canfil.FilterScale = CAN_FILTERSCALE_32BIT;
    canfil.SlaveStartFilterBank = 0;  // doesn't matter in single can controllers

    HAL_CAN_ConfigFilter(&hcan,&canfil);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
    hCanRcvQueue = xQueueCreate(RECEIVE_CMD_QUEUE_SIZE, RECEIVE_CMD_BUFFER_SIZE);
}

bool CanBusDrv_Receive(uint8_t *buffer, int timeout)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout);
    return xQueueReceive(hCanRcvQueue, buffer, ticks) == pdPASS;
}

void CanBusDrv_Send(uint32_t stdId, const uint8_t *const buffer, const uint8_t len)
{
    static CAN_TxHeaderTypeDef header; //CAN Bus Receive Header
    header.DLC = len;  // data length
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.StdId = stdId;  // ID
    HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)buffer, &txMailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(hCanRcvQueue, &canRX, &xHigherPriorityTaskWoken) == pdPASS) 
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        Common_Assert(__LINE__, __FILE__);
    }
}

void CanBusDrv_calculateBaudrate(CAN_HandleTypeDef *CanHandle, int baud)
{
  /* this function calculates the needed Sync Jump Width, Time segments 1 and 2 and prescaler values based on the set baud rate and APB1 clock.
  This could be done faster if needed by calculating these values beforehand and just using fixed values from table.
  The function has been optimized to give values that have sample-point between 75-94%. If some other sample-point percentage is needed, this needs to be adjusted.
  More info about this topic here: http://www.bittiming.can-wiki.info/
  */
  int sjw = 1;
  int bs1 = 5; // optimization. bs1 smaller than 5 does give too small sample-point percentages.
  int bs2 = 1;
  int prescaler = 1;
  uint16_t i = 0;

  uint32_t frequency = CanBusDrv_getAPB1Clock();

  if(frequency == 48000000) {
    for(i=0; i<sizeof(BAUD_RATE_TABLE_48M)/sizeof(CanBusDrv_BaudrateEntryTypeDef); i++) {
      if(baud == (int)BAUD_RATE_TABLE_48M[i].baudrate) {
        break;
      }
    }
    if(i < sizeof(BAUD_RATE_TABLE_48M)/sizeof(CanBusDrv_BaudrateEntryTypeDef)) {
      CanBusDrv_setBaudRateValues(CanHandle, BAUD_RATE_TABLE_48M[i].prescaler,
                                   BAUD_RATE_TABLE_48M[i].timeseg1,
                                   BAUD_RATE_TABLE_48M[i].timeseg2,
                                   1);
      return;
    }
  }
  else if(frequency == 45000000) {
    for(i=0; i<sizeof(BAUD_RATE_TABLE_45M)/sizeof(CanBusDrv_BaudrateEntryTypeDef); i++) {
      if(baud == (int)BAUD_RATE_TABLE_45M[i].baudrate) {
        break;
      }
    }
    if(i < sizeof(BAUD_RATE_TABLE_45M)/sizeof(CanBusDrv_BaudrateEntryTypeDef)) {
      CanBusDrv_setBaudRateValues(CanHandle, BAUD_RATE_TABLE_45M[i].prescaler,
                                   BAUD_RATE_TABLE_45M[i].timeseg1,
                                   BAUD_RATE_TABLE_45M[i].timeseg2,
                                   1);
      return;
    }
  }

  while (sjw <= 4) {
    while (prescaler <= 1024) {
      while (bs2 <= 3) { // Time segment 2 can get up to 8, but that causes too small sample-point percentages, so this is limited to 3.
        while (bs1 <= 15) { // Time segment 1 can get up to 16, but that causes too big sample-point percenages, so this is limited to 15.
          int calcBaudrate = (int)(frequency / (prescaler * (sjw + bs1 + bs2)));

          if (calcBaudrate == baud)
          {
            CanBusDrv_setBaudRateValues(CanHandle, prescaler, bs1, bs2, sjw);
            return;
          }
          bs1++;
        }
        bs1 = 5;
        bs2++;
      }
      bs1 = 5;
      bs2 = 1;
      prescaler++;
    }
    bs1 = 5;
    sjw++;
  }
}

void CanBusDrv_setBaudRateValues(CAN_HandleTypeDef *CanHandle, uint16_t prescaler, uint8_t timeseg1,
                                                                uint8_t timeseg2, uint8_t sjw)
{
  uint32_t _SyncJumpWidth = 0;
  uint32_t _TimeSeg1 = 0;
  uint32_t _TimeSeg2 = 0;
  uint32_t _Prescaler = 0;
  switch (sjw)
  {
    case 1:
      _SyncJumpWidth = CAN_SJW_1TQ;
      break;
    case 2:
      _SyncJumpWidth = CAN_SJW_2TQ;
      break;
    case 3:
      _SyncJumpWidth = CAN_SJW_3TQ;
      break;
    case 4:
      _SyncJumpWidth = CAN_SJW_4TQ;
      break;
    default:
      // should not happen
      _SyncJumpWidth = CAN_SJW_1TQ;
      break;
  }

  switch (timeseg1)
  {
    case 1:
      _TimeSeg1 = CAN_BS1_1TQ;
      break;
    case 2:
      _TimeSeg1 = CAN_BS1_2TQ;
      break;
    case 3:
      _TimeSeg1 = CAN_BS1_3TQ;
      break;
    case 4:
      _TimeSeg1 = CAN_BS1_4TQ;
      break;
    case 5:
      _TimeSeg1 = CAN_BS1_5TQ;
      break;
    case 6:
      _TimeSeg1 = CAN_BS1_6TQ;
      break;
    case 7:
      _TimeSeg1 = CAN_BS1_7TQ;
      break;
    case 8:
      _TimeSeg1 = CAN_BS1_8TQ;
      break;
    case 9:
      _TimeSeg1 = CAN_BS1_9TQ;
      break;
    case 10:
      _TimeSeg1 = CAN_BS1_10TQ;
      break;
    case 11:
      _TimeSeg1 = CAN_BS1_11TQ;
      break;
    case 12:
      _TimeSeg1 = CAN_BS1_12TQ;
      break;
    case 13:
      _TimeSeg1 = CAN_BS1_13TQ;
      break;
    case 14:
      _TimeSeg1 = CAN_BS1_14TQ;
      break;
    case 15:
      _TimeSeg1 = CAN_BS1_15TQ;
      break;
    case 16:
      _TimeSeg1 = CAN_BS1_16TQ;
      break;
    default:
      // should not happen
      _TimeSeg1 = CAN_BS1_1TQ;
      break;
  }

  switch (timeseg2)
  {
    case 1:
      _TimeSeg2 = CAN_BS2_1TQ;
      break;
    case 2:
      _TimeSeg2 = CAN_BS2_2TQ;
      break;
    case 3:
      _TimeSeg2 = CAN_BS2_3TQ;
      break;
    case 4:
      _TimeSeg2 = CAN_BS2_4TQ;
      break;
    case 5:
      _TimeSeg2 = CAN_BS2_5TQ;
      break;
    case 6:
      _TimeSeg2 = CAN_BS2_6TQ;
      break;
    case 7:
      _TimeSeg2 = CAN_BS2_7TQ;
      break;
    case 8:
      _TimeSeg2 = CAN_BS2_8TQ;
      break;
    default:
      // should not happen
      _TimeSeg2 = CAN_BS2_1TQ;
      break;
  }
  _Prescaler = prescaler;

  CanHandle->Init.SyncJumpWidth = _SyncJumpWidth;
  CanHandle->Init.TimeSeg1 = _TimeSeg1;
  CanHandle->Init.TimeSeg2 = _TimeSeg2;
  CanHandle->Init.Prescaler = _Prescaler;
}

uint32_t CanBusDrv_getAPB1Clock()
{
  RCC_ClkInitTypeDef clkInit;
  uint32_t flashLatency;
  HAL_RCC_GetClockConfig(&clkInit, &flashLatency);

  uint32_t hclkClock = HAL_RCC_GetHCLKFreq();
  uint8_t clockDivider = 1;
  switch (clkInit.APB1CLKDivider)
  {
  case RCC_HCLK_DIV1:
    clockDivider = 1;
    break;
  case RCC_HCLK_DIV2:
    clockDivider = 2;
    break;
  case RCC_HCLK_DIV4:
    clockDivider = 4;
    break;
  case RCC_HCLK_DIV8:
    clockDivider = 8;
    break;
  case RCC_HCLK_DIV16:
    clockDivider = 16;
    break;
  default:
    // should not happen
    break;
  }

  uint32_t apb1Clock = hclkClock / clockDivider;

  return apb1Clock;
}
