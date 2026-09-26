#include "main.h"

#include <hydrolib_bus_datalink_stream.hpp>
#include <hydrolib_bus_application_master.hpp>
#include <hydrv_gpio_low.hpp>
#include <hydrv_rs_485.hpp>

#include "ADS1220.hpp"
#include "pressure_processing.hpp"

constexpr uint32_t kTransmitPeriodMs = 5;

constinit hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 10, hydrv::GPIO::GPIOLow::GPIO_UART_RX);

constinit hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 9, hydrv::GPIO::GPIOLow::GPIO_UART_TX);

constinit hydrv::GPIO::GPIOLow direction_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 8, hydrv::GPIO::GPIOLow::GPIO_Output);

static constexpr hydrv::UART::UARTLow::UARTPreset USART1_115200_LOW{
    USART1_BASE, 7, RCC_APB2ENR_USART1EN, RCC_BASE + offsetof(RCC_TypeDef, APB2ENR), USART1_IRQn, 39, 1
};

constinit hydrv::RS485::RS485<255, 255> rs485(USART1_115200_LOW, rx_pin, tx_pin, direction_pin, 7);

class Logger
{
public:
    Logger() = default;
};

Logger logger;
hydrolib::bus::datalink::StreamManager streamManager(1, rs485, logger);
hydrolib::bus::datalink::Stream busStream(streamManager, 2);
hydrolib::bus::application::Master busMaster(busStream, logger);

CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;
ADS1220 externalAdc{ hspi1, SPI1_NSS_GPIO_Port, SPI1_NSS_Pin };
PressureProcessor pressureProcessor;
uint64_t depth = 0;
uint64_t avg_depth = 0;
uint64_t depth_sum = 0;

constexpr int kDepthBusAddress = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_CAN_Init();

    rs485.Init();
    if (not externalAdc.Init()) {
        Error_Handler();
    }

    HAL_GPIO_WritePin(STM_ALIVE_GPIO_Port, STM_ALIVE_Pin, GPIO_PIN_SET);
    uint32_t lastTransmitTick = HAL_GetTick();

    if (not externalAdc.IsMeasurementInProgress()) {
        (void)externalAdc.StartMeasurement(ADS1220::Channel::Pressure);
    }

    uint8_t k = 0;

    while (1) {
        streamManager.Process();
        const auto measurement = externalAdc.ReadMeasurement();
        if (measurement.has_value()) {
            if (measurement->channel == ADS1220::Channel::Pressure) {

                if (pressureProcessor.Process(measurement->adcValue)) {
                    const auto depthMicrometers = pressureProcessor.DepthMicrometers();
                    if (depthMicrometers.has_value()) {
                        depth = depthMicrometers.value();
                        if (k < 20)
                        {
                            depth_sum += depth;
                            k++;
                        }
                        else
                        {
                            avg_depth = depth_sum / 20;
                            depth_sum = 0;
                            k = 0;
                        }
                        const uint32_t now = HAL_GetTick();
                        if (now - lastTransmitTick >= kTransmitPeriodMs) {
                            lastTransmitTick = now;
                            busMaster.Write(static_cast<void*>(&avg_depth), kDepthBusAddress, sizeof(avg_depth));
                        }
                    }
                }
            }
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct   = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct   = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_CAN_Init(void)
{
    hcan.Instance                  = CAN1;
    hcan.Init.Prescaler            = 16;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_1TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_1TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = DISABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = DISABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, STM_ALIVE_Pin | SPI1_NSS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : STM_ALIVE_Pin SPI1_NSS_Pin */
    GPIO_InitStruct.Pin   = STM_ALIVE_Pin | SPI1_NSS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADS1220 DRDY is active low. */
    GPIO_InitStruct.Pin  = DATA_READY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DATA_READY_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

extern "C" void USART1_IRQHandler(void)
{
    rs485.IRQCallback();
}

extern "C" void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(DATA_READY_Pin);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{
    if (gpioPin == DATA_READY_Pin) {
        externalAdc.OnDataReadyInterrupt();
    }
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
