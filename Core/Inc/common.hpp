#ifndef COMMON_HPP
#define COMMON_HPP

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_uart.h"

namespace Utility {

    using SPIBusHandle = SPI_HandleTypeDef*;
    using TIMHandle = TIM_HandleTypeDef*;
    using GPIOHandle = GPIO_TypeDef*;
    using UARTHandle = UART_HandleTypeDef*;

}; // namespace Utility

#endif // COMMON_HPP