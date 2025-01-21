#include "main.h"
#include "bmp280.hpp"
#include "gpio.h"
#include "spi.h"
#include "spi_device.hpp"
#include "system_clock.h"
#include "usart.h"
#include <cstdio>

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();

    using BMP280 = BMP280::BMP280;
    using namespace Utility;

    SPIDevice spi_device(&hspi1, BMP280_CS_GPIO_Port, BMP280_CS_Pin);

    BMP280 bmp280{std::move(spi_device),
                  BMP280::Mode::FORCED,
                  BMP280::Resolution::STANDARD,
                  BMP280::Resolution::STANDARD,
                  BMP280::Filter::FILTER_OFF};

    while (true) {
        printf("Temperature: %2f Pressure: %2f, Altitude: %2f",
               bmp280.get_temperature().value(),
               bmp280.get_pressure().value(),
               bmp280.get_altitude(101300.0F).value());
    }

    return 0;
}
