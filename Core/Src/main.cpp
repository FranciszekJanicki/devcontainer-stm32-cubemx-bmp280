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

    using namespace BMP280;

    SPIDevice spi_device(&hspi1, BMP280_CS_GPIO_Port, BMP280_CS_Pin);

    CTRL_MEAS ctrl_meas{.osrs_t = std::to_underlying(Resolution::STANDARD),
                        .osrs_p = std::to_underlying(Resolution::STANDARD),
                        .mode = std::to_underlying(Mode::NORMAL)};

    CONFIG config{.spi3w_en = false, .filter = std::to_underlying(Filter::FILTER_OFF), .t_sb = false};

    BMP280::BMP280 bmp280{std::move(spi_device), ctrl_meas, config};

    while (true) {
        printf("Temperature: %f Pressure: %d, Altitude: %d",
               bmp280.get_temperature().value(),
               bmp280.get_pressure().value(),
               bmp280.get_altitude(101300.0F).value());

        HAL_Delay(50);
    }

    return 0;
}
