#ifndef BMP280_HPP
#define BMP280_HPP

#include "bmp280_register_map.hpp"
#include "spi_device.hpp"
#include "utility.hpp"
#include <cstdint>
#include <optional>
#include <utility>

namespace BMP280 {

    struct BMP280 {
    public:
        enum struct Resolution {
            ULTRALOWPOWER = 1,
            LOWPOWER = 2,
            STANDARD = 3,
            HIGHRES = 4,
            ULTRAHIGHRES = 5,
        };

        enum struct Temperature {
            TEMPERATURE_16BIT = 1,
            TEMPERATURE_17BIT = 2,
            TEMPERATURE_18BIT = 3,
            TEMPERATURE_19BIT = 4,
            TEMPERATURE_20BIT = 5,
        };

        enum struct Mode : std::uint8_t {
            SLEEP = 0,
            FORCED = 1,
            NORMAL = 3,
        };

        enum struct Standby : std::uint8_t {
            MS_0_5 = 0,
            MS_10 = 6,
            MS_20 = 7,
            MS_62_5 = 1,
            MS_125 = 2,
            MS_250 = 3,
            MS_500 = 4,
            MS_1000 = 5
        };

        enum struct Filter : std::uint8_t {
            FILTER_OFF = 0,
            FILTER_X2 = 1,
            FILTER_X4 = 2,
            FILTER_X8 = 3,
            FILTER_X16 = 4,
        };

        enum struct Digit : std::uint8_t {
            DIG_T1 = 0x88,
            DIG_T2 = 0x8A,
            DIG_T3 = 0x8C,
            DIG_P1 = 0x8E,
            DIG_P2 = 0x90,
            DIG_P3 = 0x92,
            DIG_P4 = 0x94,
            DIG_P5 = 0x96,
            DIG_P6 = 0x98,
            DIG_P7 = 0x9A,
            DIG_P8 = 0x9C,
            DIG_P9 = 0x9E,
        };

        using Temp = float;
        using Press = std::int32_t;
        using Alt = float;
        using SPIDevice = Utility::SPIDevice;
        using OptionalTemp = std::optional<Temp>;
        using OptionalPress = std::optional<Press>;
        using OptionalAlt = std::optional<Alt>;

        BMP280() noexcept = default;
        BMP280(SPIDevice&& spi_device,
               Mode const mode,
               Resolution const temp_resolution,
               Resolution const press_resolution,
               Filter const filter) noexcept;

        BMP280(BMP280 const& other) noexcept = delete;
        BMP280(BMP280&& other) noexcept = default;

        BMP280& operator=(BMP280 const& other) noexcept = delete;
        BMP280& operator=(BMP280&& other) noexcept = default;

        ~BMP280() noexcept;

        OptionalTemp get_temperature() noexcept;
        OptionalPress get_pressure() noexcept;
        OptionalAlt get_altitude(Press const sea_level_pa) noexcept;

    private:
        static constexpr std::uint8_t MEASURING{1 << 3}; // Conversion in progress
        static constexpr std::uint8_t DEVICE_ID{0x75};

        void initialize(Mode const mode,
                        Resolution const temp_resolution,
                        Resolution const press_resolution,
                        Filter const filter) noexcept;
        void deinitialize() noexcept;

        void set_config(Filter const filter) const noexcept;
        void set_control(Mode const mode,
                         Resolution const temp_resolution,
                         Resolution const press_resolution) const noexcept;
        void read_digits() noexcept;
        void set_mode(Mode const mode) const noexcept;
        Mode get_mode() const noexcept;
        std::uint8_t get_device_id() const noexcept;
        bool is_valid_device_id() const noexcept;
        void device_reset() const noexcept;

        void set_calib_register(std::uint8_t const num, CALIB const calib) const noexcept;
        void set_reset_register(RESET const reset) const noexcept;
        void set_config_register(CONFIG const config) const noexcept;
        void set_ctrl_meas_register(CTRL_MEAS const ctrl_meas) const noexcept;

        CALIB get_calib_register(std::uint8_t const num) const noexcept;
        ID get_id_register() const noexcept;
        STATUS get_status_register() const noexcept;
        CTRL_MEAS get_ctrl_meas_register() const noexcept;
        PRESS get_press_registers() const noexcept;
        PRESS_X get_press_x_register() const noexcept;
        TEMP get_temp_registers() const noexcept;
        TEMP_X get_temp_x_register() const noexcept;

        bool initialized_{false};

        std::int16_t t2_{};
        std::int16_t t3_{};
        std::int16_t p2_{};
        std::int16_t p3_{};
        std::int16_t p4_{};
        std::int16_t p5_{};
        std::int16_t p6_{};
        std::int16_t p7_{};
        std::int16_t p8_{};
        std::int16_t p9_{};
        std::uint16_t t1_{};
        std::int16_t p1_{};
        std::int32_t t_fine_{};

        SPIDevice spi_device_{};
    };

}; // namespace BMP280

#endif // BMP280_HPP