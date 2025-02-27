#ifndef BMP280_HPP
#define BMP280_HPP

#include "bmp280_config.hpp"
#include "bmp280_register.hpp"
#include "spi_device.hpp"
#include "utility.hpp"
#include <cstdint>
#include <optional>
#include <utility>

namespace BMP280 {

    struct BMP280 {
    public:
        using SPIDevice = Utility::SPIDevice;

        BMP280() noexcept = default;
        BMP280(SPIDevice&& spi_device, CTRL_MEAS const ctrl_meas, CONFIG const config) noexcept;

        BMP280(BMP280 const& other) noexcept = delete;
        BMP280(BMP280&& other) noexcept = default;

        BMP280& operator=(BMP280 const& other) noexcept = delete;
        BMP280& operator=(BMP280&& other) noexcept = default;

        ~BMP280() noexcept;

        std::optional<float> get_temperature() noexcept;
        std::optional<std::int32_t> get_pressure() noexcept;
        std::optional<float> get_altitude(std::int32_t const sea_level_pa) noexcept;

    private:
        void initialize(CTRL_MEAS const ctrl_meas, CONFIG const config) noexcept;
        void deinitialize() noexcept;

        void read_digits() noexcept;

        void set_mode(Mode const mode) const noexcept;
        Mode get_mode() const noexcept;

        std::uint8_t get_device_id() const noexcept;
        bool is_valid_device_id() const noexcept;
        void device_reset() const noexcept;

        CALIB get_calib_register(std::uint8_t const num) const noexcept;
        void set_calib_register(std::uint8_t const num, CALIB const calib) const noexcept;

        RESET get_reset_register() const noexcept;
        void set_reset_register(RESET const reset) const noexcept;

        CONFIG get_config_register() const noexcept;
        void set_config_register(CONFIG const config) const noexcept;

        CTRL_MEAS get_ctrl_meas_register() const noexcept;
        void set_ctrl_meas_register(CTRL_MEAS const ctrl_meas) const noexcept;

        ID get_id_register() const noexcept;

        STATUS get_status_register() const noexcept;

        PRESS get_press_registers() const noexcept;
        PRESS_X get_press_x_register() const noexcept;

        TEMP get_temp_registers() const noexcept;
        TEMP_X get_temp_x_register() const noexcept;

        bool initialized_{false};

        std::uint16_t t1_{};
        std::uint16_t t2_{};
        std::uint16_t t3_{};
        std::uint16_t p2_{};
        std::uint16_t p3_{};
        std::uint16_t p4_{};
        std::uint16_t p5_{};
        std::uint16_t p6_{};
        std::uint16_t p7_{};
        std::uint16_t p8_{};
        std::uint16_t p9_{};
        std::uint16_t p1_{};
        std::uint32_t t_fine_{};

        SPIDevice spi_device_{};
    };

}; // namespace BMP280

#endif // BMP280_HPP