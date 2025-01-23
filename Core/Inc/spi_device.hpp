#ifndef SPI_DEVICE_HPP
#define SPI_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_spi.h"
#include "utility.hpp"
#include <algorithm>
#include <array>
#include <bitset>
#include <cstdint>
#include <ranges>
#include <utility>

namespace Utility {

    struct SPIDevice {
    public:
        SPIDevice() noexcept = default;
        SPIDevice(SPIBusHandle const spi_bus, GPIOHandle const gpio, std::uint16_t const chip_select) noexcept;

        SPIDevice(SPIDevice const& other) noexcept = delete;
        SPIDevice(SPIDevice&& other) noexcept = default;

        auto operator=(SPIDevice const& other) noexcept -> SPIDevice& = delete;
        auto operator=(SPIDevice&& other) noexcept -> SPIDevice& = default;

        ~SPIDevice() noexcept = default;

        template <std::size_t TRANSMIT_SIZE>
        auto transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void;
        auto transmit_dword(DWord const transmit_data) const noexcept -> void;

        template <std::size_t TRANSMIT_SIZE>
        auto transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void;
        auto transmit_word(Word const transmit_data) const noexcept -> void;

        template <std::size_t TRANSMIT_SIZE>
        auto transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void;
        auto transmit_byte(Byte const transmit_data) const noexcept -> void;

        template <std::size_t RECEIVE_SIZE>
        auto receive_dwords() const noexcept -> [[nodiscard]] DWords<RECEIVE_SIZE>;
        auto receive_dword() const noexcept -> [[nodiscard]] DWord;

        template <std::size_t RECEIVE_SIZE>
        auto receive_words() const noexcept -> [[nodiscard]] Words<RECEIVE_SIZE>;
        auto receive_word() const noexcept -> [[nodiscard]] Word;

        template <std::size_t RECEIVE_SIZE>
        auto receive_bytes() const noexcept -> [[nodiscard]] Bytes<RECEIVE_SIZE>;
        auto receive_byte() const noexcept -> [[nodiscard]] Byte;

        template <std::size_t READ_SIZE>
        auto read_dwords(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] DWords<READ_SIZE>;
        auto read_dword(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] DWord;

        template <std::size_t READ_SIZE>
        auto read_words(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Words<READ_SIZE>;
        auto read_word(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Word;

        template <std::size_t READ_SIZE>
        auto read_bytes(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Bytes<READ_SIZE>;
        auto read_byte(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Byte;

        auto read_bits(std::uint8_t const reg_address,
                       std::uint8_t const read_position,
                       std::size_t const read_size) const noexcept -> [[nodiscard]] Byte;
        auto read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept ->
            [[nodiscard]] Bit;

        template <std::size_t WRITE_SIZE>
        auto write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept -> void;
        auto write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept -> void;

        template <std::size_t WRITE_SIZE>
        auto write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept -> void;
        auto write_word(std::uint8_t const reg_address, Word const write_data) const noexcept -> void;

        template <std::size_t WRITE_SIZE>
        auto write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept -> void;
        auto write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept -> void;

        auto write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept -> void;
        auto write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept -> void;

    private:
        static constexpr std::uint32_t SPI_TIMEOUT{100U};

        void initialize() noexcept;
        void deinitialize() noexcept;

        bool initialized_{false};

        std::uint16_t chip_select_{};

        SPIBusHandle spi_bus_{nullptr};
        GPIOHandle gpio_{nullptr};
    };

    template <std::size_t TRANSMIT_SIZE>
    auto SPIDevice::transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void
    {
        this->transmit_bytes(dwords_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    auto SPIDevice::transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void
    {
        this->transmit_bytes(words_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    auto SPIDevice::transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void
    {
        if (this->initialized_) {
            Bytes<TRANSMIT_SIZE> transmit{transmit_data};
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_SPI_Transmit(this->spi_bus_, transmit.data(), transmit.size(), SPI_TIMEOUT);
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_SET);
        }
    }

    template <std::size_t RECEIVE_SIZE>
    auto SPIDevice::receive_dwords() const noexcept -> DWords<RECEIVE_SIZE>
    {
        return bytes_to_dwords(this->receive_bytes<4 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    auto SPIDevice::receive_words() const noexcept -> Words<RECEIVE_SIZE>
    {
        return bytes_to_words(this->receive_bytes<2 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    auto SPIDevice::receive_bytes() const noexcept -> Bytes<RECEIVE_SIZE>
    {
        if (this->initialized_) {
            Bytes<RECEIVE_SIZE> receive{};
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_SPI_Receive(this->spi_bus_, receive.data(), receive.size(), SPI_TIMEOUT);
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_SET);
            return receive;
        }
        std::unreachable();
    }

    template <std::size_t READ_SIZE>
    auto SPIDevice::read_dwords(std::uint8_t const reg_address) const noexcept -> DWords<READ_SIZE>
    {
        return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    auto SPIDevice::read_words(std::uint8_t const reg_address) const noexcept -> Words<READ_SIZE>
    {
        return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    auto SPIDevice::read_bytes(std::uint8_t const reg_address) const noexcept -> Bytes<READ_SIZE>
    {
        if (this->initialized_) {
            Bytes<READ_SIZE> read{};
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(this->spi_bus_, &reg_address, read.data(), read.size(), SPI_TIMEOUT);
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_SET);
            return read;
        }
        std::unreachable();
    }

    template <std::size_t WRITE_SIZE>
    auto SPIDevice::write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
        -> void
    {
        this->write_bytes(reg_address, dwords_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    auto SPIDevice::write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
        -> void
    {
        this->write_bytes(reg_address, words_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    auto SPIDevice::write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
        -> void
    {
        if (this->initialized_) {
            Bytes<1 + WRITE_SIZE> write{reg_address};
            std::ranges::copy(write_data, write.data() + 1);
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_SPI_Transmit(this->spi_bus_, write.data(), write.size(), SPI_TIMEOUT);
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_SET);
        }
    }

}; // namespace Utility

#endif // SPI_DEVICE_HPP