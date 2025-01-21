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

        SPIDevice& operator=(SPIDevice const& other) noexcept = delete;
        SPIDevice& operator=(SPIDevice&& other) noexcept = default;

        ~SPIDevice() noexcept = default;

        template <std::size_t TRANSMIT_SIZE>
        void transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept;

        void transmit_dword(DWord const transmit_data) const noexcept;

        template <std::size_t TRANSMIT_SIZE>
        void transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept;

        void transmit_word(Word const transmit_data) const noexcept;

        template <std::size_t TRANSMIT_SIZE>
        void transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept;

        void transmit_byte(Byte const transmit_data) const noexcept;

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] DWords<RECEIVE_SIZE> receive_dwords() const noexcept;

        [[nodiscard]] DWord receive_dword() const noexcept;

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Words<RECEIVE_SIZE> receive_words() const noexcept;

        [[nodiscard]] Word receive_word() const noexcept;

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Bytes<RECEIVE_SIZE> receive_bytes() const noexcept;

        [[nodiscard]] Byte receive_byte() const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] DWord read_dword(std::uint8_t const reg_address) const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] Words<READ_SIZE> read_words(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] Word read_word(std::uint8_t const reg_address) const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] Bytes<READ_SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] Byte read_byte(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept;

        [[nodiscard]] Byte read_bits(std::uint8_t const reg_address,
                                     std::uint8_t const read_position,
                                     std::size_t const read_size) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept;

        void write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept;

        void write_word(std::uint8_t const reg_address, Word const write_data) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept;

        void write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept;

        void write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept;

        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept;

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
    void SPIDevice::transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept
    {
        this->transmit_bytes(dwords_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    void SPIDevice::transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept
    {
        this->transmit_bytes(words_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    void SPIDevice::transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept
    {
        if (this->initialized_) {
            Bytes<TRANSMIT_SIZE> transmit{transmit_data};
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_SPI_Transmit(this->spi_bus_, transmit.data(), transmit.size(), SPI_TIMEOUT);
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_SET);
        }
    }

    template <std::size_t RECEIVE_SIZE>
    DWords<RECEIVE_SIZE> SPIDevice::receive_dwords() const noexcept
    {
        return bytes_to_dwords(this->receive_bytes<4 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    Words<RECEIVE_SIZE> SPIDevice::receive_words() const noexcept
    {
        return bytes_to_words(this->receive_bytes<2 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    Bytes<RECEIVE_SIZE> SPIDevice::receive_bytes() const noexcept
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
    DWords<READ_SIZE> SPIDevice::read_dwords(std::uint8_t const reg_address) const noexcept
    {
        return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    Words<READ_SIZE> SPIDevice::read_words(std::uint8_t const reg_address) const noexcept
    {
        return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    Bytes<READ_SIZE> SPIDevice::read_bytes(std::uint8_t const reg_address) const noexcept
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
    void SPIDevice::write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
    {
        this->write_bytes(reg_address, dwords_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    void SPIDevice::write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
    {
        this->write_bytes(reg_address, words_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    void SPIDevice::write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
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