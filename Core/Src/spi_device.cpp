#include "spi_device.hpp"
#include "common.hpp"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_spi.h"

namespace Utility {

    SPIDevice::SPIDevice(SPIBusHandle const spi_bus, GPIOHandle const gpio, std::uint16_t const chip_select) noexcept :
        chip_select_{chip_select}, spi_bus_{spi_bus}, gpio_{gpio}
    {
        this->initialize();
    }

    auto SPIDevice::transmit_dword(DWord const transmit_data) const noexcept -> void
    {
        this->transmit_dwords(DWords<1UL>{transmit_data});
    }

    auto SPIDevice::transmit_word(Word const transmit_data) const noexcept -> void
    {
        this->transmit_words(Words<1UL>{transmit_data});
    }

    auto SPIDevice::transmit_byte(Byte const transmit_data) const noexcept -> void
    {
        this->transmit_bytes(Bytes<1UL>{transmit_data});
    }

    auto SPIDevice::receive_dword() const noexcept -> DWord
    {
        return this->receive_dwords<1UL>()[0];
    }

    auto SPIDevice::receive_word() const noexcept -> Word
    {
        return this->receive_words<1UL>()[0];
    }

    auto SPIDevice::receive_byte() const noexcept -> Byte
    {
        return this->receive_bytes<1UL>()[0];
    }

    auto SPIDevice::read_dword(std::uint8_t const reg_address) const noexcept -> DWord
    {
        return this->read_dwords<1UL>(reg_address)[0];
    }

    auto SPIDevice::read_word(std::uint8_t const reg_address) const noexcept -> Word
    {
        return this->read_words<1UL>(reg_address)[0];
    }

    auto SPIDevice::read_byte(std::uint8_t const reg_address) const noexcept -> Byte
    {
        return this->read_bytes<1UL>(reg_address)[0];
    }

    auto SPIDevice::read_bits(std::uint8_t const reg_address,
                              std::uint8_t const read_position,
                              std::size_t const read_size) const noexcept -> Byte
    {
        return get_bits(this->read_byte(reg_address), read_size, read_position);
    }

    auto SPIDevice::read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept -> Bit
    {
        return get_bit(this->read_byte(reg_address), read_position);
    }

    auto SPIDevice::write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept -> void
    {
        this->write_dwords(reg_address, DWords<1UL>{write_data});
    }

    auto SPIDevice::write_word(std::uint8_t const reg_address, Word const write_data) const noexcept -> void
    {
        this->write_words(reg_address, Words<1UL>{write_data});
    }

    auto SPIDevice::write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept -> void
    {
        this->write_bytes(reg_address, Bytes<1UL>{write_data});
    }

    auto SPIDevice::write_bits(std::uint8_t const reg_address,
                               Byte const write_data,
                               std::uint8_t const write_position,
                               std::size_t const write_size) const noexcept -> void
    {
        Byte write{this->read_byte(reg_address)};
        set_bits(write, write_data, write_size, write_position);
        this->write_byte(reg_address, write);
    }

    auto SPIDevice::write_bit(std::uint8_t const reg_address,
                              Bit const write_data,
                              std::uint8_t const write_position) const noexcept -> void
    {
        Byte write{this->read_byte(reg_address)};
        set_bit(write, write_data, write_position);
        this->write_byte(reg_address, write);
    }

    auto SPIDevice::initialize() noexcept -> void
    {
        if (this->spi_bus_ != nullptr && this->gpio_ != nullptr) {
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_SET);
            this->initialized_ = true;
        }
    }

    auto SPIDevice::deinitialize() noexcept -> void
    {
        if (this->spi_bus_ != nullptr && this->gpio_ != nullptr) {
            HAL_GPIO_WritePin(this->gpio_, this->chip_select_, GPIO_PinState::GPIO_PIN_RESET);
            this->initialized_ = false;
        }
    }

}; // namespace Utility