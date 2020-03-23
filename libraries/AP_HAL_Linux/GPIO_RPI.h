#pragma once

#include <stdint.h>
#include "AP_HAL_Linux.h"

#define LOW                 0
#define HIGH                1

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

/**
 * @brief Check for valid Raspberry Pi pin range
 *
 * @tparam pin
 * @return uint8_t
 */
template <uint8_t pin> constexpr uint8_t RPI_GPIO_()
{
    static_assert(pin > 1 && pin < 32, "Invalid pin value.");
    return pin;
}

namespace Linux {

class GPIO_RPI : public AP_HAL::GPIO {
public:
    GPIO_RPI();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;
    void    gpclk(uint8_t pin, uint32_t frequency) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;

private:
    // Raspberry Pi BASE memory address
    enum class Address : uint32_t {
        BCM2708_PERIPHERAL_BASE = 0x20000000, // Raspberry Pi 0/1
        BCM2709_PERIPHERAL_BASE = 0x3F000000, // Raspberry Pi 2/3
        BCM2711_PERIPHERAL_BASE = 0xFE000000, // Raspberry Pi 4
    };

    // Offset between peripheral base address
    enum class PeripheralOffset : uint32_t {
        CLOCK_MANAGER = 0x101000,
        GPIO          = 0x200000,
    };

    // Offset between registers and clock manager offset
    enum class ClockManager : uint32_t {
        GP0CTL = 0x70,
        GP1CTL = 0x78,
        GP2CTL = 0x80,

        GP0DIV = 0x74,
        GP1DIV = 0x7C,
        GP2DIV = 0x84,
    };

    bool openMemoryDevice();
    void closeMemoryDevice();
    constexpr uint32_t gpio_address() const;
    //TODO: check if volatile is necessary
    volatile uint32_t* get_memory_pointer(uint32_t address, uint32_t range) const;
    //constexpr uint32_t get_address(GPIO_RPI::Address address, GPIO_RPI::PeripheralOffset offset) const
    constexpr uint32_t get_address(GPIO_RPI::Address address, GPIO_RPI::PeripheralOffset offset) const;

    volatile uint32_t *_clock_manager;
    volatile uint32_t *_gpio;

    static const char* _system_memory_device_path;
    int _system_memory_device;
};

}
