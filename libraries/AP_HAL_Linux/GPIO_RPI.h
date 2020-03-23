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

    /**
     * @brief Open memory device to allow gpio address access
     *  Should be used before get_memory_pointer calls in the initialization
     *
     * @return true
     * @return false
     */
    bool openMemoryDevice();

    /**
     * @brief Close open memory device
     *
     */
    void closeMemoryDevice();

    //TODO: check if volatile is necessary
    /**
     * @brief Return pointer to memory location with specific range access
     *
     * @param address
     * @param range
     * @return volatile uint32_t*
     */
    volatile uint32_t* get_memory_pointer(uint32_t address, uint32_t range) const;

    /**
     * @brief Get memory address based in base address and peripheral offset
     *
     * @param address
     * @param offset
     * @return constexpr uint32_t
     */
    constexpr uint32_t get_address(GPIO_RPI::Address address, GPIO_RPI::PeripheralOffset offset) const;

    /**
     * @brief Change functionality of GPIO Function Select Registers (GPFSELn) to input.
     * Each GPIO pin is mapped to 3 bits inside a 32 bits register, E.g:
     *
     * 0b00...'010'101
     *   ││    │││ ││└── N pin, 1st bit, LSBit
     *   ││    │││ │└─── N pin, 2nd bit
     *   ││    │││ └──── N pin, 3rd bit, MSBit
     *   ││    ││└────── (N+1) pin, 1st bit, LSBit
     *   ││    │└─────── (N+1) pin, 2nd bit,
     *   ││    └──────── (N+1) pin, 3rd bit, MSBit
     *   ││  ...
     *   │└───────────── Reserved
     *   └────────────── Reserved
     *
     * And the value of this 3 bits selects the functionality of the GPIO pin, E.g:
     *  000 = GPIO Pin N is an input
     *  001 = GPIO Pin N is an output
     *  100 = GPIO Pin N takes alternate function 0
     *  101 = GPIO Pin N takes alternate function 1
     *  110 = GPIO Pin N takes alternate function 2
     *  111 = GPIO Pin N takes alternate function 3
     *  011 = GPIO Pin N takes alternate function 4
     *  010 = GPIO Pin N takes alternate function 5

     * @param pin
     */
    void GPIO_RPI::set_gpio_mode_in(int pin);

    // Memory pointer to clock manager register
    volatile uint32_t* _clock_manager;
    // Memory pointer to gpio registers
    volatile uint32_t* _gpio;
    // Path to memory device (E.g: /dev/mem)
    static const char* _system_memory_device_path;
    // File descriptor for the memory device file
    // If it's negative, then there was an error opening the file.
    int _system_memory_device;
};

}
