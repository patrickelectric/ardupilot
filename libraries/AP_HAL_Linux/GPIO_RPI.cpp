#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "GPIO.h"
#include "Util_RPI.h"

// Raspberry Pi GPIO memory
#define BCM2708_PERI_BASE   0x20000000
#define BCM2709_PERI_BASE   0x3F000000
#define BCM2711_PERI_BASE   0xFE000000

#define BCM_CM_GP0CTL 0x0070
#define BCM_CM_GP1CTL 0x0078
#define BCM_CM_GP2CTL 0x0080

#define BCM_CM_GP0DIV 0x0074
#define BCM_CM_GP1DIV 0x007c
#define BCM_CM_GP2DIV 0x0084

#define CLOCK_MANAGER_BASE(address)  (address + 0x101000)
#define GPIO_BASE(address)  (address + 0x200000)

// GPIO setup. Always use INP_GPIO(x) before OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define GPIO_MODE_IN(g)     *(_gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define GPIO_MODE_OUT(g)    *(_gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_MODE_ALT(g,a)  *(_gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET_HIGH       *(_gpio+7)  // sets   bits which are 1
#define GPIO_SET_LOW        *(_gpio+10) // clears bits which are 1
#define GPIO_GET(g)         (*(_gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
#define GPIO_RPI_MAX_NUMBER_PINS 32

using namespace Linux;

extern const AP_HAL::HAL& hal;

GPIO_RPI::GPIO_RPI()
{
}

void GPIO_RPI::init()
{
    int rpi_version = UtilRPI::from(hal.util)->get_rpi_version();
    uint32_t gpio_address;
    uint32_t clock_manager_address;
    if(rpi_version == 1) {
        gpio_address = GPIO_BASE(BCM2708_PERI_BASE);
        clock_manager_address = CLOCK_MANAGER_BASE(BCM2708_PERI_BASE);
    } else if (rpi_version == 2) {
        gpio_address = GPIO_BASE(BCM2709_PERI_BASE);
        clock_manager_address = CLOCK_MANAGER_BASE(BCM2709_PERI_BASE);
    } else {
        gpio_address = GPIO_BASE(BCM2711_PERI_BASE);
    }

    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    if (mem_fd < 0) {
        AP_HAL::panic("Can't open /dev/mem");
    }

    // mmap GPIO
    void *gpio_map = mmap(
        nullptr,              // Any adddress in our space will do
        0xB4,           // Map length // TODO: check this value
        PROT_READ|PROT_WRITE|PROT_EXEC, // Enable reading & writting to mapped memory
        MAP_SHARED|MAP_LOCKED,           // Shared with other processes
        mem_fd,               // File to map
        gpio_address          // Offset to GPIO peripheral
    );

    if (gpio_map == MAP_FAILED) {
        AP_HAL::panic("Can't open /dev/mem");
    }

    _gpio = (volatile uint32_t *)gpio_map;

    // mmap GPIO
    void *clock_manager_map = mmap(
        nullptr,              // Any adddress in our space will do
        0xA8,           // Map length // TODO: check this value
        PROT_READ|PROT_WRITE|PROT_EXEC, // Enable reading & writting to mapped memory
        MAP_SHARED|MAP_LOCKED,           // Shared with other processes
        mem_fd,               // File to map
        clock_manager_address // Offset to clock manager peripheral
    );

    if (clock_manager_map == MAP_FAILED) {
        AP_HAL::panic("Can't open /dev/mem");
    }

    _clock_manager = (volatile uint32_t *)clock_manager_map;

    close(mem_fd); // No need to keep mem_fd open after mmap

    gpclk(4, 25000000);
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output)
{
    if (output == HAL_GPIO_INPUT) {
        GPIO_MODE_IN(pin);
    } else {
        GPIO_MODE_IN(pin);
        GPIO_MODE_OUT(pin);
    }
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    if (output == HAL_GPIO_INPUT) {
        GPIO_MODE_IN(pin);
    } else if (output == HAL_GPIO_ALT) {
        GPIO_MODE_IN(pin);
        GPIO_MODE_ALT(pin, alt);
    } else {
        GPIO_MODE_IN(pin);
        GPIO_MODE_OUT(pin);
    }
}

uint8_t GPIO_RPI::read(uint8_t pin)
{
    if (pin >= GPIO_RPI_MAX_NUMBER_PINS) {
        return 0;
    }
    uint32_t value = GPIO_GET(pin);
    return value ? 1: 0;
}

void GPIO_RPI::write(uint8_t pin, uint8_t value)
{
    if (value == LOW) {
        GPIO_SET_LOW = 1 << pin;
    } else {
        GPIO_SET_HIGH = 1 << pin;
    }
}

void GPIO_RPI::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

#include <iostream>

void GPIO_RPI::gpclk(uint8_t pin, uint32_t frequency)
{
    std::cout << "CONFIGURINGGGGGG\n";

    //TODO: use atari method
    uint32_t BUSY_BIT = 0b01 << 7;
    uint32_t PASSWORD = 0x005a << 24;
    uint32_t ENAB = 0b01 << 4;
    uint32_t OSCILLATOR = 5;
    if (_clock_manager[BCM_CM_GP0CTL / 4] & BUSY_BIT) {
        std::cout << "BUSY BIT IS SET!!!!!!!!!!!!!!!!!!!\n";
        _clock_manager[BCM_CM_GP0CTL / 4] = PASSWORD | 1 << 5; // reset

        usleep(10);
        if(_clock_manager[BCM_CM_GP0CTL / 4] & BUSY_BIT) {
            std::cout << "NOOOOO!\n";
            return;
        }
    }

    _clock_manager[BCM_CM_GP0CTL / 4] = PASSWORD | 1 << 5;
    while(_clock_manager[BCM_CM_GP0CTL / 4] & BUSY_BIT) {
        std::cout << ".";
        usleep(100);
    }

    // 18Mhz clock
    // Configure divisor
    _clock_manager[BCM_CM_GP0DIV / 4] = PASSWORD | 80 << 12 | 0;
    std::cout << "DIV:" << std::hex << _clock_manager[BCM_CM_GP0DIV / 4] << '\n';

    while(_clock_manager[BCM_CM_GP0CTL / 4] & BUSY_BIT) {
        std::cout << ".";
        usleep(10);
    }

    // Configure oscillator
    _clock_manager[BCM_CM_GP0CTL / 4] = (PASSWORD | OSCILLATOR);
    usleep(10);

    std::cout << "OSC:" << std::hex << _clock_manager[BCM_CM_GP0CTL / 4] << _clock_manager[BCM_CM_GP0CTL / 4] << "=" << (PASSWORD | OSCILLATOR)  <<'\n';

    while(_clock_manager[BCM_CM_GP0CTL / 4] & BUSY_BIT) {
        std::cout << "1";
        usleep(10);
    }

    // Enable
    _clock_manager[BCM_CM_GP0CTL / 4] |= PASSWORD | ENAB;
    std::cout << "OSC:" << std::hex << _clock_manager[BCM_CM_GP0CTL / 4] << '\n';

    pinMode(4, HAL_GPIO_ALT, 0);


    std::cout << "Done!!!!!!!!\n";
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_RPI::channel(uint16_t n)
{
    return new DigitalSource(n);
}

bool GPIO_RPI::usb_connected(void)
{
    return false;
}

#endif
