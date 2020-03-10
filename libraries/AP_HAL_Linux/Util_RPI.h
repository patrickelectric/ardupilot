#pragma once

#include "Util.h"

namespace Linux {

class UtilRPI : public Util {
public:
    UtilRPI();

    static UtilRPI *from(AP_HAL::Util *util) {
        return static_cast<UtilRPI*>(util);
    }

    enum class Version {
        PI_INVALID = 0,
        PI_0_OR_1 = 1,
        PI_2_OR_3 = 2,
        PI_4 = 4,
    };

    /* return the Raspberry Pi version */
    UtilRPI::Version get_rpi_version() const;

protected:
    // Called in the constructor once
    UtilRPI::Version _check_rpi_version();

private:
    UtilRPI::Version _rpi_version = Version::PI_INVALID;
};

}
