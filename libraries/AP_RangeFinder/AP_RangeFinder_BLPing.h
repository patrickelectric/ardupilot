#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_BLPing : public AP_RangeFinder_Backend
{

public:

    // constructor
    AP_RangeFinder_BLPing(RangeFinder::RangeFinder_State &_state,
                          AP_RangeFinder_Params &_params,
                          AP_SerialManager &serial_manager,
                          uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    void init_sensor();

    /**
     * @brief Send message with a specific payload
     *
     * @param msgid
     * @param payload
     * @param payload_len
     */
    void send_message(uint16_t msgid, const uint8_t *payload, uint16_t payload_len);

    /**
     * @brief Send set ping interval message
     *
     * @param interval
     */
    void set_ping_interval(uint16_t interval);

    /**
     * @brief Request a specific message ID
     *
     * @param msg_id
     */
    void request_message(uint16_t msg_id);

    /**
     * @brief Check for new available messages
     *
     */
    bool check_new_messages();

    /**
     * @brief Process available message in msg structure
     *
     */
    void process_available_message();

    /**
     * @brief Process one byte received on serial port
     *  returns the message ID that was correctly parsed
     *  state is stored in msg structure
     *
     * @param b
     * @return uint16_t
     */
    uint16_t parse_byte(uint8_t b);

    enum class ParseState {
        HEADER1 = 0,
        HEADER2,
        LEN_L,
        LEN_H,
        MSG_ID_L,
        MSG_ID_H,
        SRC_ID,
        DST_ID,
        PAYLOAD,
        CRC_L,
        CRC_H
    };

    AP_HAL::UARTDriver *uart;
    uint32_t last_init_ms;      // system time that sensor was last initialised
    uint16_t distance_cm;       // latest distance
    uint16_t ping_interval;     // sensor value for ping_interval, interval between pings
    bool sensor_initialized;    // variable to check for sensor initial configuration

    // structure holding latest message contents
    struct {
        ParseState state;       // state of incoming message processing
        uint8_t payload[40];    // payload
        uint16_t payload_len;   // latest message payload length
        uint16_t msgid;         // latest message's message id
        uint16_t payload_recv;  // number of message's payload bytes received so far
        uint16_t crc;           // latest message's crc
        uint16_t crc_expected;  // latest message's expected crc
    } msg;
};
