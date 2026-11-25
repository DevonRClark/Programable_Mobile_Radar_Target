#pragma once
#include "CartCommon.h"

// A zero-only protocol responder. No sensor, PWM, or motor-control code runs.
namespace probe {
enum class Result { BAD_PACKET, HEARTBEAT_OK, ZERO_OK, MOVEMENT_REFUSED };

struct State {
    uint32_t session = 0;
    uint32_t acknowledged_sequence = 0;
    bool zero_seen = false;

    Result receive(const uint8_t *data, int length) {
        if (!data || length < static_cast<int>(sizeof(cart::Header)))
            return Result::BAD_PACKET;
        cart::Header header;
        memcpy(&header, data, sizeof(header));
        uint32_t incoming_session = 0;
        cart::Command command = {};
        bool is_command = false;
        if (length == sizeof(cart::Heartbeat) &&
            cart::validHeader(header, cart::HEARTBEAT)) {
            cart::Heartbeat heartbeat;
            memcpy(&heartbeat, data, sizeof(heartbeat));
            incoming_session = heartbeat.session;
        } else if (length == sizeof(cart::Command) &&
                   cart::validHeader(header, cart::COMMAND)) {
            memcpy(&command, data, sizeof(command));
            if (!cart::validCommand(command))
                return Result::BAD_PACKET;
            incoming_session = command.session;
            is_command = true;
        } else {
            return Result::BAD_PACKET;
        }
        if (incoming_session == 0) return Result::BAD_PACKET;
        if (incoming_session != session) {
            session = incoming_session;
            acknowledged_sequence = 0;
            zero_seen = false;
        }
        if (!is_command) return Result::HEARTBEAT_OK;
        if (!cart::isStop(command)) {
            // Do not acknowledge movement: this is only a communication test.
            zero_seen = false;
            return Result::MOVEMENT_REFUSED;
        }
        if (acknowledged_sequence == 0 ||
            cart::newerSequence(command.sequence, acknowledged_sequence)) {
            acknowledged_sequence = command.sequence;
            zero_seen = true;
        }
        return Result::ZERO_OK;
    }

    cart::Telemetry telemetry() const {
        cart::Telemetry packet = {};
        packet.header = cart::makeHeader(cart::TELEMETRY);
        packet.session = session;
        packet.acknowledged_sequence = acknowledged_sequence;
        packet.stop_reason = zero_seen ? cart::STOP_NONE : cart::STOP_REARM_REQUIRED;
        packet.flags = zero_seen ? 0 : cart::FLAG_REARM;
        return packet;
    }
};
} // namespace probe
