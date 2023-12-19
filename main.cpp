//#include "asio-1.28.0/include/asio.hpp"
#include "boost/asio.hpp"
#include "c_library_v2-master/common/mavlink.h"
#include <iostream>
#include <array>

boost::asio::io_service io_service;
boost::asio::ip::udp::socket socket_other(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 14551));
boost::asio::ip::udp::endpoint sender_endpoint;
std::array<char, 1024> buffer;

void handle_receive(const std::error_code& error, std::size_t bytes_transferred) {
    if (!error) {
        mavlink_message_t msg;
        mavlink_status_t status;

        for (std::size_t i = 0; i < bytes_transferred; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                        std::cout << "Received Heartbeat: System ID - " << (int)msg.sysid
                            << ", Component ID - " << (int)msg.compid << std::endl;
                        break;
                    case MAVLINK_MSG_ID_SCALED_PRESSURE:
                        mavlink_scaled_pressure_t scaled_pressure;
                        mavlink_msg_scaled_pressure_decode(&msg, &scaled_pressure);

                        float temperature = scaled_pressure.temperature;
                        uint32_t pressure = scaled_pressure.press_abs;

                        std::cout << "Temperature: " << temperature << " C, Pressure: " << pressure << " Pa" << std::endl;
                }
            }
        }
        socket_other.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, handle_receive);
    } else {
        std::cerr << "Error in receive: " << error.message() << std::endl;
    }
}

void sendHeartbeat(const std::error_code& /*error*/, boost::asio::steady_timer* heartbeat_timer) {
    std::cerr << "Send Heartbeat" << std::endl;
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    socket_other.send_to(boost::asio::buffer(buffer, len), sender_endpoint);

    heartbeat_timer->expires_at(heartbeat_timer->expiry() + boost::asio::chrono::seconds(1));
    heartbeat_timer->async_wait(std::bind(&sendHeartbeat, std::placeholders::_1, heartbeat_timer));
}

int main() {
    boost::asio::steady_timer heartbeat_timer(io_service, boost::asio::chrono::seconds(1));
    heartbeat_timer.async_wait(std::bind(&sendHeartbeat, std::placeholders::_1, &heartbeat_timer));

    std::cout << "Received message" <<std::endl;
    socket_other.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, handle_receive);
    io_service.run();

    return 0;
}