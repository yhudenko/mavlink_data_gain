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

                        break;
                    case MAVLINK_MSG_ID_SCALED_PRESSURE:

                        break;
                    case MAVLINK_MSG_ID_SCALED_IMU:

                        break;
                }
            }
        }
        socket_other.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, handle_receive);
    } else {
        std::cerr << "Error in receive: " << error.message() << std::endl;
    }
}

int main() {
    socket_other.async_receive_from(boost::asio::buffer(buffer), sender_endpoint, handle_receive);
    io_service.run();

    return 0;
}