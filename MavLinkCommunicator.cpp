#include "MavLinkCommunicator.h"
#include <iostream>
#include <functional>

MavLinkCommunicator::MavLinkCommunicator(int port) 
    : socket_other(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
    heartbeat_timer(io_service, boost::asio::chrono::seconds(1))
{
    // Schedule the first heartbeat transmission
    heartbeat_timer.async_wait([this](const boost::system::error_code& error) {
        if (!error) {
            sendHeartbeat(error);
        } else {
            // Handle error in wait
            std::cerr << "Error in wait: " << error.message() << std::endl;
        }
    });

    // Start listening for incoming messages
    std::cout << "Received message" << std::endl;
    socket_other.async_receive_from(boost::asio::buffer(buffer), sender_endpoint,
                                    std::bind(&MavLinkCommunicator::handle_receive, this, std::placeholders::_1, std::placeholders::_2));

    // Run the IO service to perform asynchronous operations
    io_service.run();
}

MavLinkCommunicator::~MavLinkCommunicator()
{
    // Stop the IO service to terminate any pending asynchronous operations
    io_service.stop();
    // Close the socket
    socket_other.close();
}

// Callback function for handling received data
void MavLinkCommunicator::handle_receive(const std::error_code& error, std::size_t bytes_transferred) {
    if (!error) {
        mavlink_message_t msg;
        mavlink_status_t status;

        // Iterate through received bytes
        for (std::size_t i = 0; i < bytes_transferred; ++i) {
            // Parse MAVLink message from the received byte
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                // Handle different MAVLink message types
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                        // Print information about the received heartbeat message
                        std::cout << "Received Heartbeat: System ID - " << (int)msg.sysid
                            << ", Component ID - " << (int)msg.compid << std::endl;
                        break;
                    case MAVLINK_MSG_ID_SCALED_PRESSURE:
                        mavlink_scaled_pressure_t scaled_pressure;
                        mavlink_msg_scaled_pressure_decode(&msg, &scaled_pressure);

                        // Print information about the received scaled pressure message
                        float temperature = scaled_pressure.temperature;
                        uint32_t pressure = scaled_pressure.press_abs;

                        std::cout << "Temperature: " << temperature << " C, Pressure: " << pressure << " Pa" << std::endl;
                }
            }
        }

        // Continue listening for incoming messages
        socket_other.async_receive_from(boost::asio::buffer(buffer), sender_endpoint,
                                        std::bind(&MavLinkCommunicator::handle_receive, this, std::placeholders::_1, std::placeholders::_2));
    } else {
        // Handle error in receiving data
        std::cerr << "Error in receive: " << error.message() << std::endl;
    }
}

// Function to send heartbeat messages at regular intervals
void MavLinkCommunicator::sendHeartbeat(const std::error_code& error) {
    std::cerr << "Send Heartbeat" << std::endl;

    // Create and pack MAVLink heartbeat message
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

    // Convert MAVLink message to a send buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Send heartbeat message
    socket_other.send_to(boost::asio::buffer(buffer, len), sender_endpoint);

    // Schedule the next heartbeat transmission after one second
    heartbeat_timer.expires_at(heartbeat_timer.expiry() + boost::asio::chrono::seconds(1));
    heartbeat_timer.async_wait([this](const boost::system::error_code& error) {
        if (!error) {
            sendHeartbeat(error);
        } else {
            // Handle error in wait
            std::cerr << "Error in wait: " << error.message() << std::endl;
        }
    });
}