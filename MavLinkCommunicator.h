#include "boost/asio.hpp"
#include "c_library_v2-master/common/mavlink.h"
#include <array>

class MavLinkCommunicator
{
public:
    MavLinkCommunicator(int port = 14551);
    ~MavLinkCommunicator();


private:
    // Callback function for handling received data
    void handle_receive(const std::error_code &error, std::size_t bytes_transferred);
    // Function to send heartbeat messages at regular intervals
    void sendHeartbeat(const std::error_code &error);

    // Boost Asio IO service for asynchronous operations
    boost::asio::io_service io_service;
    // UDP socket for receiving messages
    boost::asio::ip::udp::socket socket_other;
    // Endpoint for the sender of the received message
    boost::asio::ip::udp::endpoint sender_endpoint;
    // Create a steady timer for sending heartbeats
    boost::asio::steady_timer heartbeat_timer;
    // Buffer for storing received data
    std::array<char, 1024> buffer;
};