#pragma once

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <mutex>

namespace precise_driver
{
    struct Response
    {
        bool success;
        std::string message;
        int error;
    };

    class TCPClient
    {
    public:
        TCPClient(const std::string &ip, const unsigned &port);

        void connect();
        void disconnect();
        Response send(const std::string &data);

    private:
        bool _connected;
        std::string _ip;
        unsigned _port;
        boost::asio::io_service _io_service;
        boost::shared_ptr<boost::asio::ip::tcp::socket> _socket;
        std::mutex _comm_mutex;

    }; // class

} // namespace precise_driver
