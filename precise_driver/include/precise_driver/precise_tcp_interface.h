#pragma once

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace precise_driver
{
    class PreciseTCPInterface
    {
    public:
        PreciseTCPInterface(const std::string &ip, const unsigned &port);

        void connect();
        void disconnect();
        std::string send(const std::string &data);

    private:
        bool _connected;
        std::string _ip;
        unsigned _port;
        boost::asio::io_service _io_service;
        boost::shared_ptr<boost::asio::ip::tcp::socket> _socket;
  
    }; // class
  
} // namespace precise_driver
