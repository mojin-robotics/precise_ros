#pragma once

#include <iostream>
#include <boost/asio.hpp>

namespace precise_driver
{
    class PreciseTCPInterface
    {
    public:
        PreciseTCPInterface(std::string &ip);

        void connect();
        void disconnect();
        std::string send(std::string &data);

    private:
      
    }; // class
  
} // namespace precise_driver
