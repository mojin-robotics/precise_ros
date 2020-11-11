#include <precise_driver/device/tcp_client.h>
#include <ros/ros.h>

using namespace boost::asio;
using ip::tcp;

namespace precise_driver
{
    TCPClient::TCPClient(const std::string &ip, const unsigned &port)
    {
        _connected = false;
        _ip = ip;
        _port = port;
        _socket.reset(new boost::asio::ip::tcp::socket(_io_service));
    }

    void TCPClient::connect()
    {
        if (!_connected)
        {
            ROS_INFO_STREAM("Connecting to "<<_ip<<":"<<_port);
            try{
                _socket->connect(tcp::endpoint(boost::asio::ip::address::from_string(_ip), _port));
                _connected = true;
            }catch(boost::system::system_error &e)
            {
                _connected = false;
                throw boost::system::system_error(e);
            }
        }
        else
        {
            disconnect();
            connect();
        }
    }

    void TCPClient::disconnect()
    {
        if (_connected)
        {
            _socket->close();
            _connected = false;
        }
    }

    Response TCPClient::send(const std::string &data)
    {
        std::lock_guard<std::mutex> guard(this->_comm_mutex);
        if (!_connected)
            connect();

        boost::system::error_code error;

        ROS_DEBUG_STREAM_NAMED("tcp_client","sending to "<<_ip<<":"<<_port<<": "<<data);

        // Send
        boost::asio::write(*_socket.get(), boost::asio::buffer(data + "\n"), error);
        if (error)
        {
            throw boost::system::system_error(error);
        }

        // Receive
        std::string result = "";

        for (;;)
        {
            char buf[128];

            size_t len = _socket->read_some(boost::asio::buffer(buf), error);

            std::string data(buf, buf + len);
            result += data;

            if (error == boost::asio::error::eof) // Connection was closed clean
                break;
            else if (len < 128) // Buffer was not filled completely
                break;
            else if (error) // Some error occured
                throw boost::system::system_error(error);
        }

        ROS_DEBUG_STREAM_NAMED("tcp_client","received from "<<_ip<<":"<<_port<<": "<<result);

        Response res;
        std::stringstream ss;
        ss.str(result);
        if(result.size() > 0)
        {
            ss >> res.error;
        }
        if(res.error == 0 && result.size() > 1)
        {
            std::getline(ss, res.message);
            res.success = true;
        }
        else if(res.error == 0 && result.size() == 1)
        {
            res.success = true;
        }
        else
        {
            res.success = false;
            std::getline(ss, res.message);
            ROS_ERROR_STREAM("error response code: "<< res.error <<" message: "<<res.message);
        }

        return res;
    }

} // namespace precise_driver
