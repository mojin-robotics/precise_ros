#include <precise_driver/precise_tcp_interface.h>
#include <ros/ros.h>

using namespace boost::asio;
using ip::tcp;

namespace precise_driver
{
    PreciseTCPInterface::PreciseTCPInterface(const std::string &ip, const unsigned &port)
    {
        _connected = false;
        _ip = ip;
        _port = port;
        _socket.reset(new boost::asio::ip::tcp::socket(_io_service));
    }

    void PreciseTCPInterface::connect()
    {
        if (!_connected)
        {
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

    void PreciseTCPInterface::disconnect()
    {
        if (_connected)
        {
            _socket->close();
            _connected = false;
        }
    }

    Response PreciseTCPInterface::send(const std::string &data)
    {
        std::lock_guard<std::mutex> guard(this->_comm_mutex);
        if (!_connected)
            connect();

        boost::system::error_code error;

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
