#include <precise_driver/precise_tcp_interface.h>

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
            _socket->connect(tcp::endpoint(boost::asio::ip::address::from_string(_ip), _port));
	    _connected = true;
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


    std::string PreciseTCPInterface::send(const std::string &data)
    {
        if (!_connected)
	  connect();
	      
        boost::system::error_code error;
      
	// Send
        boost::asio::write(*_socket.get(), boost::asio::buffer(data + "\r\n"), error);
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
	    else if (len < 128)	                  // Buffer was not filled completely
	        break;
	    else if (error)	                  // Some error occured
	      throw boost::system::system_error(error);

	}

        return result;
    }

} // namespace precise_driver
