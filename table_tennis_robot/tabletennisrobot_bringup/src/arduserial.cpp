#include "arduserial.h"

ArduSerial::ArduSerial(std::string port, size_t length)
{
    _port = port;
    _length = length;
}

ArduSerial::~ArduSerial()
{
}

void ArduSerial::init()
{
    _ser.setPort(_port);
    _ser.setBaudrate(9600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    _ser.setTimeout(to);
    _ser.open();
    ROS_INFO("arduserial init port: %s", _port.c_str());
}

void ArduSerial::read(uint8_t *data, size_t size)
{
    try
    {
        if (_ser.isOpen())
        {
            // read string from serial device
            if (_ser.available())
            {
                _read = _ser.read(_ser.available());
                ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)_read.size(), (int)_input.size());
                _input += _read;
                //ROS_INFO("in _ser.available");
                while (_input.length() >= size) // while there might be a complete package in input
                {
                    //ROS_INFO("in while");
                    //parse for data packets
                    data_packet_start = _input.find("$\x03");
                    if (data_packet_start != std::string::npos)
                    {
                        //ROS_INFO("found possible start of data packet at position %d", data_packet_start);
                        if ((_input.length() >= data_packet_start + size) && (_input.compare(data_packet_start + size - 2, 2, "\r\n") == 0)) //check if positions 26,27 exist, then test values
                        {
                            ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                            /////////////////////////////////////////////////////////
                            for (int i = 0; i < size - (unsigned long)4; i++)
                            {
                                data[i] = _input[data_packet_start + i + 2];
                            }
                            /////////////////////////////////////////////////////////

                            _input.erase(0, data_packet_start + size); // delete everything up to and including the processed packet
                        }
                        else
                        {
                            if (_input.length() >= data_packet_start + size)
                            {
                                _input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                            }
                            else
                            {
                                // do not delete start character, maybe complete package has not arrived yet
                                _input.erase(0, data_packet_start);
                            }
                        }
                    }
                    else
                    {
                        // no start character found in input, so delete everything
                        _input.clear();
                    }
                }
            }
        }
        else
        {
            // try and open the serial port
            try
            {
                _ser.setPort(_port);
                _ser.setBaudrate(9600);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                _ser.setTimeout(to);
                _ser.open();
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("Unable to open serial port " << _ser.getPort() << ". Trying again in 5 seconds.");
                ros::Duration(5).sleep();
            }

            if (_ser.isOpen())
            {
                ROS_DEBUG_STREAM("Serial port " << _ser.getPort() << " initialized and opened.");
            }
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Error reading from the serial port " << _ser.getPort() << ". Closing connection.");
        _ser.close();
    }
}

void ArduSerial::write(uint8_t *data, size_t size)
{
    if (_ser.isOpen())
    {
        size_t size_write = _ser.write(data, size);
        ///ROS_INFO("write data to arduino : %s",_ser.getPort().c_str());
        //std::cout << "write: " << size_write <<std::endl;
    }
}

void ArduSerial::read_dick(uint8_t *data, size_t size)
{
    try
    {
        if (_ser.isOpen())
        {
            //std::cout << "open" << std::endl;
            //std::cout << "in dick serial available: " << _ser.available() << std::endl;
            if (_ser.available())
            {
                data[0] =  0x24;
                data[1] =  0x25;
                _read = _ser.read(_ser.available());
                ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)_read.size(), (int)_input.size());
                _input += _read;
                //std::cout << "input length: " << _input.length() << std::endl;
                while (_input.length() >= size) // while there might be a complete package in input
                {
                    //std::cout << "input: " << _input.c_str() << std::endl;
                    //std::cout << "###############" << std::endl;
                    data_packet_start = _input.find("\n");
                    if (data_packet_start != std::string::npos)
                    {
                        //std::cout << "data_packet_start : " << data_packet_start << std::endl;
                        //std::cout << "size : " << size << std::endl;
                        //ROS_INFO("found possible start of data packet at position %d", data_packet_start);
                        if ((_input.length() >= data_packet_start + size) && (_input.compare(data_packet_start + size + 2, 1, "\n") == 0)) //check if positions 26,27 exist, then test values
                        {
                            ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                            /////////////////////////////////////////////////////////
                            if (_input.compare(data_packet_start + 2, 1, "s") == 0)
                            {
                                //std::cout << "aaaaaaaaaaaaaaaa" << std::endl;
                                /*std::string _raw_data;
                        for (int i = 0; i < size; i++)
                        {
                            //data[i] = _input[data_packet_start + i + 1];
                            _raw_data += _input[data_packet_start+i+1];
                        }
                        std::cout <<  _raw_data.c_str() << std::endl;
                        std::cout << "aaaaaaaaaaaaaaaa: " << _raw_data.length() << std::endl;*/
                                data[0] = _input[data_packet_start + 1];
                                data[1] = _input[data_packet_start + 3];
                            }
                            /////////////////////////////////////////////////////////
                            _input.clear();
                            // _input.erase(0, data_packet_start + size); // delete everything up to and including the processed packet
                        }
                        else
                        {
                            if (_input.length() >= data_packet_start + size)
                            {
                                // delete up to false data_packet_start character so it is not found again
                                _input.erase(0, data_packet_start + 1);
                            }
                            else
                            {
                                // do not delete start character, maybe complete package has not arrived yet
                                _input.erase(0, data_packet_start);
                            }
                        }
                    }
                    else
                    {
                        _input.clear();
                    }
                }
            }
        }
        else
        {
            // try and open the serial port
            try
            {
                _ser.setPort(_port);
                _ser.setBaudrate(9600);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                _ser.setTimeout(to);
                _ser.open();
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("Unable to open serial port " << _ser.getPort() << ". Trying again in 5 seconds.");
                ros::Duration(5).sleep();
            }

            if (_ser.isOpen())
            {
                ROS_DEBUG_STREAM("Serial port " << _ser.getPort() << " initialized and opened.");
            }
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Error reading from the serial port " << _ser.getPort() << ". Closing connection.");
        _ser.close();
    }
    //data[0] = 0x24; //$
    //data[1] = 0x25; //%
}