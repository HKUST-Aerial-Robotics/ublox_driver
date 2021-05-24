/**
* This file is part of ublox-driver.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* ublox-driver is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ublox-driver is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ublox-driver. If not, see <http://www.gnu.org/licenses/>.
*/

#include "socket_handler.hpp"

SocketHandler::SocketHandler(const std::string &host, uint64_t port, unsigned int buf_size) : host_(host),
                    port_(port), buf_size_(buf_size), data_buf_(new uint8_t[buf_size_]), data_socket_(io_service_), 
                    keep_working_task_(io_service_), write_timeout_(timer_io_service_)
{
    boost::asio::ip::tcp::resolver resolver(io_service_);
    try
    {
        boost::asio::ip::tcp::resolver::query query_config(host, std::to_string(port));
        boost::asio::ip::tcp::resolver::iterator endpoint_config = resolver.resolve(query_config);
        data_socket_.connect(*endpoint_config);
    }
    catch (boost::system::system_error const &e)
    {
        std::cerr << "Could not connect to " << host << " at port " << port << '\n';
        return;
    }

    if (!data_socket_.is_open())
    {
        std::cerr << "Cannot open socket " << host << ':' << port << '\n';
        return;
    }
    boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service_));
}

SocketHandler::~SocketHandler()
{
    close();
    data_socket_.close();
}

void SocketHandler::addCallback(std::function<void(const uint8_t*, size_t)> callback)
{
    callbacks_.push_back(callback);
}

void SocketHandler::startRead()
{
    boost::asio::async_read(data_socket_, boost::asio::buffer(data_buf_.get(), buf_size_), 
            boost::asio::transfer_at_least(1),
            boost::bind(&SocketHandler::read_handler, this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));
}

void SocketHandler::write(const std::string &str, uint32_t timeout_ms)
{
    writeRaw((const uint8_t*)(str.c_str()), str.size(), timeout_ms);
}

void SocketHandler::writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    boost::asio::async_write(data_socket_, boost::asio::buffer(data, len), 
        boost::bind(&SocketHandler::write_callback, this, 
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    write_timeout_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    write_timeout_.async_wait(boost::bind(&SocketHandler::wait_callback, this, 
        boost::asio::placeholders::error));
    timer_io_service_.run();
    timer_io_service_.reset();
}

void SocketHandler::close()
{
    data_socket_.cancel();
    write_timeout_.cancel();
    if (!io_service_.stopped())
        io_service_.stop();
    if (!timer_io_service_.stopped())
        timer_io_service_.stop();
}

void SocketHandler::read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error)
    {
        // simply invoke callback functions
        for (auto &f : callbacks_)
            f(data_buf_.get(), bytes_transferred);
        
        // start another read session
        startRead();
    }
    else if (error == boost::asio::error::operation_aborted || error != boost::asio::error::bad_descriptor)
    {
        std::cerr << "Socket " << host_ << ':' << port_ << " read handler aborted due to shutdown.\n";
        return;
    }
    else
        std::cerr << "Connection to socket" << host_ << ':' << port_ << " lost.\n";
}

void SocketHandler::write_callback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (error || !bytes_transferred)
    {
        // no data was written
        std::cerr << "Socket write failed or operation timeout: " << host_ << ':' << port_ << '\n';
        return;
    }
    write_timeout_.cancel();
}

void SocketHandler::wait_callback(const boost::system::error_code& error)
{
    if (error)  return;                 // data was written and this timer was cancelled

    data_socket_.cancel();       // will cause write_callback to fire with an error
}
