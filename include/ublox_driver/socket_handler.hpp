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

#ifndef SOCKET_HANDLER_HPP_
#define SOCKET_HANDLER_HPP_

#include <iostream>
#include <vector>
#include <memory>
#include <functional>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "parameter_manager.hpp"

class SocketHandler
{
    public:
        SocketHandler(const std::string &host, uint64_t port, unsigned int buf_size=8192);
        SocketHandler(const SocketHandler&) = delete;
        SocketHandler& operator=(const SocketHandler&) = delete;
        ~SocketHandler();
        void addCallback(std::function<void (const uint8_t*, size_t)> callback);
        void startRead();
        void write(const std::string &str, uint32_t timeout_ms=50);
        void writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms=50);
        void close();

    private:
        void read_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
        void write_callback(const boost::system::error_code& error, std::size_t bytes_transferred);
        void wait_callback(const boost::system::error_code& error);
        std::string host_;
        uint64_t port_;
        uint32_t buf_size_;
        std::unique_ptr<uint8_t[]> data_buf_;
        boost::asio::io_service io_service_, timer_io_service_;
        boost::asio::ip::tcp::socket data_socket_;
        boost::asio::io_service::work keep_working_task_;
        boost::asio::deadline_timer write_timeout_;
        std::vector<std::function<void(const uint8_t*, size_t)>> callbacks_;
};

#endif