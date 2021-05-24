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

#ifndef FILE_LOADER_HPP_
#define FILE_LOADER_HPP_

#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>

#include "parameter_manager.hpp"

class FileLoader
{
    public:
        FileLoader(const std::string &filepath, uint32_t baud_rate=921600);
        FileLoader(const FileLoader&) = delete;
        FileLoader& operator=(const FileLoader&) = delete;
        ~FileLoader();
        void startRead();
        void close();
        void addCallback(std::function<void (const uint8_t*, size_t)> callback);

    private:
        void read_handler();

    private:
        std::string filepath_;
        uint32_t baud_rate_;
        std::ifstream ifs_;
        std::atomic<bool> paused_;
        std::atomic<bool> stopped_;
        std::atomic<bool> finished_;
        std::vector<std::function<void (const uint8_t*, size_t)>> callbacks_;
        std::thread read_thread_;
        uint32_t MSG_HEADER_LEN;
};

#endif