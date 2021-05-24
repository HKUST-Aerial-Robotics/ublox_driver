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

#include "file_loader.hpp"

// data members are initialized base on their decleration order in header file
FileLoader::FileLoader(const std::string &filepath, uint32_t baud_rate) : 
    filepath_(filepath), baud_rate_(baud_rate), paused_(true), stopped_(false),
    finished_(false), read_thread_(std::bind(&FileLoader::read_handler, this)), 
    MSG_HEADER_LEN(ParameterManager::getInstance().MSG_HEADER_LEN)
{
    // open file
    ifs_.open(filepath_, std::ios::in | std::ios::binary);
    if (!ifs_.good())
    {
        std::cerr << "Unable to open file " << filepath_ << ".\n";
        finished_ = true;
    }
}

FileLoader::~FileLoader()
{
    close();
    finished_ = true;
    read_thread_.join();
    if (ifs_.is_open())
        ifs_.close();
}

void FileLoader::startRead()
{
    paused_ = false;
}

void FileLoader::close()
{
    stopped_ = true;
}

void FileLoader::addCallback(std::function<void (const uint8_t*, size_t)> callback)
{
    callbacks_.push_back(callback);
}

void FileLoader::read_handler()
{
    uint32_t buf_size = 8192;
    std::unique_ptr<uint8_t[]> data_buf(new uint8_t[buf_size]);
    while(!finished_ && !stopped_)
    {
        if (paused_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        ifs_.read((char*)(data_buf.get()), MSG_HEADER_LEN);
        uint32_t header_remains = MSG_HEADER_LEN;
        while (ifs_.good() && header_remains != 0)
        {
            uint32_t pream_idx = 0;
            // search for u-blox preamble sequence 
            for (; pream_idx < MSG_HEADER_LEN-1; ++pream_idx)
                if (data_buf[pream_idx] == 0xB5 && data_buf[pream_idx+1] == 0x62)
                    break;
            header_remains = pream_idx;
            // if more data needed by header, should only happen at boot phase
            if (pream_idx != 0)
            {
                // copy the valid header / remain unchecked part to the front of data buffer
                for (uint32_t i = pream_idx; i < MSG_HEADER_LEN; ++i)
                    data_buf[i-pream_idx] = data_buf[i];
                
                ifs_.read((char*)(data_buf.get()+MSG_HEADER_LEN-pream_idx), header_remains);
            }
        }

        if (!ifs_.good())
        {
            finished_ = true;
            continue;
        }

        const uint16_t payload_len = *reinterpret_cast<uint16_t*>(data_buf.get()+4);
        const uint16_t msg_len = MSG_HEADER_LEN+payload_len+2;
        // if message is too large, reallocate data_buf
        if (msg_len > buf_size)
        {
            const uint32_t new_buf_size = buf_size*2;
            uint8_t *new_data_buf = new uint8_t[new_buf_size];
            memcpy((char*)new_data_buf, (char*)(data_buf.get()), MSG_HEADER_LEN);
            data_buf.reset(new_data_buf);
            buf_size = new_buf_size;
        }

        // read message data and CRC
        ifs_.read((char*)(data_buf.get()+MSG_HEADER_LEN), payload_len+2);
        if (!ifs_.good())
        {
            finished_ = true;
            continue;
        }

        // invoke all callbacks
        for (auto &f : callbacks_)
            f(data_buf.get(), msg_len);
        
        int64_t sleep_time_us = static_cast<int64_t>(msg_len*8.0/baud_rate_*1.0e6);
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
    }
    if (finished_)
        std::cout << "Done with file " << filepath_ << std::endl;
    ifs_.close();
}