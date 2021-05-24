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

#ifndef FILE_DUMPER_HPP_
#define FILE_DUMPER_HPP_

#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>

class FileDumper
{
    public:
        FileDumper(const std::string &filepath, uint32_t buf_len=8192);
        FileDumper(const FileDumper&) = delete;
        FileDumper& operator=(const FileDumper&) = delete;
        ~FileDumper();
        void process_data(const uint8_t *data, size_t len);

        static int createDirectoryIfNotExists(const char *path);

    private:
        std::string filepath_;
        uint32_t buf_len_;
        uint32_t buf_pos_;
        std::unique_ptr<uint8_t[]> data_buf_;
};

#endif