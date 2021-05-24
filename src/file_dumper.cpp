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

#include "file_dumper.hpp"

FileDumper::FileDumper(const std::string &filepath, uint32_t buf_len) : 
    filepath_(filepath), buf_len_(buf_len), buf_pos_(0), data_buf_(new uint8_t[buf_len_])
{
    // create file at first
    const size_t last_slash_pos = filepath_.rfind('/');
    if (last_slash_pos == std::string::npos)
    {
        std::cerr << "filepath error: " << filepath_ << '\n';
        return;
    }
    const std::string folder_path = filepath_.substr(0, last_slash_pos);
    if (FileDumper::createDirectoryIfNotExists(folder_path.c_str()))
    {
        std::cerr << "Failed to create " << folder_path << '\n';
        return;
    }

    std::ofstream ofs(filepath_, std::ios::out | std::ios::binary);
    if (!ofs.good())
        std::cerr << "Unable to open file " << filepath_ << ".\n";
    ofs.close();
}

FileDumper::~FileDumper()
{
    if (buf_pos_ > 0)
    {
        std::ofstream ofs(filepath_, std::ios::out | std::ios::app | std::ios::binary);
        ofs.write((const char*)(data_buf_.get()), buf_pos_);
        ofs.close();
    }
}

void FileDumper::process_data(const uint8_t *data, size_t len)
{
    if (buf_pos_+len > buf_len_)
    {
        std::ofstream ofs(filepath_, std::ios::out | std::ios::app | std::ios::binary);
        ofs.write((const char*)(data_buf_.get()), buf_pos_);
        ofs.write((const char*)data, len);
        ofs.close();
        buf_pos_ = 0;
    }
    else
    {
        memcpy(data_buf_.get()+buf_pos_, data, len);
        buf_pos_ += len;
    }
}

/******************************************************************************
 * Recursively create directory if `path` not exists.
 * Return 0 if success.
 *****************************************************************************/
int FileDumper::createDirectoryIfNotExists(const char *path)
{
    struct stat info;
    int statRC = stat(path, &info);
    if( statRC != 0 )
    {
        if (errno == ENOENT)  
        {
            printf("%s not exists, trying to create it \n", path);
            if (! createDirectoryIfNotExists(dirname(strdupa(path))))
            {
                if (mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
                {
                    fprintf(stderr, "Failed to create folder %s \n", path);
                    return 1;
                }
                else
                    return 0;
            }
            else 
                return 1;
        } // directory not exists
        if (errno == ENOTDIR) 
        { 
            fprintf(stderr, "%s is not a directory path \n", path);
            return 1; 
        } // something in path prefix is not a dir
        return 1;
    }
    return ( info.st_mode & S_IFDIR ) ? 0 : 1;
}
