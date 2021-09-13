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

#ifndef PARAMETER_MANAGER_HPP_
#define PARAMETER_MANAGER_HPP_

#include <string>
#include <fstream>
#include <glog/logging.h>
#include <eigen3/Eigen/Dense>

#include "yaml/Yaml.hpp"

typedef std::pair<std::string, std::string> RcvConfigRecord;

class ParameterManager
{
    private:
        ParameterManager() { };
        ~ParameterManager() { };
        ParameterManager(const ParameterManager&);
        ParameterManager& operator=(const ParameterManager&);

        std::vector<double> parse_list(std::string list_str)
        {
            std::vector<double> result;
            size_t data_start = list_str.find_first_not_of("[ ");
            list_str = list_str.substr(data_start);
            size_t data_ends = list_str.find_last_not_of("] ");
            list_str.resize(data_ends+1);

            std::stringstream ss(list_str);
            std::string line;
            while(std::getline(ss, line, ','))
            {
                result.emplace_back(std::stod(line));
            }

            return result;
        }

        Eigen::MatrixXd parse_matrix(Yaml::Node &matrix_node)
        {
            Eigen::MatrixXd result;
            int rows = matrix_node["rows"].As<int>();
            int cols = matrix_node["cols"].As<int>();
            assert(rows > 0 && cols > 0);
            result.resize(rows, cols);
            std::vector<double> matrix_data = parse_list(matrix_node["data"].As<std::string>());
            assert(matrix_data.size() == static_cast<size_t>(rows*cols));
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j)
                    result(i, j) = matrix_data.at(i*cols+j);
            return result;
        }

        // convert relative path to the absolute one
        std::string path_rel2abs(std::string rel_path)
        {
            assert(!rel_path.empty() && "Input relative path cannot be empty.\n");
            if (rel_path[0] == '~')
                rel_path.replace(0, 1, getenv("HOME"));
            char actual_dir[PATH_MAX+1];
            if(!realpath(rel_path.c_str(), actual_dir))
                std::cerr << "ERROR: Failed to obtain the absolute path of " << rel_path << '\n';
            return std::string(actual_dir);
        }

    public:
        static ParameterManager& getInstance() 
            {
            static ParameterManager instance;
            return instance;
        }

        void read_parameter(const std::string &driver_config_file)
        {
            Yaml::Node driver_config_root;
            Yaml::Parse(driver_config_root, driver_config_file.c_str());
            online = driver_config_root["online"].As<int>() > 0 ? true : false;
            input_rtcm = driver_config_root["input_rtcm"].As<int>() > 0 ? true : false;
            to_ros = driver_config_root["to_ros"].As<int>() > 0 ? true : false;
            to_file = driver_config_root["to_file"].As<int>() > 0 ? true : false;
            to_serial = driver_config_root["to_serial"].As<int>() > 0 ? true : false;
            config_receiver_at_start = driver_config_root["config_receiver_at_start"].As<int>() > 0 ? true : false;
            if (online)
            {
                input_serial_port = driver_config_root["input_serial_port"].As<std::string>();
                serial_baud_rate = driver_config_root["serial_baud_rate"].As<long>();
                rtcm_tcp_port = driver_config_root["rtcm_tcp_port"].As<uint64_t>();
            }
            else
            {
                ubx_filepath = path_rel2abs(driver_config_root["ubx_filepath"].As<std::string>());
            }
            if (to_serial)
                output_serial_port = driver_config_root["output_serial_port"].As<std::string>();
            if (to_file)
                dump_dir = path_rel2abs(driver_config_root["dump_dir"].As<std::string>());
            rtk_correction_ecef = parse_matrix(driver_config_root["rtk_correction_ecef"]).topLeftCorner<3, 1>();
            if (online && config_receiver_at_start)
            {
                std::string rcv_config_filepath = path_rel2abs(driver_config_root[
                    "receiver_config_filepath"].As<std::string>());
                if (!rcv_config_filepath.empty())
                {
                    Yaml::Node rcv_config_root;
                    Yaml::Parse(rcv_config_root, rcv_config_filepath.c_str());
                    for (Yaml::Iterator it = rcv_config_root.Begin(); it != rcv_config_root.End(); it++)
                        receiver_configs.emplace_back((*it).first, (*it).second.As<std::string>());
                }
            }
        }

        bool online, input_rtcm;
        bool to_ros, to_file, to_serial;
        bool config_receiver_at_start;
        std::string ubx_filepath;
        std::string output_serial_port;
        std::string input_serial_port;
        long serial_baud_rate;
        uint64_t rtcm_tcp_port;
        std::string dump_dir;
        Eigen::Vector3d rtk_correction_ecef;
        std::vector<RcvConfigRecord> receiver_configs;
        constexpr static uint32_t MSG_HEADER_LEN = 6;
        constexpr static uint32_t IO_TIMEOUT_MS = 50;
};

#endif