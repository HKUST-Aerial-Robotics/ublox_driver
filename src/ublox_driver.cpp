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

#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stdlib.h>
#include <time.h>
// #include <sys/procmgr.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ros/ros.h>

#include "parameter_manager.hpp"
#include "serial_handler.hpp"
#include "socket_handler.hpp"
#include "file_dumper.hpp"
#include "file_loader.hpp"
#include "ublox_message_processor.hpp"

static std::atomic<bool> interrupted(false);

// variables for receiver config at start
std::mutex ack_m;
std::condition_variable ack_cv;
int ack_flag = 0;

void ctrl_c_handler(int s)
{
    interrupted = true;
}

std::string time_str()
{
    std::stringstream ss;
    std::time_t time_ptr;
    time_ptr = time(NULL);
    tm *tm_local = localtime(&time_ptr);
    ss << tm_local->tm_year + 1900 << '_' << tm_local->tm_mon + 1 << '_'
       << tm_local->tm_mday << '_' << tm_local->tm_hour << '_' 
       << tm_local->tm_min << '_' << tm_local->tm_sec;
    return ss.str();
}

void config_ack_callback(const uint8_t *data, size_t len)
{
    int ack_result = UbloxMessageProcessor::check_ack(data, len);
    if (ack_result != 0)
    {
        std::lock_guard<std::mutex> ack_lk(ack_m);
        ack_flag = ack_result;
    }
    ack_cv.notify_one();
}

bool config_receiver(std::shared_ptr<SerialHandler> serial, std::vector<RcvConfigRecord> &rcv_configs)
{
    const uint32_t rcv_config_buff_capacity = 8192;
    std::unique_ptr<uint8_t[]> rcv_config_buff(new uint8_t[rcv_config_buff_capacity]);
    memset(rcv_config_buff.get(), 0, rcv_config_buff_capacity);
    uint32_t msg_len = 0;
    UbloxMessageProcessor::build_config_msg(rcv_configs, rcv_config_buff.get(), msg_len);
    std::unique_lock<std::mutex> ack_lk(ack_m);
    ack_flag = 0;
    serial->addCallback(std::bind(&config_ack_callback, 
        std::placeholders::_1, std::placeholders::_2));
    serial->writeRaw(rcv_config_buff.get(), msg_len);
    serial->startRead();
    // block, wait ack
    ack_cv.wait(ack_lk, []{return ack_flag != 0;});
    serial->stop_read();
    ack_lk.unlock();
    // resume
    if (ack_flag == 1)
        return true;
    
    return false;
}

int main(int argc, char **argv)
{
    ::google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    if (!interrupted.is_lock_free())  return 10;

    ros::init(argc, argv, "ublox_driver");
    ros::NodeHandle nh("~");

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    // procmgr_ability( 0, PROCMGR_AID_CLOCKSET );

    std::string config_filepath;
    nh.getParam("config_file", config_filepath);
    // config_filepath = argv[1];
    ParameterManager &pm(ParameterManager::getInstance());
    pm.read_parameter(config_filepath);

    std::shared_ptr<SerialHandler> serial;
    std::shared_ptr<SocketHandler> socket;
    std::shared_ptr<FileLoader> file_loader;
    std::shared_ptr<FileDumper> file_dumper;
    std::shared_ptr<UbloxMessageProcessor> ublox_msg_processor;
    std::shared_ptr<SerialHandler> output_serial;
    if (pm.to_ros)
        ublox_msg_processor.reset(new UbloxMessageProcessor(nh));
    if (pm.to_file)
    {
        const std::string t_str = time_str();
        const std::string dump_filepath = pm.dump_dir + "/" + t_str + ".ubx";
        file_dumper.reset(new FileDumper(dump_filepath));
    }
    if (pm.to_serial)
    {
        output_serial.reset(new SerialHandler(pm.output_serial_port, pm.serial_baud_rate));
    }
    
    if (pm.online)
    {
        serial.reset(new SerialHandler(pm.input_serial_port, pm.serial_baud_rate));

        if (pm.config_receiver_at_start)
        {
            if (config_receiver(serial, pm.receiver_configs))
                LOG(ERROR) << "Successfully configured the receiver.";
            else
                LOG(FATAL) << "Error occurs when configuring the receiver.";
        }
        
        if (pm.input_rtcm)
        {
            socket.reset(new SocketHandler("localhost", pm.rtcm_tcp_port));
            socket->addCallback(std::bind(&SerialHandler::writeRaw, serial.get(), 
                std::placeholders::_1, std::placeholders::_2, pm.IO_TIMEOUT_MS));
            socket->startRead();
        }
        
        if (pm.to_ros)
            serial->addCallback(std::bind(&UbloxMessageProcessor::process_data, 
                ublox_msg_processor.get(), std::placeholders::_1, std::placeholders::_2));
            
        if (pm.to_file)
            serial->addCallback(std::bind(&FileDumper::process_data, file_dumper.get(), 
                std::placeholders::_1, std::placeholders::_2));
        
        if (pm.to_serial)
            serial->addCallback(std::bind(&SerialHandler::writeRaw, output_serial.get(), 
                std::placeholders::_1, std::placeholders::_2, pm.IO_TIMEOUT_MS));

        serial->startRead();
    }
    else
    {
        file_loader.reset(new FileLoader(pm.ubx_filepath, pm.serial_baud_rate));

        if (pm.to_ros)
            file_loader->addCallback(std::bind(&UbloxMessageProcessor::process_data, 
                ublox_msg_processor.get(), std::placeholders::_1, std::placeholders::_2));
            
        if (pm.to_file)
            file_loader->addCallback(std::bind(&FileDumper::process_data, file_dumper.get(), 
                std::placeholders::_1, std::placeholders::_2));
            
        if (pm.to_serial)
            file_loader->addCallback(std::bind(&SerialHandler::writeRaw, output_serial.get(), 
                std::placeholders::_1, std::placeholders::_2, pm.IO_TIMEOUT_MS));

        file_loader->startRead();
    }

    ros::Rate loop(50);
    while (ros::ok() && !interrupted)
    {
        ros::spinOnce();
        loop.sleep();
    }

    if (serial)
        serial->close();
    if (file_loader)
        file_loader->close();
    
    return 0;
}