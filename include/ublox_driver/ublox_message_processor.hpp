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

#ifndef UBLOX_MESSAGE_PROCESSOR_HPP_
#define UBLOX_MESSAGE_PROCESSOR_HPP_

#include <map>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <glog/logging.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>

#include "parameter_manager.hpp"

using namespace gnss_comm;

class UbloxMessageProcessor
{
    public:
        UbloxMessageProcessor(ros::NodeHandle &nh);
        void process_data(const uint8_t *data, size_t len);

        // check ack message, 1:ack, -1:not-ack, 0:not-received-yet
        static int check_ack(const uint8_t *data, size_t len);
        static int build_config_msg(const std::vector<RcvConfigRecord> &rcv_configs, 
            uint8_t *buff, uint32_t &msg_len);

    private:
        void parse_ion_utc(const uint8_t *data, const size_t header_len);
        TimePulseInfoPtr parse_time_pulse(const uint8_t *msg_data, const uint32_t msg_len);
        PVTSolutionPtr parse_pvt(const uint8_t *msg_data, const uint32_t msg_len);

        std::vector<ObsPtr> parse_meas_msg(const uint8_t *msg_data, const uint32_t msg_len);
        EphemBasePtr parse_subframe(const uint8_t *msg_data, const uint32_t msg_len, std::vector<double> &iono_params);
        int decode_GLO_subframe(const uint8_t *msg_data, const uint32_t msg_len, GloEphemPtr glo_ephem);
        int decode_GLO_ephem(GloEphemPtr glo_ephem);
        int decode_BDS_subframe(const uint8_t *msg_data, const uint32_t msg_len, EphemPtr ephem, std::vector<double> &iono_params);
        int decode_BDS_D2_ephem(EphemPtr ephem);
        int decode_BDS_D1_ephem(EphemPtr ephem, std::vector<double> &iono_params);
        int decode_GAL_subframe(const uint8_t *msg_data, const uint32_t msg_len, EphemPtr ephem);
        int decode_GAL_ephem(EphemPtr ephem);
        int decode_GPS_subframe(const uint8_t *msg_data, const uint32_t msg_len, EphemPtr ephem);
        int decode_GPS_ephem(EphemPtr ephem);

        /* freq index to frequency ---------------------------------------------------*/
        double sig_freq(const int sys, const int sid, const int fcn);

        /* signal index in obs data --------------------------------------------------*/
        int sig_idx(const int sys, const int code);
        
        /* ubx sigid to signal ([5] Appendix B) --------------------------------------*/
        int ubx_sig(const int sys, const int sigid);

        /* ubx gnssid to system (ref [2] 25) -----------------------------------------*/
        int ubx_sys(const int gnssid);

        /* test hamming code of glonass ephemeris string -------------------------------
         * test hamming code of glonass ephemeris string (ref [2] 4.7)
         * args   : unsigned char *buff I glonass navigation data string bits in frame
         *                                with hamming
         *                                  buff[ 0]: string bit 85-78
         *                                  buff[ 1]: string bit 77-70
         *                                  ...
         *                                  buff[10]: string bit  5- 1 (0 padded)
         * return : status (1:ok,0:error)
         *-----------------------------------------------------------------------------*/
        int test_glostr(const uint8_t *data);

        /* crc-24q parity --------------------------------------------------------------
         * compute crc-24q parity for sbas, rtcm3
         * args   : unsigned char *buff I data
         *          int    len    I      data length (bytes)
         * return : crc-24Q parity
         * notes  : see reference [2] A.4.3.3 Parity
         *-----------------------------------------------------------------------------*/
        uint32_t crc24q(const uint8_t *buff, const uint32_t len) const;

        void setbitu(uint8_t *buff, const uint32_t pos, const uint32_t len, uint32_t data) const;
        uint32_t getbitu(const uint8_t *buff, const uint32_t pos, const uint32_t len) const;
        int getbits(const uint8_t *buff, const uint32_t pos, const uint32_t len) const;
        /* get two component bits ----------------------------------------------------*/
        uint32_t getbitu2(const uint8_t *buff, const uint32_t p1, const uint32_t l1, 
                          const uint32_t p2, const uint32_t l2) const;
        int getbits2(const uint8_t *buff, const uint32_t p1, const uint32_t l1, 
                     const uint32_t p2, const uint32_t l2) const;
        /* get three component bits --------------------------------------------------*/
        uint32_t getbitu3(const uint8_t *buff, const uint32_t p1, const uint32_t l1, 
                          const uint32_t p2, const uint32_t l2, const uint32_t p3, const uint32_t l3) const;
        int getbits3(const uint8_t *buff, const uint32_t p1, const uint32_t l1, const uint32_t p2, 
                     const uint32_t l2, const uint32_t p3, const uint32_t l3) const;
        double getbitg(const uint8_t *buff, const int pos, const int len) const;
        uint32_t merge_two_u(const uint32_t a, const uint32_t b, const uint32_t n) const;
        int merge_two_s(const int a, const uint8_t b, const int n) const;

        static bool check_checksum(const uint8_t *data, const uint32_t size);
        static bool verify_msg(const uint8_t *data, const size_t len);
        static void set_checksum(uint8_t *data, const uint32_t size);

    private:
        double  lock_time_rec[MAX_SAT][N_FREQ]; /* lock time (s) */
        uint8_t halfc_rec[MAX_SAT][N_FREQ];    /* half-cycle add flag */
        uint8_t subfrm[MAX_SAT][380];     /* subframe buffer */
        gtime_t curr_time;
        ros::NodeHandle nh_;
        ros::Publisher pub_pvt_, pub_lla_;
        ros::Publisher pub_tp_info_;
        ros::Publisher pub_range_meas_, pub_ephem_, pub_glo_ephem_, pub_iono_;
        const uint32_t MSG_HEADER_LEN;

        static constexpr uint8_t UBX_SYNC_1 = 0xB5;        // ubx message sync code 1
        static constexpr uint8_t UBX_SYNC_2 = 0x62;        // ubx message sync code 2
        static constexpr uint16_t UBX_ACK_ACK_ID = 0x0501;          // ubx message id: message Acknowledged
        static constexpr uint16_t UBX_ACK_NAK_ID = 0x0500;          // ubx message id: message Not-Acknowledged
        static constexpr uint16_t UBX_CFG_VALSET_ID = 0x068A;       // ubx message id: sets values corresponding to provided key-value pairs within a transaction
        static constexpr uint16_t UBX_RXMSFRBX_ID = 0x0213;      // ubx message id: raw subframe data
        static constexpr uint16_t UBX_RXMRAWX_ID = 0x0215;       // ubx message id: multi-gnss raw meas data
        static constexpr uint16_t UBX_NAVPOS_ID = 0x0107;        // ubx message id: Navigation Position Velocity Time Solution
        static constexpr uint16_t UBX_TIM_TP_ID = 0x0D01;        // ubx message id:  information on the timing of the next pulse
        static constexpr uint16_t UBX_UNKNOWN_ID = 0x0000;        // ubx message id:  unknown or unsupported message

        static constexpr uint32_t UBX_PVT_PAYLOAD_LEN = 92;

        static constexpr uint8_t CPSTD_VALID = 10;         // carrier-phase std threshold for cycle clip detection
        static constexpr uint8_t LLI_SLIP = 0x01;          // LLI: cycle-slip
        static constexpr uint8_t LLI_HALFC = 0x02;         // LLI: half-cycle not resovled
        static constexpr double lam_carr[MAXFREQ]=         // carrier wave length (m)
        {        
            LIGHT_SPEED/FREQ1,LIGHT_SPEED/FREQ2,LIGHT_SPEED/FREQ5,LIGHT_SPEED/FREQ6,LIGHT_SPEED/FREQ7,
            LIGHT_SPEED/FREQ8,LIGHT_SPEED/FREQ9
        };
        static constexpr uint8_t GPS_WEEK_ROLLOVER_N = 2;  // TODO: assuming 2 GPS week rollovers
        static constexpr uint8_t PREAMB_CNAV = 0x8B;      // cnav preamble

        static constexpr double P2_5  = 0.03125;               // 2^-5
        static constexpr double P2_6  = 0.015625;              // 2^-6
        static constexpr double P2_11 = 4.882812500000000E-04; // 2^-11
        static constexpr double P2_15 = 3.051757812500000E-05; // 2^-15
        static constexpr double P2_17 = 7.629394531250000E-06; // 2^-17
        static constexpr double P2_19 = 1.907348632812500E-06; // 2^-19
        static constexpr double P2_20 = 9.536743164062500E-07; // 2^-20
        static constexpr double P2_21 = 4.768371582031250E-07; // 2^-21
        static constexpr double P2_23 = 1.192092895507810E-07; // 2^-23
        static constexpr double P2_24 = 5.960464477539063E-08; // 2^-24
        static constexpr double P2_27 = 7.450580596923828E-09; // 2^-27
        static constexpr double P2_29 = 1.862645149230957E-09; // 2^-29
        static constexpr double P2_30 = 9.313225746154785E-10; // 2^-30
        static constexpr double P2_31 = 4.656612873077393E-10; // 2^-31
        static constexpr double P2_32 = 2.328306436538696E-10; // 2^-32
        static constexpr double P2_33 = 1.164153218269348E-10; // 2^-33
        static constexpr double P2_34 = 5.820766091346740E-11; // 2^-34
        static constexpr double P2_35 = 2.910383045673370E-11; // 2^-35
        static constexpr double P2_38 = 3.637978807091710E-12; // 2^-38
        static constexpr double P2_39 = 1.818989403545856E-12; // 2^-39
        static constexpr double P2_40 = 9.094947017729280E-13; // 2^-40
        static constexpr double P2_43 = 1.136868377216160E-13; // 2^-43
        static constexpr double P2_46 = 1.421085471520200E-14; // 2^-46
        static constexpr double P2_48 = 3.552713678800501E-15; // 2^-48
        static constexpr double P2_50 = 8.881784197001252E-16; // 2^-50
        static constexpr double P2_55 = 2.775557561562891E-17; // 2^-55
        static constexpr double P2_59 = 1.734723475976810E-18; // 2^-59
        static constexpr double P2_66 = 1.355252715606881E-20; // 2^-66

        static constexpr unsigned int tbl_CRC24Q[] = {
            0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
            0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
            0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
            0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
            0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
            0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
            0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
            0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
            0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
            0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
            0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
            0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
            0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
            0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
            0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
            0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
            0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
            0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
            0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
            0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
            0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
            0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
            0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
            0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
            0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
            0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
            0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
            0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
            0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
            0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
            0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
            0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
        };
};

#endif