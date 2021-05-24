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
*
* As most of the ephemeris parsing functions are adapted from RTKLIB, 
* the license for those functions is claimed as follows:
* 
* The RTKLIB software package is distributed under the following BSD 2-clause
* license (http://opensource.org/licenses/BSD-2-Clause) and additional two
* exclusive clauses. Users are permitted to develop, produce or sell their own
* non-commercial or commercial products utilizing, linking or including RTKLIB as
* long as they comply with the license.
* 
*         Copyright (c) 2007-2020, T. Takasu, All rights reserved.
*/

#include "ublox_message_processor.hpp"

constexpr double UbloxMessageProcessor::lam_carr[];
constexpr unsigned int UbloxMessageProcessor::tbl_CRC24Q[];

UbloxMessageProcessor::UbloxMessageProcessor(ros::NodeHandle& nh) : 
    nh_(nh),
    MSG_HEADER_LEN(ParameterManager::getInstance().MSG_HEADER_LEN)
{
    pub_pvt_ = nh_.advertise<GnssPVTSolnMsg>("receiver_pvt", 100);
    pub_lla_ = nh_.advertise<sensor_msgs::NavSatFix>("receiver_lla", 100);
    pub_tp_info_ = nh_.advertise<GnssTimePulseInfoMsg>("time_pulse_info", 100);
    pub_range_meas_ = nh_.advertise<GnssMeasMsg>("range_meas", 100);
    pub_ephem_ = nh_.advertise<GnssEphemMsg>("ephem", 100);
    pub_glo_ephem_ = nh_.advertise<GnssGloEphemMsg>("glo_ephem", 100);
    pub_iono_ = nh_.advertise<StampedFloat64Array>("iono_params", 100);
}

void UbloxMessageProcessor::process_data(const uint8_t *data, size_t len)
{
    if (!verify_msg(data, len))
        return;
    
    const uint16_t msg_type = (data[2]<<8) | data[3];

    if (msg_type == UBX_RXMRAWX_ID)
    {
        std::vector<ObsPtr> meas = parse_meas_msg(data, len);
        if (meas.empty())   return;
        GnssMeasMsg meas_msg = meas2msg(meas);
        pub_range_meas_.publish(meas_msg);
        return;
    }
    else if (msg_type == UBX_RXMSFRBX_ID)
    {
        std::vector<double> iono_params;
        EphemBasePtr ephem = parse_subframe(data, len, iono_params);
        if (ephem && ephem->ttr.time!=0)
        {
            if (satsys(ephem->sat, NULL) == SYS_GLO)
            {
                GnssGloEphemMsg glo_ephem_msg = glo_ephem2msg(std::dynamic_pointer_cast<GloEphem>(ephem));
                pub_glo_ephem_.publish(glo_ephem_msg);
            }
            else
            {
                GnssEphemMsg ephem_msg = ephem2msg(std::dynamic_pointer_cast<Ephem>(ephem));
                pub_ephem_.publish(ephem_msg);
            }
        }
        if (iono_params.size() == 8)
        {
            // publish ionosphere parameters
            StampedFloat64Array iono_msg;
            if (ephem && ephem->ttr.time!=0)
                iono_msg.header.stamp = ros::Time(time2sec(ephem->ttr));
            std::copy(iono_params.begin(), iono_params.end(), std::back_inserter(iono_msg.data));
            pub_iono_.publish(iono_msg);
        }
        return;
    }
    else if (msg_type == UBX_TIM_TP_ID)
    {
        TimePulseInfoPtr tp_info = parse_time_pulse(data, len);
        if (tp_info && tp_info->time.time != 0)
        {
            GnssTimePulseInfoMsg tp_info_msg = tp_info2msg(tp_info);
            pub_tp_info_.publish(tp_info_msg);
        }
        return;
    }
    else if (msg_type == UBX_NAVPOS_ID)
    {
        PVTSolutionPtr pvt_soln = parse_pvt(data, len);
        if (!pvt_soln || pvt_soln->time.time == 0)    return;

        // add RTK station offset if appliable
        if (pvt_soln->carr_soln == 1 || pvt_soln->carr_soln == 2)
        {
            Eigen::Vector3d pvt_lla(pvt_soln->lat, pvt_soln->lon, pvt_soln->hgt);
            Eigen::Vector3d pvt_ecef = geo2ecef(pvt_lla);
            pvt_ecef += ParameterManager::getInstance().rtk_correction_ecef;
            pvt_lla = ecef2geo(pvt_ecef);
            pvt_soln->lat = pvt_lla.x();
            pvt_soln->lon = pvt_lla.y();
            pvt_soln->hgt = pvt_lla.z();
        }

        GnssPVTSolnMsg pvt_msg = pvt2msg(pvt_soln);
        pub_pvt_.publish(pvt_msg);

        sensor_msgs::NavSatFix lla_msg;
        lla_msg.header.stamp = ros::Time(time2sec(pvt_soln->time));
        lla_msg.latitude    = pvt_soln->lat;
        lla_msg.longitude   = pvt_soln->lon;
        lla_msg.altitude    = pvt_soln->hgt;
        lla_msg.status.status = static_cast<int8_t>(pvt_soln->fix_type);
        lla_msg.status.service = static_cast<uint16_t>(pvt_soln->carr_soln);
        pub_lla_.publish(lla_msg);
        return;
    }
    // unsupported message reach here
    return;
}

int UbloxMessageProcessor::check_ack(const uint8_t *data, size_t len)
{
    if (!verify_msg(data, len))
        return 0;
    const uint16_t msg_type = (data[2]<<8) | data[3];
    if (msg_type == UBX_ACK_ACK_ID)
        return 1;
    else if (msg_type == UBX_ACK_NAK_ID)
        return -1;
    
    // ignore other messages
    return 0;
}


bool UbloxMessageProcessor::verify_msg(const uint8_t *data, const size_t len)
{
    if (len < 8)  // ubx header length is 6, checksum length is 2
    {
        LOG(ERROR) << "Invalid message length ";
        return false;
    }

    if (data[0] != 0xB5 || data[1] != 0x62)
    {
        LOG(ERROR) << "Invalid ublox message preamble.";
        return false;
    }

    uint16_t payload_len = *reinterpret_cast<const uint16_t*>(data+4);
    if (len != payload_len+ParameterManager::getInstance().MSG_HEADER_LEN+2)
    {
        LOG(ERROR) << "Invalid message length.";
        return false;
    }

    if (!check_checksum(data, len))
    {
        LOG(ERROR) << "Invalid checksum.\n";
        return false;
    }

    return true;
}

void UbloxMessageProcessor::parse_ion_utc(const uint8_t *data, const size_t header_len)
{
    // TODO: parse ionosphere and utc parameters
    return;
}

TimePulseInfoPtr UbloxMessageProcessor::parse_time_pulse(const uint8_t *msg_data, const uint32_t msg_len)
{
    TimePulseInfoPtr tp_info;
    if (msg_len != 24)          // header(6) + payload(16) + checksum(2)
    {
        LOG(ERROR) << "UBX-TIM-TP message length error. len=" << msg_len << " rather than " << 24;
        return tp_info;
    }
    tp_info.reset(new TimePulseInfo());
    const uint8_t *p = msg_data + 6;      // skip header
    double tow_ms = *reinterpret_cast<const uint32_t*>(p);
    double tow_sub_ms = *reinterpret_cast<const uint32_t*>(p+4);
    double tow = tow_ms * 1.0e-3 + tow_sub_ms * 1.0e-3 * std::pow(2, -32);
    uint16_t week = *reinterpret_cast<const uint16_t*>(p+12);
    tp_info->time = gpst2time(week, tow);
    uint8_t flags = *(p+14);
    tp_info->utc_based = flags & 0x01;
    uint8_t ref_info = *(p+15);
    uint8_t ref_system = ref_info & 0x0F;
    switch (ref_system)
    {
        case SYS_GPS:
            tp_info->time_sys = SYS_GPS;
            break;
        case SYS_BDS:
            tp_info->time_sys = SYS_BDS;
            break;
        case SYS_GLO:
            tp_info->time_sys = SYS_GLO;
            break;
        case SYS_GAL:
            tp_info->time_sys = SYS_GAL;
            break;
        default:
            tp_info->time_sys = SYS_NONE;
            break;
    }

    return tp_info;
}

PVTSolutionPtr UbloxMessageProcessor::parse_pvt(const uint8_t *msg_data, const uint32_t msg_len)
{
    PVTSolutionPtr pvt_soln;

    if (msg_len != MSG_HEADER_LEN+UBX_PVT_PAYLOAD_LEN+2)     // header(6) + payload(92) + checksum(2)
    {
        LOG(ERROR) << "ubx nav-pvt message length error. len=" << msg_len;
        return pvt_soln;
    }
    if (curr_time.time == 0)    return pvt_soln;
    const uint8_t *p = msg_data+6;
    uint32_t itow = *reinterpret_cast<const uint32_t*>(p);
    uint32_t week = 0;
    double tow = time2gpst(curr_time, &week);
    if      (itow*1e-3 > tow+302400.0)    --week;
    else if (itow*1e-3 < tow-302400.0)    ++week;
    gtime_t time = gpst2time(week, itow*1e-3);
    if (false && (p[11]&0x04) && ((p[22]&0xF0) == 0xE0))    // skip UTC time check
    {
        // check time 
        double ep[6];
        ep[0] = static_cast<double>((p[5]<<8)+p[4]);
        ep[1] = static_cast<double>(p[6]);
        ep[2] = static_cast<double>(p[7]);
        ep[3] = static_cast<double>(p[8]);
        ep[4] = static_cast<double>(p[9]);
        ep[5] = static_cast<double>(p[10]) + (*reinterpret_cast<const int32_t*>(p+16))*1e-9;
        gtime_t time_utc = epoch2time(ep);
        gtime_t time_gpst = utc2gpst(time_utc);
        LOG(INFO) << "time_diff is " << std::setprecision(20) << time_diff(time_gpst, time);
        LOG_IF(FATAL, time2sec(time) != time2sec(time_gpst)) << "Time mismatch";
    }

    pvt_soln.reset(new PVTSolution());
    pvt_soln->fix_type = p[20];
    uint8_t flags = p[21];
    pvt_soln->valid_fix = static_cast<bool>(flags&0x01);
    pvt_soln->diff_soln = static_cast<bool>((flags&0x02)>>1);
    pvt_soln->carr_soln = (flags&0xC0)>>6;
    pvt_soln->num_sv = p[23];
    pvt_soln->lat = (*reinterpret_cast<const int32_t*>(p+28)) * 1e-7;
    pvt_soln->lon = (*reinterpret_cast<const int32_t*>(p+24)) * 1e-7;
    pvt_soln->hgt = (*reinterpret_cast<const int32_t*>(p+32)) * 1e-3;
    pvt_soln->hgt_msl = (*reinterpret_cast<const int32_t*>(p+36)) * 1e-3;
    pvt_soln->h_acc = (*reinterpret_cast<const uint32_t*>(p+40)) * 1e-3;
    pvt_soln->v_acc = (*reinterpret_cast<const uint32_t*>(p+44)) * 1e-3;
    pvt_soln->p_dop = (*reinterpret_cast<const uint16_t*>(p+76)) * 1e-2;
    pvt_soln->vel_n = (*reinterpret_cast<const int32_t*>(p+48)) * 1e-3;
    pvt_soln->vel_e = (*reinterpret_cast<const int32_t*>(p+52)) * 1e-3;
    pvt_soln->vel_d = (*reinterpret_cast<const int32_t*>(p+56)) * 1e-3;
    pvt_soln->vel_acc = (*reinterpret_cast<const uint32_t*>(p+68)) * 1e-3;
    pvt_soln->time = time;
    return pvt_soln;
}

std::vector<ObsPtr> UbloxMessageProcessor::parse_meas_msg(const uint8_t *msg_data, const uint32_t msg_len)
{
    std::vector<ObsPtr> obs_meas;
    if (msg_len < 24)
    {
        LOG(ERROR) << "ubx rxmrawx length error: len=" << msg_len;
        return obs_meas;
    }

    const uint8_t *p = msg_data + 6;      // skip header
    double tow = *reinterpret_cast<const double*>(p);
    uint16_t week = *reinterpret_cast<const uint16_t*>(p+8);
    uint8_t num_meas = p[11];

    if (msg_len < static_cast<uint32_t>(24+32*num_meas))
    {
        LOG(ERROR) << "ubx rxmrawx length error: len=" << msg_len << " num_meas=" << num_meas;
        return obs_meas;
    }
    if (week == 0)
    {
        LOG(ERROR) << "ubx rxmrawx week=0 error";
        return obs_meas;
    }
    gtime_t time = gpst2time(week, tow);
    std::map<uint32_t, uint32_t> sat2idx;

    p += 16;
    for (size_t i=0; i < num_meas; ++i, p+=32)
    {
        double psr = *reinterpret_cast<const double*>(p);
        double cp  = *reinterpret_cast<const double*>(p+8);
        float dopp = *reinterpret_cast<const float*>(p+16);
        uint8_t gnss_id = p[20];
        uint8_t svid = p[21];
        uint8_t sig_id = p[22];
        uint8_t freq_id = p[23];
        uint16_t lock_time = *reinterpret_cast<const uint16_t*>(p+24);
        uint8_t cn0 = p[26];
        uint8_t psr_std = p[27] & 0x0F;
        uint8_t cp_std = p[28] & 0x0F;
        uint8_t dopp_std = p[29] & 0x0F;
        uint8_t tck_sta = p[30];
        if (!(tck_sta & 0x01))  psr = 0.0;        // pseudo-range invalid
        if (!(tck_sta & 0x02) || cp == -0.5 || cp_std > CPSTD_VALID)    cp = 0.0;    // carrier phase invalid
        uint32_t sys;
        if (!(sys = ubx_sys(gnss_id)))
        {
            // LOG(WARNING) << "ubx rxmrawx stallite system not supported. GNSS id: " << static_cast<int>(gnss_id);
            continue;
        }
        uint32_t sat;
        if (!(sat = sat_no(sys, svid)))
        {
            LOG(ERROR) << "ubx rxmrawx satellite number error. sys: " << sys << " prn: " << static_cast<int>(svid);
            continue;
        }
        int code = ubx_sig(sys, sig_id);
        int sid = sig_idx(sys, code);       // signal index in obs data
        if (sid == 0 || sid > N_FREQ)
        {
            LOG(ERROR) << "ubx rxmrawx signal error: sat=" << sat << " signal_id=" << sid;
            continue;
        }

        uint8_t halfv = (tck_sta & 0x04) ? 1 : 0;
        uint8_t halfc = (tck_sta & 0x08) ? 1 : 0;
        uint8_t slip  = lock_time == 0 || lock_time*1e-3 < lock_time_rec[sat-1][sid-1] ||
                        halfc != halfc_rec[sat-1][sid-1];
        lock_time_rec[sat-1][sid-1] = lock_time * 1e-3;
        halfc_rec[sat-1][sid-1] = halfc;
        uint8_t LLI = (slip ? LLI_SLIP : 0) | (!halfv ? LLI_HALFC : 0);
        if (sat2idx.count(sat) == 0)
        {
            ObsPtr obs(new Obs());
            obs->time = time;
            obs->sat = sat;
            obs_meas.emplace_back(obs);
            sat2idx.emplace(sat, obs_meas.size()-1);
        }
        ObsPtr obs = obs_meas[sat2idx.at(sat)];
        double freq = sig_freq(sys, sid, static_cast<int>(freq_id)-7);
        obs->freqs.push_back(freq);
        obs->CN0.push_back(cn0);
        obs->LLI.push_back(LLI);
        obs->code.push_back(code);
        obs->psr.push_back(psr);
        obs->psr_std.push_back(0.01*(1<<psr_std));
        obs->cp.push_back(cp);
        obs->cp_std.push_back(0.004*cp_std);
        obs->dopp.push_back(dopp);
        obs->dopp_std.push_back(0.002*(1<<dopp_std));
        obs->status.push_back(tck_sta&0x0F);
    }
    if (!obs_meas.empty())   curr_time = time;   // update current time
    return obs_meas;
}

EphemBasePtr UbloxMessageProcessor::parse_subframe(const uint8_t *msg_data, const uint32_t msg_len, std::vector<double> &iono_params)
{
    EphemBasePtr ephem;
    uint32_t sys = 0;
    if (!(sys=ubx_sys(msg_data[6])))
    {
        // LOG(ERROR) << "ubx rxmsfrbx not supported satellite system : " << int(msg_data[6]);
        return ephem;
    }
    uint8_t prn = msg_data[7];
    uint32_t sat = 0;
    if (!(sat=sat_no(sys, prn)))
    {
        LOG(ERROR) << "ubx rxmsfrbx sat number error: sys=" << sys << " prn=" << int(prn);
        return ephem;
    }
    switch (sys)
    {
        case SYS_GPS:
            ephem.reset(new Ephem());
            ephem->sat = sat;
            ephem->ttr.time = 0;
            decode_GPS_subframe(msg_data, msg_len, std::dynamic_pointer_cast<Ephem>(ephem));
            break;
        case SYS_BDS:
            ephem.reset(new Ephem());
            ephem->sat = sat;
            ephem->ttr.time = 0;
            decode_BDS_subframe(msg_data, msg_len, std::dynamic_pointer_cast<Ephem>(ephem), iono_params);
            break;
        case SYS_GAL:
            ephem.reset(new Ephem());
            ephem->sat = sat;
            ephem->ttr.time = 0;
            decode_GAL_subframe(msg_data, msg_len, std::dynamic_pointer_cast<Ephem>(ephem));
            break;
        case SYS_GLO:
            ephem.reset(new GloEphem());
            ephem->sat = sat;
            ephem->ttr.time = 0;
            decode_GLO_subframe(msg_data, msg_len, std::dynamic_pointer_cast<GloEphem>(ephem));
            break;
    }
    return ephem;
}

int UbloxMessageProcessor::decode_GLO_subframe(const uint8_t *msg_data, const uint32_t msg_len, GloEphemPtr glo_ephem)
{
    int i,j,k,m;
    const uint8_t *p=msg_data+14;
    uint8_t buff[64],*fid;
    
    if (msg_len<32)
    {
        LOG(ERROR) << "ubx rawsfrbx gnav length error: len=" << msg_len;
        return -1;
    }
    glo_ephem->freqo = msg_data[9]-7;
    for (i=k=0;i<4;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    /* test hamming of glonass string */
    if (!test_glostr(buff)) 
    {
        LOG(ERROR) << "ubx rawsfrbx glo string hamming error: sat=" << glo_ephem->sat;
        return -1;
    }
    m=getbitu(buff,1,4);
    if (m<1||15<m) 
    {
        LOG(ERROR) << "ubx rawsfrbx glo string no error: sat=" << glo_ephem->sat;
        return -1;
    }
    /* flush frame buffer if frame-id changed */
    fid=subfrm[glo_ephem->sat-1]+150;
    if (fid[0]!=buff[12]||fid[1]!=buff[13]) {
        for (i=0;i<4;i++) memset(subfrm[glo_ephem->sat-1]+i*10,0,10);
        memcpy(fid,buff+12,2); /* save frame-id */
    }
    memcpy(subfrm[glo_ephem->sat-1]+(m-1)*10,buff,10);
    
    if (m!=4) return 0;
    
    /* decode glonass ephemeris strings */
    decode_GLO_ephem(glo_ephem);
    return 0;
}

int UbloxMessageProcessor::decode_GLO_ephem(GloEphemPtr glo_ephem)
{
    double tow,tod,tof,toe;
    int tk_h,tk_m,tk_s,tb,slot;
    uint32_t week;
    int i=1,frn1,frn2,frn3,frn4;
    
    // LOG(INFO) << "decode_glostr";
    uint8_t *buff = subfrm[glo_ephem->sat-1];
    /* frame 1 */
    frn1                =getbitu(buff,i, 4);           i+= 4+2;
                         getbitu(buff,i, 2);           i+= 2;       // skip
    tk_h                =getbitu(buff,i, 5);           i+= 5;
    tk_m                =getbitu(buff,i, 6);           i+= 6;
    tk_s                =getbitu(buff,i, 1)*30;        i+= 1;
    glo_ephem->vel[0]   =getbitg(buff,i,24)*P2_20*1E3; i+=24;
    glo_ephem->acc[0]   =getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
    glo_ephem->pos[0]   =getbitg(buff,i,27)*P2_11*1E3; i+=27+4;
    
    /* frame 2 */
    frn2                =getbitu(buff,i, 4);           i+= 4;
    glo_ephem->health   =getbitu(buff,i, 3);           i+= 3;
                         getbitu(buff,i, 1);           i+= 1;       // skip
    tb                  =getbitu(buff,i, 7);           i+= 7+5;
    glo_ephem->vel[1]   =getbitg(buff,i,24)*P2_20*1E3; i+=24;
    glo_ephem->acc[1]   =getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
    glo_ephem->pos[1]   =getbitg(buff,i,27)*P2_11*1E3; i+=27+4;
    
    /* frame 3 */
    frn3                =getbitu(buff,i, 4);           i+= 4;
                         getbitu(buff,i, 1);           i+= 1;       // skip
    glo_ephem->gamma    =getbitg(buff,i,11)*P2_40;     i+=11+1;
                         getbitu(buff,i, 2);           i+= 2;       // skip
                         getbitu(buff,i, 1);           i+= 1;       // skip
    glo_ephem->vel[2]   =getbitg(buff,i,24)*P2_20*1E3; i+=24;
    glo_ephem->acc[2]   =getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
    glo_ephem->pos[2]   =getbitg(buff,i,27)*P2_11*1E3; i+=27+4;
    
    /* frame 4 */
    frn4                    =getbitu(buff,i, 4);           i+= 4;
    glo_ephem->tau_n        =getbitg(buff,i,22)*P2_30;     i+=22;
    glo_ephem->delta_tau_n  =getbitg(buff,i, 5)*P2_30;     i+= 5;
    glo_ephem->age          =getbitu(buff,i, 5);           i+= 5+14;
                             getbitu(buff,i, 1);           i+= 1;   // skip
    uint32_t ft             =getbitu(buff,i, 4);           i+= 4+3;
                             getbitu(buff,i,11);           i+=11;   // skip
    slot                    =getbitu(buff,i, 5);           i+= 5;
                             getbitu(buff,i, 2);                    // skip
    
    if (ft == 2)        glo_ephem->ura = 2.5;   
    else if (ft < 5)    glo_ephem->ura = ft+1.0;
    else if (ft == 5)   glo_ephem->ura = 7.0;
    else if (ft < 10)   glo_ephem->ura = 10.0+(ft-6)*2.0;
    else if (ft < 15)   glo_ephem->ura = 1 << (ft-5);
    else                glo_ephem->ura = -1.0;

    if (frn1!=1||frn2!=2||frn3!=3||frn4!=4) 
    {
        // LOG(ERROR) << "decode_GLO_ephem error: frn=" << frn1 << ' ' 
        //            << frn2 << ' ' << frn3 << ' ' << frn4;
        return -1;
    }
    if (glo_ephem->sat != sat_no(SYS_GLO,slot)) 
    {
        LOG(ERROR) << "decode_GLO_ephem error: slot=" << slot;
        return -1;
    }
    glo_ephem->iode=tb;
    tow=time2gpst(gpst2utc(curr_time),&week);
    tod=fmod(tow,86400.0); tow-=tod;
    tof=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
    if      (tof<tod-43200.0) tof+=86400.0;
    else if (tof>tod+43200.0) tof-=86400.0;
    glo_ephem->ttr=utc2gpst(gpst2time(week,tow+tof));
    toe=tb*900.0-10800.0; /* lt->utc */
    if      (toe<tod-43200.0) toe+=86400.0;
    else if (toe>tod+43200.0) toe-=86400.0;
    glo_ephem->toe=utc2gpst(gpst2time(week,tow+toe)); /* utc->gpst */
    return 0;
}

int UbloxMessageProcessor::decode_BDS_subframe(const uint8_t *msg_data, const uint32_t msg_len, EphemPtr ephem, std::vector<double> &iono_params)
{
    uint32_t words[10];
    uint32_t prn;
    int i,id,pgn;
    const uint8_t *p=msg_data+14;
    
    if (msg_len < 56) 
    {
        LOG(INFO) << "ubx rawsfrbx length error: sat=" << ephem->sat << " len=" << msg_len;
        return -1;
    }
    for (i=0;i<10;i++,p+=4) words[i]=(*reinterpret_cast<const uint32_t*>(p))&0x3FFFFFFF; /* 30 bits */
    
    satsys(ephem->sat,&prn);
    id=(words[0]>>12)&0x07; /* subframe id (3bit) */
    if (id<1||5<id) 
    {
        LOG(INFO) << "ubx rawsfrbx subfrm id error: sat=" << ephem->sat;
        return -1;
    }
    if (prn>5&&prn<59) { /* IGSO/MEO */
        
        for (i=0;i<10;i++) {
            setbitu(subfrm[ephem->sat-1]+(id-1)*38,i*30,30,words[i]);
        }
        if (id!=3) return -1;
        
        /* decode beidou D1 ephemeris */
        decode_BDS_D1_ephem(ephem, iono_params);
    }
    else { /* GEO (C01-05, C59-63) */
        if (id!=1) return -1;
        
        /* subframe 1 */
        pgn=(words[1]>>14)&0x0F; /* page number (4bit) */
        if (pgn<1||10<pgn) {
            LOG(INFO) << "ubx rawsfrbx page number error: sat=" << ephem->sat;
            return -1;
        }
        for (i=0;i<10;i++) {
            setbitu(subfrm[ephem->sat-1]+(pgn-1)*38,i*30,30,words[i]);
        }
        if (pgn!=10) return -1;
        
        /* decode beidou D2 ephemeris */
        decode_BDS_D2_ephem(ephem);
    }
    return 0;
}

int UbloxMessageProcessor::decode_BDS_D2_ephem(EphemPtr ephem)
{
    double toc_bds,sqrtA;
    uint32_t f1p4,cucp5,ep6,cicp7,i0p8,OMGdp9,omgp10;
    uint32_t bdt_week,toes,sow1,sow3,sow4,sow5,sow6,sow7,sow8,sow9,sow10;
    int i,f1p3,cucp4,ep5,cicp6,i0p7,OMGdp8,omgp9;
    int pgn1,pgn3,pgn4,pgn5,pgn6,pgn7,pgn8,pgn9,pgn10;

    // LOG(INFO) << "decode_bds_d2";
    uint8_t *buff = subfrm[ephem->sat-1];
    i=8*38*0; /* page 1 */
    pgn1            =getbitu (buff,i+ 42, 4);
    sow1            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    ephem->health   =getbitu (buff,i+ 46, 1); /* SatH1 */
    ephem->iodc     =getbitu (buff,i+ 47, 5); /* AODC */
    uint8_t urai    =getbitu (buff,i+ 60, 4);
    ephem->ura      = (urai < 6) ? pow(2, urai/2.0+1) : (1<<(urai-2));
    bdt_week        =getbitu (buff,i+ 64,13); /* week in BDT */
    toc_bds         =getbitu2(buff,i+ 77, 5,i+ 90,12)*8.0;
    ephem->tgd[0]   =getbits (buff,i+102,10)*0.1*1E-9;
    ephem->tgd[1]   =getbits (buff,i+120,10)*0.1*1E-9;
    
    i=8*38*2; /* page 3 */
    pgn3            =getbitu (buff,i+ 42, 4);
    sow3            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    ephem->af0      =getbits2(buff,i+100,12,i+120,12)*P2_33;
    f1p3            =getbits (buff,i+132,4);
    
    i=8*38*3; /* page 4 */
    pgn4            =getbitu (buff,i+ 42, 4);
    sow4            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    f1p4            =getbitu2(buff,i+ 46, 6,i+ 60,12);
    ephem->af2      =getbits2(buff,i+ 72,10,i+ 90, 1)*P2_66;
    ephem->iode     =getbitu (buff,i+ 91, 5); /* AODE */
    ephem->delta_n  =getbits (buff,i+ 96,16)*P2_43*SC2RAD;
    cucp4           =getbits (buff,i+120,14);
    
    i=8*38*4; /* page 5 */
    pgn5            =getbitu (buff,i+ 42, 4);
    sow5            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    cucp5           =getbitu (buff,i+ 46, 4);
    ephem->M0       =getbits3(buff,i+ 50, 2,i+ 60,22,i+ 90, 8)*P2_31*SC2RAD;
    ephem->cus      =getbits2(buff,i+ 98,14,i+120, 4)*P2_31;
    ep5             =getbits (buff,i+124,10);
    
    i=8*38*5; /* page 6 */
    pgn6            =getbitu (buff,i+ 42, 4);
    sow6            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    ep6             =getbitu2(buff,i+ 46, 6,i+ 60,16);
    sqrtA           =getbitu3(buff,i+ 76, 6,i+ 90,22,i+120,4)*P2_19;
    cicp6           =getbits (buff,i+124,10);
    ephem->A        =sqrtA*sqrtA;
    
    i=8*38*6; /* page 7 */
    pgn7            =getbitu (buff,i+ 42, 4);
    sow7            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    cicp7           =getbitu2(buff,i+ 46, 6,i+ 60, 2);
    ephem->cis      =getbits (buff,i+ 62,18)*P2_31;
    toes            =getbitu2(buff,i+ 80, 2,i+ 90,15)*8.0;
    i0p7            =getbits2(buff,i+105, 7,i+120,14);
    
    i=8*38*7; /* page 8 */
    pgn8            =getbitu (buff,i+ 42, 4);
    sow8            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    i0p8            =getbitu2(buff,i+ 46, 6,i+ 60, 5);
    ephem->crc      =getbits2(buff,i+ 65,17,i+ 90, 1)*P2_6;
    ephem->crs      =getbits (buff,i+ 91,18)*P2_6;
    OMGdp8          =getbits2(buff,i+109, 3,i+120,16);
    
    i=8*38*8; /* page 9 */
    pgn9            =getbitu (buff,i+ 42, 4);
    sow9            =getbitu2(buff,i+ 18, 8,i+ 30,12);
    OMGdp9          =getbitu (buff,i+ 46, 5);
    ephem->OMG0     =getbits3(buff,i+ 51, 1,i+ 60,22,i+ 90, 9)*P2_31*SC2RAD;
    omgp9           =getbits2(buff,i+ 99,13,i+120,14);
    
    i=8*38*9; /* page 10 */
    pgn10           =getbitu (buff,i+ 42, 4);
    sow10           =getbitu2(buff,i+ 18, 8,i+ 30,12);
    omgp10          =getbitu (buff,i+ 46, 5);
    ephem->i_dot    =getbits2(buff,i+ 51, 1,i+ 60,13)*P2_43*SC2RAD;
    
    /* check consistency of page numbers, sows and toe/toc */
    if (pgn1!=1||pgn3!=3||pgn4!=4||pgn5!=5||pgn6!=6||pgn7!=7||pgn8!=8||pgn9!=9||
        pgn10!=10) {
        LOG(ERROR) << "decode_bds_d2 error: pgn=" << pgn1 << ' ' << pgn3 << ' '
                   << pgn4 << ' ' << pgn5 << ' ' << pgn6 << ' ' << pgn7 << ' ' 
                   << pgn8 << ' ' << pgn9 << ' ' << pgn10;
        return -1;
    }
    if (sow3!=sow1+6||sow4!=sow3+3||sow5!=sow4+3||sow6!=sow5+3||
        sow7!=sow6+3||sow8!=sow7+3||sow9!=sow8+3||sow10!=sow9+3) {
        LOG(ERROR) << "decode_bds_d2 error: sow=" << sow1 << ' ' << sow3 << ' '
                   << sow4 << ' ' << sow5 << ' ' << sow6 << ' ' << sow7 << ' ' 
                   << sow8 << ' ' << sow9 << ' ' << sow10;
        return -1;
    }
    if (toc_bds!=toes) {
        LOG(ERROR) << "decode_bds_d2 error: toe=" << toes << " toc=" << toc_bds;
        return -1;
    }
    ephem->af1      = merge_two_s(f1p3  ,f1p4  ,18)*P2_50;
    ephem->cuc      = merge_two_s(cucp4 ,cucp5 , 4)*P2_31;
    ephem->e        = merge_two_s(ep5   ,ep6   ,22)*P2_33;
    ephem->cic      = merge_two_s(cicp6 ,cicp7 , 8)*P2_31;
    ephem->i0       = merge_two_s(i0p7  ,i0p8  ,11)*P2_31*SC2RAD;
    ephem->OMG_dot  = merge_two_s(OMGdp8,OMGdp9, 5)*P2_43*SC2RAD;
    ephem->omg      = merge_two_s(omgp9 ,omgp10, 5)*P2_31*SC2RAD;
    
    ephem->ttr      = bdt2time(bdt_week,sow1);
    ephem->ttr      = time_add(ephem->ttr, 14.0);
    if      (toes>sow1+302400.0) bdt_week++;
    else if (toes<sow1-302400.0) bdt_week--;
    ephem->toe      = bdt2time(bdt_week,toes);
    ephem->toe      = time_add(ephem->toe, 14.0);
    ephem->toe_tow  = time2gpst(ephem->toe, &ephem->week);
    ephem->toc      = bdt2time(bdt_week,toc_bds);
    ephem->toc      = time_add(ephem->toc, 14.0);
    ephem->code     = 0; /* data source = unknown */
    curr_time       = ephem->ttr;      // update current time
    return 0;
}

int UbloxMessageProcessor::decode_BDS_D1_ephem(EphemPtr ephem, std::vector<double> &iono_params)
{
    std::vector<double> tmp_iono_params;
    double toc_bds,sqrtA;
    uint32_t bdt_week,toes,toe1,toe2,sow1,sow2,sow3;
    int i,frn1,frn2,frn3;
    // LOG(INFO) << "decode_bds_d1";
    uint8_t *buff = subfrm[ephem->sat-1];
    i=8*38*0; /* subframe 1 */
    frn1            =getbitu (buff,i+ 15, 3);
    sow1            =getbitu2(buff,i+ 18, 8,i+30,12);
    ephem->health   =getbitu (buff,i+ 42, 1); /* SatH1 */
    ephem->iodc     =getbitu (buff,i+ 43, 5); /* AODC */
    uint8_t urai    =getbitu (buff,i+ 48, 4);
    ephem->ura      = (urai < 6) ? pow(2, urai/2.0+1) : (1<<(urai-2));
    bdt_week        =getbitu (buff,i+ 60,13); /* week in BDT */
    toc_bds         =getbitu2(buff,i+ 73, 9,i+ 90, 8)*8.0;
    ephem->tgd[0]   =getbits (buff,i+ 98,10)*0.1*1E-9;
    ephem->tgd[1]   =getbits2(buff,i+108, 4,i+120, 6)*0.1*1E-9;
    tmp_iono_params.push_back(getbits (buff, i+126, 8)*P2_30);
    tmp_iono_params.push_back(getbits (buff, i+134, 8)*P2_27);
    tmp_iono_params.push_back(getbits (buff, i+150, 8)*P2_24);
    tmp_iono_params.push_back(getbits (buff, i+158, 8)*P2_24);
    tmp_iono_params.push_back(getbits2(buff,i+166, 6,i+180, 2)*(1<<11));
    tmp_iono_params.push_back(getbits (buff, i+182, 8)*(1<<14));
    tmp_iono_params.push_back(getbits (buff, i+190, 8)*(1<<16));
    tmp_iono_params.push_back(getbits2(buff,i+198, 4,i+210, 4)*(1<<16));
    ephem->af2      =getbits (buff,i+214,11)*P2_66;
    ephem->af0      =getbits2(buff,i+225, 7,i+240,17)*P2_33;
    ephem->af1      =getbits2(buff,i+257, 5,i+270,17)*P2_50;
    ephem->iode     =getbitu (buff,i+287, 5); /* AODE */
    
    i=8*38*1; /* subframe 2 */
    frn2            =getbitu (buff,i+ 15, 3);
    sow2            =getbitu2(buff,i+ 18, 8,i+30,12);
    ephem->delta_n  =getbits2(buff,i+ 42,10,i+ 60, 6)*P2_43*SC2RAD;
    ephem->cuc      =getbits2(buff,i+ 66,16,i+ 90, 2)*P2_31;
    ephem->M0       =getbits2(buff,i+ 92,20,i+120,12)*P2_31*SC2RAD;
    ephem->e        =getbitu2(buff,i+132,10,i+150,22)*P2_33;
    ephem->cus      =getbits (buff,i+180,18)*P2_31;
    ephem->crc      =getbits2(buff,i+198, 4,i+210,14)*P2_6;
    ephem->crs      =getbits2(buff,i+224, 8,i+240,10)*P2_6;
    sqrtA           =getbitu2(buff,i+250,12,i+270,20)*P2_19;
    toe1            =getbitu (buff,i+290, 2); /* TOE 2-MSB */
    ephem->A        =sqrtA*sqrtA;
    
    i=8*38*2; /* subframe 3 */
    frn3            =getbitu (buff,i+ 15, 3);
    sow3            =getbitu2(buff,i+ 18, 8,i+30,12);
    toe2            =getbitu2(buff,i+ 42,10,i+ 60, 5); /* TOE 5-LSB */
    ephem->i0       =getbits2(buff,i+ 65,17,i+ 90,15)*P2_31*SC2RAD;
    ephem->cic      =getbits2(buff,i+105, 7,i+120,11)*P2_31;
    ephem->OMG_dot  =getbits2(buff,i+131,11,i+150,13)*P2_43*SC2RAD;
    ephem->cis      =getbits2(buff,i+163, 9,i+180, 9)*P2_31;
    ephem->i_dot    =getbits2(buff,i+189,13,i+210, 1)*P2_43*SC2RAD;
    ephem->OMG0     =getbits2(buff,i+211,21,i+240,11)*P2_31*SC2RAD;
    ephem->omg      =getbits2(buff,i+251,11,i+270,21)*P2_31*SC2RAD;
    toes            =merge_two_u(toe1,toe2,15)*8.0;
    
    /* check consistency of subframe numbers, sows and toe/toc */
    if (frn1!=1||frn2!=2||frn3!=3) 
    {
        LOG(ERROR) << "decode_bds_d1 error: frn=" << frn1 << ' ' << frn2 << ' ' << frn3;
        return -1;
    }
    if (sow2!=sow1+6||sow3!=sow2+6) 
    {
        // LOG(ERROR) << "decode_bds_d1 error: sow=" << sow1 << ' ' << sow2 << ' ' << sow3;
        return -1;
    }
    if (toc_bds!=toes) 
    {
        LOG(ERROR) << "decode_bds_d1 error: toe=" << toes << " toc=" << toc_bds;
        return -1;
    }
    ephem->ttr      = bdt2time(bdt_week,sow1);
    ephem->ttr      = time_add(ephem->ttr, 14.0);
    if      (toes>sow1+302400.0) bdt_week--;
    else if (toes<sow1-302400.0) bdt_week++;
    ephem->toe      = bdt2time(bdt_week,toes);
    ephem->toe      = time_add(ephem->toe, 14.0);
    ephem->toe_tow  = time2gpst(ephem->toe, &(ephem->week));
    ephem->toc      = bdt2time(bdt_week,toc_bds);
    ephem->toc      = time_add(ephem->toc, 14.0);
    ephem->code     = 0; /* data source = unknown */
    curr_time       = ephem->ttr;      // update current time
    iono_params.swap(tmp_iono_params);
    return 0;
}

int UbloxMessageProcessor::decode_GAL_subframe(const uint8_t *msg_data, const uint32_t msg_len, EphemPtr ephem)
{
    const uint8_t *p = msg_data + 14;
    uint8_t buff[32],crc_buff[26]={0};
    int i,j,k,part1,page1,part2,page2,type;
    
    if (msg_len < 52)      // header(6) + payload_1(8) + payload_2(36) + checksum(2)
    {
        LOG(ERROR) << "ubx rxmsfrbx length error: sat=" << sat2str(ephem->sat) << " len=" << msg_len;
        return -1;
    }
    for (i=k=0;i<8;i++,p+=4) 
        for (j=0;j<4;j++) 
            buff[k++]=p[3-j];
    part1=getbitu(buff   ,0,1);
    page1=getbitu(buff   ,1,1);
    part2=getbitu(buff+16,0,1);
    page2=getbitu(buff+16,1,1);
    
    /* skip alert page */
    if (page1==1||page2==1) return 0;
    
    /* test even-odd parts */
    if (part1!=0||part2!=1) 
    {
        LOG(ERROR) << "ubx rawsfrbx gal page even/odd error: sat=" << ephem->sat;
        return -1;
    }
    /* test crc (4(pad) + 114 + 82 bits) */
    for (i=0,j=  4;i<15;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff   ,i*8,8));
    for (i=0,j=118;i<11;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff+16,i*8,8));
    if (crc24q(crc_buff,25)!=getbitu(buff+16,82,24)) 
    {
        LOG(ERROR) << "ubx rawsfrbx gal page crc error: sat=" << ephem->sat;
        return -1;
    }
    type=getbitu(buff,2,6); /* word type */
    
    /* skip word except for ephemeris, iono, utc parameters */
    if (type>6) return 0;
    
    /* clear word 0-6 flags */
    if (type==2) subfrm[ephem->sat-1][112]=0;
    
    /* save page data (112 + 16 bits) to frame buffer */
    k=type*16;
    for (i=0,j=2;i<14;i++,j+=8) subfrm[ephem->sat-1][k++]=getbitu(buff   ,j,8);
    for (i=0,j=2;i< 2;i++,j+=8) subfrm[ephem->sat-1][k++]=getbitu(buff+16,j,8);
    
    /* test word 0-6 flags */
    subfrm[ephem->sat-1][112]|=(1<<type);
    if (subfrm[ephem->sat-1][112]!=0x7F) return 0;

    decode_GAL_ephem(ephem);
    return 0;
}

int UbloxMessageProcessor::decode_GAL_ephem(EphemPtr ephem)
{
    double tow,toc,tt,sqrtA, toes;
    int i,time_f,week,svid,e5b_hs,e1b_hs,e5b_dvs,e1b_dvs,type[6],iod_nav[4];
    
    uint8_t *buff = subfrm[ephem->sat-1];
    i=0; /* word type 0 */
    type[0]    =getbitu(buff,i, 6);              i+= 6;
    time_f     =getbitu(buff,i, 2);              i+= 2+88;
    week       =getbitu(buff,i,12);              i+=12; /* gst-week */
    tow        =getbitu(buff,i,20);
    
    i=128; /* word type 1 */
    type[1]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[0] =getbitu(buff,i,10);              i+=10;
    toes       =getbitu(buff,i,14)*60.0;         i+=14;
    ephem->M0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    ephem->e   =getbitu(buff,i,32)*P2_33;        i+=32;
    sqrtA      =getbitu(buff,i,32)*P2_19;
    
    i=128*2; /* word type 2 */
    type[2]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[1] =getbitu(buff,i,10);              i+=10;
    ephem->OMG0=getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    ephem->i0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    ephem->omg =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    ephem->i_dot=getbits(buff,i,14)*P2_43*SC2RAD;
    
    i=128*3; /* word type 3 */
    type[3]         =getbitu(buff,i, 6);              i+= 6;
    iod_nav[2]      =getbitu(buff,i,10);              i+=10;
    ephem->OMG_dot  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
    ephem->delta_n  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
    ephem->cuc      =getbits(buff,i,16)*P2_29;        i+=16;
    ephem->cus      =getbits(buff,i,16)*P2_29;        i+=16;
    ephem->crc      =getbits(buff,i,16)*P2_5;         i+=16;
    ephem->crs      =getbits(buff,i,16)*P2_5;         i+=16;
    uint32_t sisa   =getbitu(buff,i, 8);
    if (sisa < 50)       ephem->ura = 0.01*sisa;
    else if (sisa < 75)  ephem->ura = 0.5+0.02*(sisa-50);
    else if (sisa < 100) ephem->ura = 1.0+0.04*(sisa-75);
    else if (sisa < 125) ephem->ura = 2.0+0.16*(sisa-100);
    else                 ephem->ura = -1.0;
    
    i=128*4; /* word type 4 */
    type[4]         =getbitu(buff,i, 6);              i+= 6;
    iod_nav[3]      =getbitu(buff,i,10);              i+=10;
    svid            =getbitu(buff,i, 6);              i+= 6;
    ephem->cic      =getbits(buff,i,16)*P2_29;        i+=16;
    ephem->cis      =getbits(buff,i,16)*P2_29;        i+=16;
    toc             =getbitu(buff,i,14)*60.0;         i+=14;
    ephem->af0      =getbits(buff,i,31)*P2_34;        i+=31;
    ephem->af1      =getbits(buff,i,21)*P2_46;        i+=21;
    ephem->af2      =getbits(buff,i, 6)*P2_59;
    
    i=128*5; /* word type 5 */
    type[5]         =getbitu(buff,i, 6);              i+= 6+41;
    ephem->tgd[0]   =getbits(buff,i,10)*P2_32;        i+=10; /* BGD E5a/E1 */
    ephem->tgd[1]   =getbits(buff,i,10)*P2_32;        i+=10; /* BGD E5b/E1 */
    e5b_hs          =getbitu(buff,i, 2);              i+= 2;
    e1b_hs          =getbitu(buff,i, 2);              i+= 2;
    e5b_dvs         =getbitu(buff,i, 1);              i+= 1;
    e1b_dvs         =getbitu(buff,i, 1);
    
    /* test word types */
    if (type[0]!=0||type[1]!=1||type[2]!=2||type[3]!=3||type[4]!=4||type[5]!=5) 
    {
        LOG(ERROR) << "decode gal inav error: type=" << type[0] << ' ' << type[1]
                   << ' ' << type[2] << ' ' << ' ' << type[3] << ' '  << type[4]
                   << ' ' << type[5];
        return -1;
    }
    /* test word type 0 time field */
    if (time_f!=2) 
    {
        LOG(ERROR) << "decode_gal_inav error: word0-time=" << time_f;
        return -1;
    }
    /* test consistency of iod_nav */
    if (iod_nav[0]!=iod_nav[1]||iod_nav[0]!=iod_nav[2]||iod_nav[0]!=iod_nav[3]) {
        LOG(ERROR) << "decode_gal_inav error: ionav=" << iod_nav[0] << ' '
                   << iod_nav[1] << ' ' << iod_nav[2] << ' ' << iod_nav[3];
              
        return 0;
    }
    if (ephem->sat != sat_no(SYS_GAL,svid)) 
    {
        LOG(ERROR) << "decode_gal_inav svid error: svid=" << svid;
        return 0;
    }
    ephem->A=sqrtA*sqrtA;
    ephem->iode=ephem->iodc=iod_nav[0];
    ephem->health=(e5b_hs<<7)|(e5b_dvs<<6)|(e1b_hs<<1)|e1b_dvs;
    ephem->ttr=gst2time(week,tow);
    tt=time_diff(gst2time(week,toes),ephem->ttr); /* week complient to toe */
    if      (tt> 302400.0) week--;
    else if (tt<-302400.0) week++;
    ephem->toe=gst2time(week,toes);
    ephem->toc=gst2time(week,toc);
    ephem->toe_tow = time2gpst(ephem->toe, &ephem->week);
    ephem->code =(1<<0)|(1<<9); /* data source = i/nav e1b, af0-2,toc,sisa for e5b-e1 */
    curr_time = ephem->ttr;      // update current time
    memset(subfrm[ephem->sat-1], 0, 380);
    return 0;
}

int UbloxMessageProcessor::decode_GPS_subframe(const uint8_t *msg_data, const uint32_t msg_len, EphemPtr ephem)
{
    uint32_t words[10];
    if (msg_len < 56)     // header(6) + payload_1(8) + payload_2(40) + checksum(2)
    {
        LOG(ERROR) << "ubx rxmsfrbx length error: sat=" << ephem->sat << " len=" << msg_len;
        return -1;
    }
    const uint8_t *p = msg_data + 14;
    if ((*reinterpret_cast<const uint32_t*>(p))>>24 == PREAMB_CNAV)
    {
        // LOG(INFO) << "ubx rxmsfrbx CNAV not supported sat=" << ephem->sat;     // suppress info
        return -1;
    }
    for (size_t i = 0; i < 10; i++, p+=4)
        words[i] = (*reinterpret_cast<const uint32_t*>(p)) >> 6;  // 24 bits without parity
    

    uint32_t subframe_id = (words[1]>>2) & 0x07;
    if (subframe_id < 1 || subframe_id > 5)
    {
        LOG(ERROR) << "ubx rxmsfrbx subframe id error: sat=" << ephem->sat << " id=" << subframe_id; 
        return -1;
    }
    // LOG(INFO) << "sat is " << ephem->sat << ", GPS subframe id is " << subframe_id;
    uint32_t subframe_pos = (subframe_id-1) * 30;  // 240 bits for each subframe
    
    // now set bits to subframe buffer
    for (size_t i = 0; i < 10; i++)
        setbitu(subfrm[ephem->sat]+subframe_pos, 24*i, 24, words[i]);
    
    if (subframe_id == 3)
        decode_GPS_ephem(ephem);
    return 0;
}

int UbloxMessageProcessor::decode_GPS_ephem(EphemPtr ephem)
{
    uint8_t *frame_buf = subfrm[ephem->sat];
    // check if subframe1,2,3 are received or not
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (getbitu(frame_buf+30*i, 43, 3) != i+1)
            return static_cast<int>(i+1);
        if (i != 0 && (getbitu(frame_buf+30*i-30, 24, 17)+1) != getbitu(frame_buf+30*i, 24, 17))
            return -2;      // z count discontinue
    }
    // LOG(FATAL) << "decoding  GPS ephem";
    // decode subframe 1
    double tow          = getbitu(frame_buf, 24, 17) * 6.0;
    uint32_t off = 48;
    uint32_t week       = getbitu(frame_buf, off, 10);          off += 10;
    
                                                                off += 2; // skip signal code
    uint8_t urai        = getbitu(frame_buf, off, 4);           off += 4;
    ephem->ura          = (urai < 6) ? pow(2, urai/2.0+1) : (1<<(urai-2));
    ephem->health       = getbitu(frame_buf, off, 6);           off += 6;
    uint32_t iodc0      = getbitu(frame_buf, off, 2);           off += 2;
                                                                off += 1+87;
    int tgd             = getbits(frame_buf, off, 8);           off += 8;
    uint32_t iodc1      = getbitu(frame_buf, off, 8);           off += 8;
    double toc          = getbitu(frame_buf, off, 16) * 16.0;   off += 16;
    ephem->af2          = getbits(frame_buf, off, 8)  * P2_55;  off += 8;
    ephem->af1          = getbits(frame_buf, off, 16) * P2_43;  off += 16;
    ephem->af0          = getbits(frame_buf, off, 22) * P2_31;

    ephem->tgd[0]       = (tgd == -128 ? 0.0 : tgd*P2_31);
    ephem->iodc         = (iodc0<<8) + iodc1;
    // ephem->week      = adjgpsweek(week);
    ephem->week         = week + 1024 * GPS_WEEK_ROLLOVER_N;     // adjust GPS week
    ephem->ttr          = gpst2time(ephem->week, tow);
    ephem->toc          = gpst2time(ephem->week, toc);

    // decode subframe 2
    frame_buf += 30;
    off = 48;
    ephem->iode         = getbitu(frame_buf, off, 8);                   off += 8;
    ephem->crs          = getbits(frame_buf, off, 16) * P2_5;           off += 16;
    ephem->delta_n      = getbits(frame_buf, off, 16) * P2_43*SC2RAD;   off += 16;
    ephem->M0           = getbits(frame_buf, off, 32) * P2_31*SC2RAD;   off += 32;
    ephem->cuc          = getbits(frame_buf, off, 16) * P2_29;          off += 16;
    ephem->e            = getbitu(frame_buf, off, 32) * P2_33;          off += 32;
    ephem->cus          = getbits(frame_buf, off, 16) * P2_29;          off += 16;
    double sqrtA        = getbitu(frame_buf, off, 32) * P2_19;          off += 32;
    ephem->toe_tow      = getbitu(frame_buf, off, 16) * 16.0;           off += 16;
    ephem->A            = sqrtA * sqrtA;

    // decode subframe 3
    frame_buf += 30;
    off = 48;
    ephem->cic          = getbits(frame_buf, off, 16) * P2_29;          off += 16;
    ephem->OMG0         = getbits(frame_buf, off, 32) * P2_31*SC2RAD;   off += 32;
    ephem->cis          = getbits(frame_buf, off, 16) * P2_29;          off += 16;
    ephem->i0           = getbits(frame_buf, off, 32) * P2_31*SC2RAD;   off += 32;
    ephem->crc          = getbits(frame_buf, off, 16) * P2_5;           off += 16;
    ephem->omg          = getbits(frame_buf, off, 32) * P2_31*SC2RAD;   off += 32;
    ephem->OMG_dot      = getbits(frame_buf, off, 24) * P2_43*SC2RAD;   off += 24;
    uint32_t iode       = getbitu(frame_buf, off, 8);                   off += 8;
    ephem->i_dot        = getbits(frame_buf, off, 14) * P2_43*SC2RAD;

    /* check iode and iodc consistency */
    if (iode != ephem->iode || iode != (ephem->iodc & 0xFF)) return -1;

    /* adjustment for week handover */
    tow = time2gpst(ephem->ttr, &ephem->week);
    toc = time2gpst(ephem->toc, NULL);
    if      (ephem->toe_tow < tow-302400.0) {ephem->week++; tow -= 604800.0;}
    else if (ephem->toe_tow > tow+302400.0) {ephem->week--; tow += 604800.0;}
    ephem->toe = gpst2time(ephem->week, ephem->toe_tow);
    ephem->toc = gpst2time(ephem->week, toc);
    ephem->ttr = gpst2time(ephem->week, tow);
    curr_time = ephem->ttr;      // update current time
    return 0;
}

int UbloxMessageProcessor::build_config_msg(const std::vector<RcvConfigRecord> &rcv_configs, 
    uint8_t *buff, uint32_t &msg_len)
{
    const std::map<std::string, std::pair<uint32_t, std::string>> rcv_config_item2code = 
    {
        {"CFG-MSGOUT-UBX_NAV_HPPOSECEF" , {0x2091002f, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_HPPOSLLH"  , {0x20910034, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_POSECEF"   , {0x20910025, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_POSLLH"    , {0x2091002A, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_PVT"       , {0x20910007, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_SAT"       , {0x20910016, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_STATUS"    , {0x2091001B, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_VELECEF"   , {0x2091003E, "U1"}},
        {"CFG-MSGOUT-UBX_NAV_VELNED"    , {0x20910043, "U1"}},
        {"CFG-MSGOUT-UBX_RXM_RAWX"      , {0x209102A5, "U1"}},
        {"CFG-MSGOUT-UBX_RXM_SFRBX"     , {0x20910232, "U1"}},
        {"CFG-NAVSPG-DYNMODEL"          , {0x20110021, "E1"}},
        {"CFG-RATE-MEAS"                , {0x30210001, "U2"}},
        {"CFG-RATE-NAV"                 , {0x30210002, "U2"}},
        {"CFG-RATE-TIMEREF"             , {0x20210003, "E1"}},
        {"CFG-SIGNAL-GPS_ENA"           , {0x1031001F, "L" }},
        {"CFG-SIGNAL-GPS_L1CA_ENA"      , {0x10310001, "L" }},
        {"CFG-SIGNAL-GPS_L2CA_ENA"      , {0x10310003, "L" }},
        {"CFG-SIGNAL-GAL_ENA"           , {0x10310021, "L" }},
        {"CFG-SIGNAL-GAL_E1_ENA"        , {0x10310007, "L" }},
        {"CFG-SIGNAL-GAL_E5B_ENA"       , {0x1031000A, "L" }},
        {"CFG-SIGNAL-BDS_ENA"           , {0x10310022, "L" }},
        {"CFG-SIGNAL-BDS_B1_ENA"        , {0x1031000D, "L" }},
        {"CFG-SIGNAL-BDS_B2_ENA"        , {0x1031000E, "L" }},
        {"CFG-SIGNAL-QZSS_ENA"          , {0x10310024, "L" }},
        {"CFG-SIGNAL-QZSS_L1CA_ENA"     , {0x10310012, "L" }},
        {"CFG-SIGNAL-QZSS_L2C_ENA"      , {0x10310015, "L" }},
        {"CFG-SIGNAL-GLO_ENA"           , {0x10310025, "L" }},
        {"CFG-SIGNAL-GLO_L1_ENA"        , {0x10310018, "L" }},
        {"CFG-SIGNAL-GLO_L2_ENA"        , {0x1031001A, "L" }},
        {"CFG-UART1-BAUDRATE"           , {0x40520001, "U4"}},
        {"CFG-UART1-STOPBITS"           , {0x20520002, "E1"}},
        {"CFG-UART1-DATABITS"           , {0x20520003, "E1"}},
        {"CFG-UART1-PARITY"             , {0x20520004, "E1"}},
        {"CFG-UART1-ENABLED"            , {0x10520005, "L" }},
        {"CFG-UART1INPROT-UBX"          , {0x10730001, "L" }},
        {"CFG-UART1INPROT-NMEA"         , {0x10730002, "L" }},
        {"CFG-UART1INPROT-RTCM3X"       , {0x10730004, "L" }},
        {"CFG-UART1OUTPROT-UBX"         , {0x10740001, "L" }},
        {"CFG-UART1OUTPROT-NMEA"        , {0x10740002, "L" }},
        {"CFG-UART1OUTPROT-RTCM3X"      , {0x10740004, "L" }},
        {"CFG-UART2-BAUDRATE"           , {0x40530001, "U4"}},
        {"CFG-UART2-STOPBITS"           , {0x20530002, "E1"}},
        {"CFG-UART2-DATABITS"           , {0x20530003, "E1"}},
        {"CFG-UART2-PARITY"             , {0x20530004, "E1"}},
        {"CFG-UART2-ENABLED"            , {0x10530005, "L" }},
        {"CFG-UART2INPROT-UBX"          , {0x10750001, "L" }},
        {"CFG-UART2INPROT-NMEA"         , {0x10750002, "L" }},
        {"CFG-UART2INPROT-RTCM3X"       , {0x10750004, "L" }},
        {"CFG-UART2OUTPROT-UBX"         , {0x10760001, "L" }},
        {"CFG-UART2OUTPROT-NMEA"        , {0x10760002, "L" }},
        {"CFG-UART2OUTPROT-RTCM3X"      , {0x10760004, "L" }},
        {"CFG-USB-ENABLED"              , {0x10650001, "L" }},
        {"CFG-USBINPROT-UBX"            , {0x10770001, "L" }},
        {"CFG-USBINPROT-NMEA"           , {0x10770002, "L" }},
        {"CFG-USBINPROT-RTCM3X"         , {0x10770004, "L" }},
        {"CFG-USBOUTPROT-UBX"           , {0x10780001, "L" }},
        {"CFG-USBOUTPROT-NMEA"          , {0x10780002, "L" }},
        {"CFG-USBOUTPROT-RTCM3X"        , {0x10780004, "L" }},
        {"CFG-TP-PULSE_DEF"             , {0x20050023, "E1" }},
    };
    const std::map<std::string, uint8_t> rcv_config_type2code = 
    {
        {"U1", 1 },
        {"U2", 2 },
        {"U4", 3 },
        {"U8", 4 },
        {"E1", 5 },
        {"E2", 6 },
        {"E4", 7 },
        {"X1", 8 },
        {"X2", 9 },
        {"X4", 10},
        {"X8", 11},
        {"L" , 12},
        {"I1", 13},
        {"I2", 14},
        {"I4", 15},
        {"I8", 16},
        {"R4", 17},
        {"R8", 18},

    };
    buff[0] = UBX_SYNC_1;
    buff[1] = UBX_SYNC_2;
    buff[2] = 0x06;
    buff[3] = 0x8A;     // UBX-CFG-VALSET
    buff[6] = 0;        // Message version, set to 0 for transactionless, 1 for transaction
    buff[7] = 1;        // apply config to RAM(0) layer. NOTE: RAM(1<<0), BBR(1<<1), Flash(1<<2)
    buff[8] = 0;
    buff[9] = 0;
    uint8_t *p = buff + 10;
    uint16_t payload_len = 4;
    for(const RcvConfigRecord &record : rcv_configs)        // for each config record
    {
        const std::string &record_key = record.first;
        const std::string &record_value_str = record.second;
        if (record_key.empty())
        {
            LOG(ERROR) << "Empty record key.";
            return -1;
        }
        uint32_t code = 0;
        std::string data_type;
        if (rcv_config_item2code.count(record_key) != 0)
        {
            code = rcv_config_item2code.at(record_key).first;
            data_type = rcv_config_item2code.at(record_key).second;
        }
        else if (record_key.rfind("CFG-MSGOUT-UBX_NAV_", 0) != std::string::npos)
        {
            size_t delimiter_pos = record_key.find_last_of('_');
            std::string msgout_type = record_key.substr(0, delimiter_pos);
            if (rcv_config_item2code.count(msgout_type) == 0)
            {
                LOG(ERROR) << "Not supported command type: " << record_key;
                return -1;
            }
            if (record_key.substr(delimiter_pos+1) == "UART1")
                code = rcv_config_item2code.at(msgout_type).first;
            else if (record_key.substr(delimiter_pos+1) == "UART2")
                code = rcv_config_item2code.at(msgout_type).first + 1;
            else if (record_key.substr(delimiter_pos+1) == "USB")
                code = rcv_config_item2code.at(msgout_type).first + 2;
            data_type = rcv_config_item2code.at(msgout_type).second;
        }
        else 
        {
            LOG(ERROR) << "Not supported command type: " << record_key;
            return -1;
        }

        uint8_t value_size = static_cast<uint8_t>((code>>28)&0x07);
        uint8_t value_num_bytes = (value_size>2 ? (1<<(value_size-2)) : 1);
        uint8_t *value_bytes;
        int64_t value_u;
        int64_t value_i;
        float value_f;
        double value_d;
        switch(rcv_config_type2code.at(data_type))
        {
            case 1 :
            case 2 :
            case 3 :
            case 4 :
            case 5 :
            case 6 :
            case 7 :
            case 8 :
            case 9 :
            case 10:
            case 11:
            case 12:
                value_u = stoull(record_value_str);
                value_bytes = reinterpret_cast<uint8_t*>(&value_u);
                break;
            case 13:
            case 14:
            case 15:
            case 16:
                value_i = stoll(record_value_str);
                value_bytes = reinterpret_cast<uint8_t*>(&value_i);
                break;
            case 17:
                value_f = stof(record_value_str);
                value_bytes = reinterpret_cast<uint8_t*>(&value_f);
                break;
            case 18:
                value_d = stod(record_value_str);
                value_bytes = reinterpret_cast<uint8_t*>(&value_d);
                break;
            default:
                LOG(INFO) << "Unrecognized date type: " << data_type;
                break;
        }
        memcpy(p, reinterpret_cast<uint8_t*>(&code), 4);     // write config bytes
        p += 4;
        payload_len += 4;
        memcpy(p, value_bytes, value_num_bytes);
        p += value_num_bytes;
        payload_len += value_num_bytes;
    }

    memcpy(buff+4, reinterpret_cast<uint8_t*>(&payload_len), 2);
    msg_len = 6 + payload_len + 2;
    set_checksum(buff, msg_len);
    return 0;
}

double UbloxMessageProcessor::sig_freq(const int sys, const int sid, const int fcn)
{
    static const double freq_glo[8]={FREQ1_GLO,FREQ2_GLO,FREQ3_GLO};
    static const double dfrq_glo[8]={DFRQ1_GLO,DFRQ2_GLO};
    static const double freq_bds[8]={FREQ1_BDS,FREQ2_BDS,FREQ3_BDS,FREQ2_BDS};
    
    if (sys == SYS_GLO) {
        return freq_glo[sid-1]+dfrq_glo[sid-1]*fcn;
    }
    else if (sys == SYS_BDS) {
        return freq_bds[sid-1];
    }
    return LIGHT_SPEED/lam_carr[sid-1];
}

int UbloxMessageProcessor::sig_idx(const int sys, const int code)
{
    if (sys == SYS_GPS) {
        if (code==CODE_L1C) return 1;
        if (code==CODE_L2L) return 2;
        if (code==CODE_L2M) return N_FREQ+1;
    }
    else if (sys == SYS_GLO) {
        if (code==CODE_L1C) return 1;
        if (code==CODE_L2C) return 2;
    }
    else if (sys == SYS_GAL) {
        if (code==CODE_L1C) return 1;
        if (code==CODE_L1B) return N_FREQ+1;
        if (code==CODE_L7I) return 2; /* E5bI */
        if (code==CODE_L7Q) return 2; /* E5bQ */
    }
    else if (sys == SYS_QZS) {
        if (code==CODE_L1C) return 1;
        if (code==CODE_L2L) return 2;
    }
    else if (sys == SYS_BDS) {
        if (code==CODE_L1I) return 1;
        if (code==CODE_L7I) return 2;
    }
    else if (sys == SYS_SBS) {
        if (code==CODE_L1C) return 1;
    }
    return 0;
}

int UbloxMessageProcessor::ubx_sig(const int sys, const int sigid)
{
    if (sys == SYS_GPS) {
        if (sigid == 0) return CODE_L1C; /* L1C/A */
        if (sigid == 3) return CODE_L2L; /* L2CL */
        if (sigid == 4) return CODE_L2M; /* L2CM */
    }
    else if (sys == SYS_GLO) {
        if (sigid == 0) return CODE_L1C; /* G1C/A (GLO L1 OF) */
        if (sigid == 2) return CODE_L2C; /* G2C/A (GLO L2 OF) */
    }
    else if (sys == SYS_GAL) {
        if (sigid == 0) return CODE_L1C; /* E1C */
        if (sigid == 1) return CODE_L1B; /* E1B */
        if (sigid == 5) return CODE_L7I; /* E5bI */
        if (sigid == 6) return CODE_L7Q; /* E5bQ */
    }
    else if (sys == SYS_QZS) {
        if (sigid == 0) return CODE_L1C; /* L1C/A */
        if (sigid == 5) return CODE_L2L; /* L2CL (not specified in [5]) */
    }
    else if (sys == SYS_BDS) {
        if (sigid == 0) return CODE_L1I; /* B1I D1 (rinex 3.02) */
        if (sigid == 1) return CODE_L1I; /* B1I D2 (rinex 3.02) */
        if (sigid == 2) return CODE_L7I; /* B2I D1 */
        if (sigid == 3) return CODE_L7I; /* B2I D2 */
    }
    else if (sys == SYS_SBS) {
        return CODE_L1C; /* L1C/A (not in [5]) */
    }
    return CODE_NONE;
}

int UbloxMessageProcessor::ubx_sys(const int gnssid)
{
    switch (gnssid) {
        case 0: return SYS_GPS;
        case 2: return SYS_GAL;
        case 3: return SYS_BDS;
        case 6: return SYS_GLO;
    }
    return 0;
}

int UbloxMessageProcessor::test_glostr(const uint8_t *data)
{
    static const uint8_t xor_8bit[256]={ /* xor of 8 bits */
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0
    };
    static const uint8_t mask_hamming[][12]={ /* mask of hamming codes */
        {0x55,0x55,0x5A,0xAA,0xAA,0xAA,0xB5,0x55,0x6A,0xD8,0x08},
        {0x66,0x66,0x6C,0xCC,0xCC,0xCC,0xD9,0x99,0xB3,0x68,0x10},
        {0x87,0x87,0x8F,0x0F,0x0F,0x0F,0x1E,0x1E,0x3C,0x70,0x20},
        {0x07,0xF8,0x0F,0xF0,0x0F,0xF0,0x1F,0xE0,0x3F,0x80,0x40},
        {0xF8,0x00,0x0F,0xFF,0xF0,0x00,0x1F,0xFF,0xC0,0x00,0x80},
        {0x00,0x00,0x0F,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x01,0x00},
        {0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00},
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8}
    };
    uint8_t cs = 0;
    int n = 0;
    
    for (uint32_t i = 0; i < 8; ++i) 
    {
        for (uint32_t j = 0, cs = 0; j < 11; ++j)
            cs ^= xor_8bit[data[j]&mask_hamming[i][j]];
        if (cs) n++;
    }
    return n==0 || (n==2 && cs);
}

uint32_t UbloxMessageProcessor::crc24q(const uint8_t *data, const uint32_t size) const 
{
    uint32_t crc = 0;
    for (uint32_t i = 0; i < size; ++i) 
        crc = ( (crc<<8) & 0xFFFFFF) ^ tbl_CRC24Q[ (crc>>16) ^ data[i]];
    return crc;
}

bool UbloxMessageProcessor::check_checksum(const uint8_t *data, const uint32_t size) 
{
    uint8_t cka = 0, ckb = 0;
    for (uint32_t i = 2;i < size-2; ++i) 
    {
        cka += data[i]; 
        ckb += cka;
    }
    return cka == data[size-2] && ckb == data[size-1];
}

void UbloxMessageProcessor::set_checksum(uint8_t *data, uint32_t size)
{
    uint8_t cka = 0, ckb = 0;
    for (uint32_t i = 2; i < size-2; ++i) 
    {
        cka += data[i]; 
        ckb += cka;
    }
    data[size-2] = cka;
    data[size-1] = ckb;
}

void UbloxMessageProcessor::setbitu(uint8_t *buff, const uint32_t pos, const uint32_t len, uint32_t data) const
{
    if (len == 0 || len > 32)   return;
    uint32_t mask = 1u << (len-1);
    for (uint32_t i = pos; i < pos+len; ++i, mask>>=1)
    {
        if (data & mask)
            buff[i/8] |= (1u<<(7-i%8));
        else
            buff[i/8] &= ~(1u<<(7-i%8));
    }
}

uint32_t UbloxMessageProcessor::getbitu(const uint8_t *buff, const uint32_t pos, const uint32_t len) const
{
    uint32_t bits = 0;
    for (size_t i = pos; i < pos+len; ++i)
        bits = (bits<<1) + ((buff[i/8]>>(7-i%8))&1u);
    return bits;
}

int UbloxMessageProcessor::getbits(const uint8_t *buff, const uint32_t pos, const uint32_t len) const
{
    uint32_t bits = getbitu(buff, pos, len);
    if (len == 0 || len >= 32 || !(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len));  // extend sign
}

uint32_t UbloxMessageProcessor::getbitu2(const uint8_t *buff, const uint32_t p1, const uint32_t l1, 
                                const uint32_t p2, const uint32_t l2) const
{
    return (getbitu(buff,p1,l1)<<l2)+getbitu(buff,p2,l2);
}

int UbloxMessageProcessor::getbits2(const uint8_t *buff, const uint32_t p1, const uint32_t l1, 
                           const uint32_t p2, const uint32_t l2) const
{
    if (getbitu(buff,p1,1))
        return (int)((getbits(buff,p1,l1)<<l2)+getbitu(buff,p2,l2));
    else
        return (int)getbitu2(buff,p1,l1,p2,l2);
}

uint32_t UbloxMessageProcessor::getbitu3(const uint8_t *buff, const uint32_t p1, const uint32_t l1, 
                          const uint32_t p2, const uint32_t l2, const uint32_t p3, const uint32_t l3) const
{
    return (getbitu(buff,p1,l1)<<(l2+l3))+(getbitu(buff,p2,l2)<<l3)+ getbitu(buff,p3,l3);
}

int UbloxMessageProcessor::getbits3(const uint8_t *buff, const uint32_t p1, const uint32_t l1, const uint32_t p2, 
                     const uint32_t l2, const uint32_t p3, const uint32_t l3) const
{
    if (getbitu(buff,p1,1))
        return (int)((getbits(buff,p1,l1)<<(l2+l3))+
                   (getbitu(buff,p2,l2)<<l3)+getbitu(buff,p3,l3));
    else
        return (int)getbitu3(buff,p1,l1,p2,l2,p3,l3);
}

double UbloxMessageProcessor::getbitg(const uint8_t *buff, const int pos, const int len) const
{
    double value=getbitu(buff,pos+1,len-1);
    return getbitu(buff,pos,1)?-value:value;
}

uint32_t UbloxMessageProcessor::merge_two_u(const uint32_t a, const uint32_t b, const uint32_t n) const
{
    return (a<<n)+b;
}

int UbloxMessageProcessor::merge_two_s(const int a, const uint8_t b, const int n) const
{
    return (int)((a<<n)+b);
}
