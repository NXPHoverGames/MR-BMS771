/****************************************************************************
 * nxp_bms/BMS_v1/src/CAN/socketcan.c
 *
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "socketcan.h"
#include "cli.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <netutils/netlib.h>

int16_t socketcanOpen(CanardSocketInstance *ins, const char *const can_iface_name, const bool can_fd)
{
    struct sockaddr_can addr;
    struct ifreq        ifr;
    int    netSockedFd;

    ins->can_fd = can_fd;

    strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex            = if_nametoindex(ifr.ifr_name);

    if(!ifr.ifr_ifindex)
    {
        perror("if_nametoindex");
        return -1;
    }

    /* Get a socket for the INET subsystem */
    netSockedFd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);

    /* Check if OK */
    if(netSockedFd >= 0)
    {
        /* Set the flag for ifup */
        ifr.ifr_flags |= IFF_UP; // IFF_DOWN for ifdown

        /* perform IOCTL */
        ioctl(netSockedFd, SIOCSIFFLAGS, (unsigned long)&ifr);

        /* close */
        close(netSockedFd);

        /* Unset the flag for ifup */
        ifr.ifr_flags &= ~IFF_UP;
    }

    /* open socket */
    if((ins->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int on = 1;
    /* RX Timestamping */

    if(setsockopt(ins->s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0)
    {
        perror("SO_TIMESTAMP is disabled");
        return -1;
    }

    /* NuttX Feature: Enable TX deadline when sending CAN frames
     * When a deadline occurs the driver will remove the CAN frame
     */

    if(setsockopt(ins->s, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0)
    {
        perror("CAN_RAW_TX_DEADLINE is disabled");
        return -1;
    }

    if(can_fd)
    {
        if(setsockopt(ins->s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0)
        {
            perror("no CAN FD support");
            return -1;
        }
    }

    if(bind(ins->s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return -1;
    }

    // Setup TX msg
    ins->send_iov.iov_base = &ins->send_frame;

    if(ins->can_fd)
    {
        ins->send_iov.iov_len = sizeof(struct canfd_frame);
    }
    else
    {
        ins->send_iov.iov_len = sizeof(struct can_frame);
    }

    memset(&ins->send_control, 0x00, sizeof(ins->send_control));

    ins->send_msg.msg_iov        = &ins->send_iov;
    ins->send_msg.msg_iovlen     = 1;
    ins->send_msg.msg_control    = &ins->send_control;
    ins->send_msg.msg_controllen = sizeof(ins->send_control);

    ins->send_cmsg             = CMSG_FIRSTHDR(&ins->send_msg);
    ins->send_cmsg->cmsg_level = SOL_CAN_RAW;
    ins->send_cmsg->cmsg_type  = CAN_RAW_TX_DEADLINE;
    ins->send_cmsg->cmsg_len   = sizeof(struct timeval);
    ins->send_tv               = (struct timeval *)CMSG_DATA(&ins->send_cmsg);

    // Setup RX msg
    ins->recv_iov.iov_base = &ins->recv_frame;

    if(can_fd)
    {
        ins->recv_iov.iov_len = sizeof(struct canfd_frame);
    }
    else
    {
        ins->recv_iov.iov_len = sizeof(struct can_frame);
    }

    memset(&ins->recv_control, 0x00, sizeof(ins->recv_control));

    ins->recv_msg.msg_iov        = &ins->recv_iov;
    ins->recv_msg.msg_iovlen     = 1;
    ins->recv_msg.msg_control    = &ins->recv_control;
    ins->recv_msg.msg_controllen = sizeof(ins->recv_control);
    ins->recv_cmsg               = CMSG_FIRSTHDR(&ins->recv_msg);

    return 0;
}

int16_t socketcanCyphalTransmit(CanardSocketInstance *ins, const CanardFrame *txf)
{
    /* Copy CanardFrame to can_frame/canfd_frame */

    if(ins->can_fd)
    {
        ins->send_frame.can_id = txf->extended_can_id;
        ins->send_frame.can_id |= CAN_EFF_FLAG;
        ins->send_frame.len = txf->payload_size;
        memcpy(&ins->send_frame.data, txf->payload, txf->payload_size);
    }
    else
    {
        struct can_frame *frame = (struct can_frame *)&ins->send_frame;
        frame->can_id           = txf->extended_can_id;
        frame->can_id |= CAN_EFF_FLAG;
        frame->can_dlc = txf->payload_size;
        memcpy(&frame->data, txf->payload, txf->payload_size);
    }

    /* Set CAN_RAW_TX_DEADLINE timestamp  */

    ins->send_tv->tv_usec = txf->timestamp_usec % 1000000ULL;
    ins->send_tv->tv_sec  = (txf->timestamp_usec - ins->send_tv->tv_usec) / 1000000ULL;

    // return;
    return sendmsg(ins->s, &ins->send_msg, 0);
}

int16_t socketcanCyphalReceive(CanardSocketInstance *ins, CanardFrame *rxf)
{
    int32_t result = recvmsg(ins->s, &ins->recv_msg, 0);

    if(result < 0)
    {
        return result;
    }

    /* Copy CAN frame to CanardFrame */

    if(ins->can_fd)
    {
        struct canfd_frame *recv_frame = (struct canfd_frame *)&ins->recv_frame;
        rxf->extended_can_id           = recv_frame->can_id & CAN_EFF_MASK;
        rxf->payload_size              = recv_frame->len;
        rxf->payload                   = &recv_frame->data;
    }
    else
    {
        struct can_frame *recv_frame = (struct can_frame *)&ins->recv_frame;
        rxf->extended_can_id         = recv_frame->can_id & CAN_EFF_MASK;
        rxf->payload_size            = recv_frame->can_dlc;
        rxf->payload                 = &recv_frame->data; // FIXME either copy or clearly state the pointer reference
    }

    /* Read SO_TIMESTAMP value */

    if(ins->recv_cmsg->cmsg_level == SOL_SOCKET && ins->recv_cmsg->cmsg_type == SO_TIMESTAMP)
    {
        struct timeval *tv  = (struct timeval *)CMSG_DATA(ins->recv_cmsg);
        rxf->timestamp_usec = tv->tv_sec * 1000000ULL + tv->tv_usec;
    }

    return result;
}


int16_t socketcanDroneCANTransmit(
    CanardSocketInstance *ins, const CanardCANFrame *txf, uint64_t transmission_deadline)
{
    /* Copy CanardCANFrame to can_frame/canfd_frame */

#if CANARD_ENABLE_CANFD
    if(txf->canfd)
    {
        ins->send_frame.can_id = txf->id;
        ins->send_frame.can_id |= CAN_EFF_FLAG;
        ins->send_frame.len = txf->data_len;
        memcpy(&ins->send_frame.data, txf->data, txf->data_len);
    }
    else
#endif
    {
        struct can_frame *frame = (struct can_frame *)&ins->send_frame;
        frame->can_id           = txf->id;
        frame->can_id |= CAN_EFF_FLAG;
        frame->can_dlc = txf->data_len;
        memcpy(&frame->data, txf->data, txf->data_len);
    }

    /* Set CAN_RAW_TX_DEADLINE timestamp  */

    ins->send_tv->tv_usec = transmission_deadline % 1000000ULL;
    ins->send_tv->tv_sec  = (transmission_deadline - ins->send_tv->tv_usec) / 1000000ULL;

    // return;
    return sendmsg(ins->s, &ins->send_msg, 0);
}

int32_t socketcanDroneCANReceive(CanardSocketInstance *ins, CanardCANFrame *rxf, uint64_t *timestamp)
{
    int32_t result = recvmsg(ins->s, &ins->recv_msg, MSG_DONTWAIT);

    if(result < 0)
    {
        return -errno;
    }

    /* Copy CAN frame to CanardCANFrame */

#if CANARD_ENABLE_CANFD
    if(ins->can_fd)
    {
        struct canfd_frame *recv_frame = (struct canfd_frame *)&ins->recv_frame;
        rxf->id                        = recv_frame->can_id;
        rxf->data_len                  = recv_frame->len;
        memcpy(&rxf->data, &recv_frame->data, rxf->data_len);
    }
    else
#endif
    {
        struct can_frame *recv_frame = (struct can_frame *)&ins->recv_frame;
        rxf->id                      = recv_frame->can_id;
        rxf->data_len                = recv_frame->can_dlc;
        // protect against too much mem copy (could corrupt stack)
        if(rxf->data_len > 8) {
        	return -EAGAIN;
        }
        memcpy(&rxf->data, &recv_frame->data, rxf->data_len);
    }

    /* Read SO_TIMESTAMP value */

    if(ins->recv_cmsg->cmsg_level == SOL_SOCKET && ins->recv_cmsg->cmsg_type == SO_TIMESTAMP)
    {
        struct timeval *tv = (struct timeval *)CMSG_DATA(ins->recv_cmsg);
        *timestamp         = tv->tv_sec * 1000000ULL + tv->tv_usec;
    }

    return result;
}


/* TODO implement corresponding IOCTL */

int16_t socketcanConfigureFilter(const fd_t fd, const size_t num_filters, const struct can_filter *filters)
{
    return -1;
}

/*!
 * @brief   this function is used to set the bitrate of both classical CAN as CAN FD
 *
 * @param   ins the canard socket instance
 * @param   can_iface_name the name of the device (see canOpen())
 * @param   arbit_bitrate the classical CAN bitrate in bit/s
 * @param   data_bitrate the CAN FD bitrate in bit/s
 *
 * @return  If successful, the function will return zero (OK), -1 otherwise.
 */
int16_t socketcanSetBitrate(
    CanardSocketInstance *ins, const char *const can_iface_name, int32_t arbit_bitrate, int32_t data_bitrate)
{
    int16_t ret = 0;

    struct ifreq ifr;

    // set the device name
    strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    //    // read the bitrates
    // if(ioctl(ins->s, SIOCGCANBITRATE, &ifr) < 0)
    // {
    //  // error and return
    //  cli_printfError("socketcan ERROR: couldn't read the new bitrates!\n");
    //  ret = -1;
    //  return ret;
    // }

    // cli_printf("bitrates: %d %d\narbi_samplep: %d %d", ifr.ifr_ifru.ifru_can_data.arbi_bitrate,
    // ifr.ifr_ifru.ifru_can_data.data_bitrate,
    //  ifr.ifr_ifru.ifru_can_data.arbi_samplep, ifr.ifr_ifru.ifru_can_data.data_samplep);

    // set the bitrates
    // the clasical CAN bitrate
    ifr.ifr_ifru.ifru_can_data.arbi_bitrate = arbit_bitrate / 1000;
    ifr.ifr_ifru.ifru_can_data.arbi_samplep = 80;

    // the CAN FD bitrate
    ifr.ifr_ifru.ifru_can_data.data_bitrate = data_bitrate / 1000;
    ifr.ifr_ifru.ifru_can_data.data_samplep = 80;

    // write the bitrates and check for error
    if(ioctl(ins->s, SIOCSCANBITRATE, &ifr) < 0)
    {
        // error and return
        cli_printfError("socketcan ERROR: couldn't write the new bitrates!\n");
        ret = -1;
        return ret;
    }

    // read the bitrates
    if(ioctl(ins->s, SIOCGCANBITRATE, &ifr) < 0)
    {
        // error and return
        cli_printfError("socketcan ERROR: couldn't read the new bitrates!\n");
        ret = -1;
        return ret;
    }

    // compare the bitrates
    if(ifr.ifr_ifru.ifru_can_data.arbi_bitrate * 1000 != arbit_bitrate)
    {
        // output error
        cli_printfError("socketcan ERROR: CAN bitrate write went wrong, read: %d != set: %d!\n",
            ifr.ifr_ifru.ifru_can_data.arbi_bitrate * 1000, arbit_bitrate);

        // set the error value
        ret = -1;
    }

    if(ifr.ifr_ifru.ifru_can_data.data_bitrate * 1000 != data_bitrate)
    {
        // output error
        cli_printfError("socketcan ERROR: CAN FD bitrate write went wrong, read: %d != set: %d!\n",
            ifr.ifr_ifru.ifru_can_data.data_bitrate * 1000, data_bitrate);

        // set the error value
        ret = -1;
    }

    // cli_printf("bitrate: %d canFD bitrate: %d\n", ifr.ifr_ifru.ifru_can_data.arbi_bitrate*1000,
    //  ifr.ifr_ifru.ifru_can_data.data_bitrate*1000);

    return ret;
}

/*!
 * @brief   this function is used to set the the HW CAN filter for the node ID
 *
 * @param   ins the canard socket instance
 * @param   can_iface_name the name of the device (see canOpen())
 * @param   can_id_filter the can ID to put in the HW CAN filter
 *
 * @return  If successful, the function will return zero (OK), -1 otherwise.
 */
int16_t socketcanSetHwCanFilterID(CanardSocketInstance *ins, const char *const can_iface_name,
    uint8_t can_id_filter)
{
    int16_t ret = 0;
    struct ifreq ifr;

    // set the device name
    strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    // Set the node ID and filter mask
    ifr.ifr_ifru.ifru_can_filter.fid1 = 0x8000 | (can_id_filter << 8);
    ifr.ifr_ifru.ifru_can_filter.fid2 = 0xFF00;
    ifr.ifr_ifru.ifru_can_filter.ftype = CAN_FILTER_MASK;

    // Delete any CAN HW ID filter and check for error
    if(ioctl(ins->s, SIOCDCANEXTFILTER, &ifr) < 0)
    {
        // error and return
        cli_printfError("socketcan ERROR: couldn't delete the CAN HW filter!\n");
        ret = -1;
    } else {
        // write the new CAN HW ID filter and check for error
        if(ioctl(ins->s, SIOCACANEXTFILTER, &ifr) < 0)
        {
            // error and return
            cli_printfError("socketcan ERROR: couldn't write the new CAN HW filter!\n");
            ret = -1;
        }
    }

    return ret;
}
