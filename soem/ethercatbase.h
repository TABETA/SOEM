/*
 * Simple Open EtherCAT Master Library
 *
 * File    : ethercatbase.h
 * Version : 1.3.1
 * Date    : 11-03-2015
 * Copyright (C) 2005-2015 Speciaal Machinefabriek Ketels v.o.f.
 * Copyright (C) 2005-2015 Arthur Ketels
 * Copyright (C) 2008-2009 TU/e Technische Universiteit Eindhoven
 * Copyright (C) 2014-2015 rt-labs AB , Sweden
 *
 * SOEM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the Free
 * Software Foundation.
 *
 * SOEM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * As a special exception, if other files instantiate templates or use macros
 * or inline functions from this file, or you compile this file and link it
 * with other works to produce a work based on this file, this file does not
 * by itself cause the resulting work to be covered by the GNU General Public
 * License. However the source code for this file must still be made available
 * in accordance with section (3) of the GNU General Public License.
 *
 * This exception does not invalidate any other reasons why a work based on
 * this file might be covered by the GNU General Public License.
 *
 * The EtherCAT Technology, the trade name and logo “EtherCAT” are the intellectual
 * property of, and protected by Beckhoff Automation GmbH. You can use SOEM for
 * the sole purpose of creating, using and/or selling or otherwise distributing
 * an EtherCAT network master provided that an EtherCAT Master License is obtained
 * from Beckhoff Automation GmbH.
 *
 * In case you did not receive a copy of the EtherCAT Master License along with
 * SOEM write to Beckhoff Automation GmbH, Eiserstraße 5, D-33415 Verl, Germany
 * (www.beckhoff.com).
 */

/** \file
 * \brief
 * Headerfile for ethercatbase.c
 */

#ifndef _ethercatbase_
#define _ethercatbase_

#define HAVE_REMOTE

#include <pcap.h>
#include <Packet32.h>

/** pointer structure to Tx and Rx stacks */
struct ec_stackT
{
	/** socket connection used */
	pcap_t      **sock;
	/** tx buffer */
	ec_bufT(*txbuf)[EC_MAXBUF];
	/** tx buffer lengths */
	int(*txbuflength)[EC_MAXBUF];
	/** temporary receive buffer */
	ec_bufT     *tempbuf;
	/** rx buffers */
	ec_bufT(*rxbuf)[EC_MAXBUF];
	/** rx buffer status fields */
	int(*rxbufstat)[EC_MAXBUF];
	/** received MAC source address (middle word) */
	int(*rxsa)[EC_MAXBUF];
};

/** pointer structure to buffers for redundant port */
struct ecx_redportt
{
	static ecx_redportt* getInstance()
	{
		static ecx_redportt ins;
		return &ins;
	}
	ec_stackT   stack;
	pcap_t      *sockhandle;
	/** rx buffers */
	ec_bufT rxbuf[EC_MAXBUF];
	/** rx buffer status */
	int rxbufstat[EC_MAXBUF];
	/** rx MAC source address */
	int rxsa[EC_MAXBUF];
	/** temporary rx buffer */
	ec_bufT tempinbuf;
} ;

struct ecx_contextt;
struct ec_alstatust;
/** pointer structure to buffers, vars and mutexes for port instantiation */
struct ecx_portt
{
private:


	int recvpkt(int stacknumber);
	int waitinframe_red(int idx, osal_timert *timer);
	int setupnic(const char *ifname, pcap_t **psock);
public:
	static ecx_portt* getInstance()
	{
		static ecx_portt ins;
		return &ins;
	}
	int setupdatagram(uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data);
	int setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data);
	int adddatagram(uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data);
	int adddatagram(void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data);
	int BWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	int BRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	int APRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	int ARMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	int FRMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	uint16 APRDw(uint16 ADP, uint16 ADO, int timeout);
	int FPRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	uint16 FPRDw(uint16 ADP, uint16 ADO, int timeout);
	int APWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout);
	int APWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	int FPWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout);
	int FPWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	int LRW(uint32 LogAdr, uint16 length, void *data, int timeout);
	int LRD(uint32 LogAdr, uint16 length, void *data, int timeout);
	int LWR(uint32 LogAdr, uint16 length, void *data, int timeout);
	int LRWDC(uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout);
	int init_redundant(ecx_redportt *redport, const char *ifname, char *if2name);
	int setupnicSecondary(const char *ifname);
	int setupnicPrimary(const char *ifname);
	int closenic();
	int getindex();
	void setbufstat(int idx, int bufstat);
	int outframe(int idx, int stacknumber);
	int outframe_red(int idx);
	int inframe(int idx, int stacknumber);
	int waitinframe(int idx, int timeout);
	int srconfirm(int idx, int timeout);
	int FPRD_multi(int n, uint16 *configlst, ec_alstatust *slstatlst, int timeout);
	void receive_processdata(int idx, uint16 DCtO, uint16 DCl, uint16 stacklen, int wkc2, void* stackdata, int64* DCtime, int* wkc, int *valid_wkc, boolean* first);

private:
	ec_stackT   stack;
	pcap_t      *sockhandle;
	/** rx buffers */
	ec_bufT rxbuf[EC_MAXBUF];
	/** rx buffer status */
	int rxbufstat[EC_MAXBUF];
	/** rx MAC source address */
	int rxsa[EC_MAXBUF];
	/** temporary rx buffer */
	ec_bufT tempinbuf;
	/** temporary rx buffer status */
	int tempinbufs;
	/** transmit buffers */
	ec_bufT txbuf[EC_MAXBUF];
	/** transmit buffer lenghts */
	int txbuflength[EC_MAXBUF];
	/** temporary tx buffer */
	ec_bufT txbuf2;
	/** temporary tx buffer length */
	int txbuflength2;
	/** last used frame index */
	int lastidx;
	/** current redundancy state */
	int redstate;
	/** pointer to redundancy port and buffers */
	ecx_redportt *redport;
	CRITICAL_SECTION getindex_mutex;
	CRITICAL_SECTION tx_mutex;
	CRITICAL_SECTION rx_mutex;
};

extern const uint16 priMAC[3];
extern const uint16 secMAC[3];

#ifdef EC_VER1
class ec
{
public:
	static int setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data);
	static int adddatagram(void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data);
	static int BWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int BRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int APRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int ARMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int FRMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static uint16 APRDw(uint16 ADP, uint16 ADO, int timeout);
	static int FPRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static uint16 FPRDw(uint16 ADP, uint16 ADO, int timeout);
	static int APWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout);
	static int APWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int FPWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout);
	static int FPWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int LRW(uint32 LogAdr, uint16 length, void *data, int timeout);
	static int LRD(uint32 LogAdr, uint16 length, void *data, int timeout);
	static int LWR(uint32 LogAdr, uint16 length, void *data, int timeout);
	static int LRWDC(uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout);
	static int setupnic(const char *ifname, int secondary);
	static int closenic(void);
	static int getindex(void);
	static void setbufstat(int idx, int bufstat);
	static int outframe(int idx, int stacknumber);
	static int outframe_red(int idx);
	static int inframe(int idx, int stacknumber);
	static int waitinframe(int idx, int timeout);
	static int srconfirm(int idx, int timeout);
	static void setupheader(void *p);
};
#endif


#ifdef WIN32


#include <sys/types.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

#include <winsock2.h>
#include "ethercattype.h"
#include <Mmsystem.h>
#include "nicdrv.h"
#include "osal_win32.h"

#endif









#endif
