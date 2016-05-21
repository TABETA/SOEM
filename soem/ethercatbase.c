/*
 * Simple Open EtherCAT Master Library
 *
 * File    : ethercatbase.c
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
 * Base EtherCAT functions.
 *
 * Setting up a datagram in an ethernet frame.
 * EtherCAT datagram primitives, broadcast, auto increment, configured and
 * logical addressed data transfers. All base transfers are blocking, so
 * wait for the frame to be returned to the master or timeout. If this is
 * not acceptable build your own datagrams and use the functions from nicdrv.c.
 */

#include <stdio.h>
#include <string.h>
#include "oshw.h"
#include "osal.h"
#include "ethercattype.h"
#include "ethercatbase.h"

/** Write data to EtherCAT datagram.
 *
 * @param[out] datagramdata   = data part of datagram
 * @param[in]  com            = command
 * @param[in]  length         = length of databuffer
 * @param[in]  data           = databuffer to be copied into datagram
 */
void ecx_portt::writedatagramdata(void *datagramdata, ec_cmdtype com, uint16 length, const void * data)
{
	if (length > 0)
	{
			switch (com)
			{
				case EC_CMD_NOP:
						/* Fall-through */
				case EC_CMD_APRD:
						/* Fall-through */
				case EC_CMD_FPRD:
						/* Fall-through */
				case EC_CMD_BRD:
						/* Fall-through */
				case EC_CMD_LRD:
						/* no data to write. initialise data so frame is in a known state */
						memset(datagramdata, 0, length);
						break;
				default:
						memcpy(datagramdata, data, length);
						break;
			}
	}
}

/** Generate and set EtherCAT datagram in a standard ethernet frame.
 *
 * @param[in] port        = port context struct
 * @param[out] frame       = framebuffer
 * @param[in]  com         = command
 * @param[in]  idx         = index used for TX and RX buffers
 * @param[in]  ADP         = Address Position
 * @param[in]  ADO         = Address Offset
 * @param[in]  length      = length of datagram excluding EtherCAT header
 * @param[in]  data        = databuffer to be copied in datagram
 * @return always 0
 */
int ecx_portt::setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
	ec_comt *datagramP;
	uint8 *frameP;

	frameP = static_cast<uint8 *>(frame);
	/* Ethernet header is preset and fixed in frame buffers
			EtherCAT header needs to be added after that */
	datagramP = (ec_comt*)&frameP[ETH_HEADERSIZE];
	datagramP->elength = htoes(EC_ECATTYPE + EC_HEADERSIZE + length);
	datagramP->command = com;
	datagramP->index = idx;
	datagramP->ADP = htoes(ADP);
	datagramP->ADO = htoes(ADO);
	datagramP->dlength = htoes(length);
	writedatagramdata(&frameP[ETH_HEADERSIZE + EC_HEADERSIZE], static_cast<ec_cmdtype>(com), length, data);
	/* set WKC to zero */
	frameP[ETH_HEADERSIZE + EC_HEADERSIZE + length] = 0x00;
	frameP[ETH_HEADERSIZE + EC_HEADERSIZE + length + 1] = 0x00;
	/* set size of frame in buffer array */
	this->txbuflength[idx] = ETH_HEADERSIZE + EC_HEADERSIZE + EC_WKCSIZE + length;

	return 0;
}

/** Add EtherCAT datagram to a standard ethernet frame with existing datagram(s).
 *
 * @param[in] port        = port context struct
 * @param[out] frame      = framebuffer
 * @param[in]  com        = command
 * @param[in]  idx        = index used for TX and RX buffers
 * @param[in]  more       = TRUE if still more datagrams to follow
 * @param[in]  ADP        = Address Position
 * @param[in]  ADO        = Address Offset
 * @param[in]  length     = length of datagram excluding EtherCAT header
 * @param[in]  data       = databuffer to be copied in datagram
 * @return Offset to data in rx frame, usefull to retrieve data after RX.
 */
int ecx_portt::adddatagram(void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
	ec_comt *datagramP;
	uint8 *frameP;
	uint16 prevlength;

	frameP = static_cast<uint8 *>(frame);
	/* copy previous frame size */
	prevlength = this->txbuflength[idx];
	datagramP = (ec_comt*)&frameP[ETH_HEADERSIZE];
	/* add new datagram to ethernet frame size */
	datagramP->elength = htoes( etohs(datagramP->elength) + EC_HEADERSIZE + length );
	/* add "datagram follows" flag to previous subframe dlength */
	datagramP->dlength = htoes( etohs(datagramP->dlength) | EC_DATAGRAMFOLLOWS );
	/* set new EtherCAT header position */
	datagramP = (ec_comt*)&frameP[prevlength - EC_ELENGTHSIZE];
	datagramP->command = com;
	datagramP->index = idx;
	datagramP->ADP = htoes(ADP);
	datagramP->ADO = htoes(ADO);
	if (more)
	{
			/* this is not the last datagram to add */
			datagramP->dlength = htoes(length | EC_DATAGRAMFOLLOWS);
	}
	else
	{
			/* this is the last datagram in the frame */
			datagramP->dlength = htoes(length);
	}
	writedatagramdata(&frameP[prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE], static_cast<ec_cmdtype>(com), length, data);
	/* set WKC to zero */
	frameP[prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE + length] = 0x00;
	frameP[prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE + length + 1] = 0x00;
	/* set size of frame in buffer array */
	this->txbuflength[idx] = prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE + EC_WKCSIZE + length;

	/* return offset to data in rx frame
			14 bytes smaller than tx frame due to stripping of ethernet header */
	return prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE - ETH_HEADERSIZE;
}

/** BRW "broadcast write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, normally 0
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] length      = length of databuffer
 * @param[in] data        = databuffer to be written to slaves
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::BWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	uint8 idx;
	int wkc;

	/* get fresh index */
	idx = getindex();
	/* setup datagram */
	setupdatagram(&(this->txbuf[idx]), EC_CMD_BWR, idx, ADP, ADO, length, data);
	/* send data and wait for answer */
	wkc = srconfirm(idx, timeout);
	/* clear buffer status */
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** BRD "broadcast read" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]  ADP        = Address Position, normally 0
 * @param[in]  ADO        = Address Offset, slave memory address
 * @param[in]  length     = length of databuffer
 * @param[out] data       = databuffer to put slave data in
 * @param[in]  timeout    = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::BRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	uint8 idx;
	int wkc;

	/* get fresh index */
	idx = getindex();
	/* setup datagram */
	setupdatagram(&(this->txbuf[idx]), EC_CMD_BRD, idx, ADP, ADO, length, data);
	/* send data and wait for answer */
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
			/* copy datagram to data buffer */
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	/* clear buffer status */
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** APRD "auto increment address read" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]  ADP        = Address Position, each slave ++, slave that has 0 excecutes
 * @param[in]  ADO        = Address Offset, slave memory address
 * @param[in]  length     = length of databuffer
 * @param[out] data       = databuffer to put slave data in
 * @param[in]  timeout    = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::APRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	int wkc;
	uint8 idx;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_APRD, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** APRMW "auto increment address read, multiple write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]  ADP        = Address Position, each slave ++, slave that has 0 reads,
 *                          following slaves write.
 * @param[in]  ADO        = Address Offset, slave memory address
 * @param[in]  length     = length of databuffer
 * @param[out] data       = databuffer to put slave data in
 * @param[in]  timeout    = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::ARMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	int wkc;
	uint8 idx;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_ARMW, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** FPRMW "configured address read, multiple write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]  ADP        = Address Position, slave that has address reads,
 *                          following slaves write.
 * @param[in]  ADO        = Address Offset, slave memory address
 * @param[in]  length     = length of databuffer
 * @param[out] data       = databuffer to put slave data in
 * @param[in]  timeout    = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::FRMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	int wkc;
	uint8 idx;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_FRMW, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** APRDw "auto increment address read" word return primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, each slave ++, slave that has 0 reads.
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return word data from slave
 */
uint16 ecx_portt::APRDw(uint16 ADP, uint16 ADO, int timeout)
{
	uint16 w;

	w = 0;
	APRD(ADP, ADO, sizeof(w), &w, timeout);

	return w;
}

/** FPRD "configured address read" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]  ADP        = Address Position, slave that has address reads.
 * @param[in]  ADO        = Address Offset, slave memory address
 * @param[in]  length     = length of databuffer
 * @param[out] data       = databuffer to put slave data in
 * @param[in]  timeout    = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::FPRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	int wkc;
	uint8 idx;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_FPRD, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** FPRDw "configured address read" word return primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, slave that has address reads.
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return word data from slave
 */
uint16 ecx_portt::FPRDw(uint16 ADP, uint16 ADO, int timeout)
{
	uint16 w;

	w = 0;
	FPRD(ADP, ADO, sizeof(w), &w, timeout);
	return w;
}

/** APWR "auto increment address write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, each slave ++, slave that has 0 writes.
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] length      = length of databuffer
 * @param[in] data        = databuffer to write to slave.
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::APWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	uint8 idx;
	int wkc;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_APWR, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** APWRw "auto increment address write" word primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, each slave ++, slave that has 0 writes.
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] data        = word data to write to slave.
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::APWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
	return APWR(ADP, ADO, sizeof(data), &data, timeout);
}

/** FPWR "configured address write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, slave that has address writes.
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] length      = length of databuffer
 * @param[in] data        = databuffer to write to slave.
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::FPWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	int wkc;
	uint8 idx;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_FPWR, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** FPWR "configured address write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] ADP         = Address Position, slave that has address writes.
 * @param[in] ADO         = Address Offset, slave memory address
 * @param[in] data        = word to write to slave.
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::FPWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
	return FPWR(ADP, ADO, sizeof(data), &data, timeout);
}

/** LRW "logical memory read / write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]     LogAdr  = Logical memory address
 * @param[in]     length  = length of databuffer
 * @param[in,out] data    = databuffer to write to and read from slave.
 * @param[in]     timeout = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::LRW(uint32 LogAdr, uint16 length, void *data, int timeout)
{
	uint8 idx;
	int wkc;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_LRW, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	wkc = srconfirm(idx, timeout);
	if ((wkc > 0) && (this->rxbuf[idx][EC_CMDOFFSET] == EC_CMD_LRW))
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** LRD "logical memory read" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in]  LogAdr     = Logical memory address
 * @param[in]  length     = length of bytes to read from slave.
 * @param[out] data       = databuffer to read from slave.
 * @param[in]  timeout    = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::LRD(uint32 LogAdr, uint16 length, void *data, int timeout)
{
	uint8 idx;
	int wkc;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_LRD, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	wkc = srconfirm(idx, timeout);
	if ((wkc > 0) && (this->rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LRD))
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** LWR "logical memory write" primitive. Blocking.
 *
 * @param[in] port        = port context struct
 * @param[in] LogAdr      = Logical memory address
 * @param[in] length      = length of databuffer
 * @param[in] data        = databuffer to write to slave.
 * @param[in] timeout     = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::LWR(uint32 LogAdr, uint16 length, void *data, int timeout)
{
	uint8 idx;
	int wkc;

	idx = getindex();
	setupdatagram(&(this->txbuf[idx]), EC_CMD_LWR, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	wkc = srconfirm(idx, timeout);
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}

/** LRW "logical memory read / write" primitive plus Clock Distribution. Blocking.
 * Frame consists of two datagrams, one LRW and one FPRMW.
 *
 * @param[in] port        = port context struct
 * @param[in]     LogAdr  = Logical memory address
 * @param[in]     length  = length of databuffer
 * @param[in,out] data    = databuffer to write to and read from slave.
 * @param[in]     DCrs    = Distributed Clock reference slave address.
 * @param[out]    DCtime  = DC time read from reference slave.
 * @param[in]     timeout = timeout in us, standard is EC_TIMEOUTRET
 * @return Workcounter or EC_NOFRAME
 */
int ecx_portt::LRWDC(uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout)
{
	uint16 DCtO;
	uint8 idx;
	int wkc;
	uint64 DCtE;

	idx = getindex();
	/* LRW in first datagram */
	setupdatagram(&(this->txbuf[idx]), EC_CMD_LRW, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	/* FPRMW in second datagram */
	DCtE = htoell(*DCtime);
	DCtO = adddatagram(&(this->txbuf[idx]), EC_CMD_FRMW, idx, FALSE, DCrs, ECT_REG_DCSYSTIME, sizeof(DCtime), &DCtE);
	wkc = srconfirm(idx, timeout);
	if ((wkc > 0) && (this->rxbuf[idx][EC_CMDOFFSET] == EC_CMD_LRW))
	{
			memcpy(data, &(this->rxbuf[idx][EC_HEADERSIZE]), length);
			memcpy(&wkc, &(this->rxbuf[idx][EC_HEADERSIZE + length]), EC_WKCSIZE);
			memcpy(&DCtE, &(this->rxbuf[idx][DCtO]), sizeof(*DCtime));
			*DCtime = etohll(DCtE);
	}
	setbufstat(idx, EC_BUF_EMPTY);

	return wkc;
}
/** Basic setup to connect NIC to socket.
* @param[in] port        = port context struct
* @param[in] ifname       = Name of NIC device, f.e. "eth0"
* @param[in] secondary      = if >0 then use secondary stack instead of primary
* @return >0 if succeeded
*/
int ecx_portt::setupnic(const char *ifname, int secondary)
{
	int i, rval;
	pcap_t **psock;

	rval = 0;
	if (secondary)
	{
		/* secondary port struct available? */
		if (this->redport)
		{
			/* when using secondary socket it is automatically a redundant setup */
			psock = &(this->redport->sockhandle);
			*psock = NULL;
			this->redstate = ECT_RED_DOUBLE;
			this->redport->stack.sock = &(this->redport->sockhandle);
			this->redport->stack.txbuf = &(this->txbuf);
			this->redport->stack.txbuflength = &(this->txbuflength);
			this->redport->stack.tempbuf = &(this->redport->tempinbuf);
			this->redport->stack.rxbuf = &(this->redport->rxbuf);
			this->redport->stack.rxbufstat = &(this->redport->rxbufstat);
			this->redport->stack.rxsa = &(this->redport->rxsa);
			ecx_clear_rxbufstat(&(this->redport->rxbufstat[0]));
		}
		else
		{
			/* fail */
			return 0;
		}
	}
	else
	{
		InitializeCriticalSection(&(this->getindex_mutex));
		InitializeCriticalSection(&(this->tx_mutex));
		InitializeCriticalSection(&(this->rx_mutex));
		this->sockhandle = NULL;
		this->lastidx = 0;
		this->redstate = ECT_RED_NONE;
		this->stack.sock = &(this->sockhandle);
		this->stack.txbuf = &(this->txbuf);
		this->stack.txbuflength = &(this->txbuflength);
		this->stack.tempbuf = &(this->tempinbuf);
		this->stack.rxbuf = &(this->rxbuf);
		this->stack.rxbufstat = &(this->rxbufstat);
		this->stack.rxsa = &(this->rxsa);
		ecx_clear_rxbufstat(&(this->rxbufstat[0]));
		psock = &(this->sockhandle);
	}
	/* we use pcap socket to send RAW packets in windows user mode*/
	*psock = pcap_open(ifname, 65536, PCAP_OPENFLAG_PROMISCUOUS |
		PCAP_OPENFLAG_MAX_RESPONSIVENESS |
		PCAP_OPENFLAG_NOCAPTURE_LOCAL, -1, NULL, errbuf);
	if (NULL == *psock)
	{
		printf("interface %s could not open with pcap\n", ifname);
		return 0;
	}

	for (i = 0; i < EC_MAXBUF; i++)
	{
		ec_setupheader(&(this->txbuf[i]));
		this->rxbufstat[i] = EC_BUF_EMPTY;
	}
	ec_setupheader(&(this->txbuf2));

	return 1;
}
/** Close sockets used
* @param[in] port        = port context struct
* @return 0
*/
int ecx_portt::closenic()
{
	timeEndPeriod(15);

	if (this->sockhandle != NULL)
	{
		DeleteCriticalSection(&(this->getindex_mutex));
		DeleteCriticalSection(&(this->tx_mutex));
		DeleteCriticalSection(&(this->rx_mutex));
		pcap_close(this->sockhandle);
		this->sockhandle = NULL;
	}
	if ((this->redport) && (this->redport->sockhandle != NULL))
	{
		pcap_close(this->redport->sockhandle);
		this->redport->sockhandle = NULL;
	}

	return 0;
}
/** Get new frame identifier index and allocate corresponding rx buffer.
* @param[in] port        = port context struct
* @return new index.
*/
int ecx_portt::getindex()
{
	int idx;
	int cnt;

	EnterCriticalSection(&(this->getindex_mutex));

	idx = this->lastidx + 1;
	/* index can't be larger than buffer array */
	if (idx >= EC_MAXBUF)
	{
		idx = 0;
	}
	cnt = 0;
	/* try to find unused index */
	while ((this->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF))
	{
		idx++;
		cnt++;
		if (idx >= EC_MAXBUF)
		{
			idx = 0;
		}
	}
	this->rxbufstat[idx] = EC_BUF_ALLOC;
	if (this->redstate != ECT_RED_NONE)
		this->redport->rxbufstat[idx] = EC_BUF_ALLOC;
	this->lastidx = idx;

	LeaveCriticalSection(&(this->getindex_mutex));

	return idx;
}
/** Set rx buffer status.
* @param[in] port        = port context struct
* @param[in] idx      = index in buffer array
* @param[in] bufstat   = status to set
*/
void ecx_portt::setbufstat(int idx, int bufstat)
{
	this->rxbufstat[idx] = bufstat;
	if (this->redstate != ECT_RED_NONE)
		this->redport->rxbufstat[idx] = bufstat;
}
/** Transmit buffer over socket (non blocking).
* @param[in] port        = port context struct
* @param[in] idx      = index in tx buffer array
* @param[in] stacknumber   = 0=Primary 1=Secondary stack
* @return socket send result
*/
int ecx_portt::outframe(int idx, int stacknumber)
{
	int lp, rval;
	ec_stackT *stack;

	if (!stacknumber)
	{
		stack = &(this->stack);
	}
	else
	{
		stack = &(this->redport->stack);
	}
	lp = (*stack->txbuflength)[idx];
	rval = pcap_sendpacket(*stack->sock, (*stack->txbuf)[idx], lp);
	(*stack->rxbufstat)[idx] = EC_BUF_TX;

	return rval;
}
/** Transmit buffer over socket (non blocking).
* @param[in] port        = port context struct
* @param[in] idx      = index in tx buffer array
* @return socket send result
*/
int ecx_portt::outframe_red(int idx)
{
	ec_comt *datagramP;
	ec_etherheadert *ehp;
	int rval;

	ehp = (ec_etherheadert *)&(this->txbuf[idx]);
	/* rewrite MAC source address 1 to primary */
	ehp->sa1 = htons(priMAC[1]);
	/* transmit over primary socket*/
	rval = outframe(idx, 0);
	if (this->redstate != ECT_RED_NONE)
	{
		EnterCriticalSection(&(this->tx_mutex));
		ehp = (ec_etherheadert *)&(this->txbuf2);
		/* use dummy frame for secondary socket transmit (BRD) */
		datagramP = (ec_comt*)&(this->txbuf2[ETH_HEADERSIZE]);
		/* write index to frame */
		datagramP->index = idx;
		/* rewrite MAC source address 1 to secondary */
		ehp->sa1 = htons(secMAC[1]);
		/* transmit over secondary socket */
		pcap_sendpacket(this->redport->sockhandle, (u_char const *)&(this->txbuf2), this->txbuflength2);
		LeaveCriticalSection(&(this->tx_mutex));
		this->redport->rxbufstat[idx] = EC_BUF_TX;
	}

	return rval;
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
* read frame with transmitted frame. To compensate for received frames that
* are out-of-order all frames are stored in their respective indexed buffer.
* If a frame was placed in the buffer previously, the function retreives it
* from that buffer index without calling ec_recvpkt. If the requested index
* is not already in the buffer it calls ec_recvpkt to fetch it. There are
* three options now, 1 no frame read, so exit. 2 frame read but other
* than requested index, store in buffer and exit. 3 frame read with matching
* index, store in buffer, set completed flag in buffer status and exit.
*
* @param[in] port        = port context struct
* @param[in] idx         = requested index of frame
* @param[in] stacknumber  = 0=primary 1=secondary stack
* @return Workcounter if a frame is found with corresponding index, otherwise
* EC_NOFRAME or EC_OTHERFRAME.
*/
int ecx_portt::inframe(int idx, int stacknumber)
{
	uint16  l;
	int     rval;
	int     idxf;
	ec_etherheadert *ehp;
	ec_comt *ecp;
	ec_stackT *stack;
	ec_bufT *rxbuf;

	if (!stacknumber)
	{
		stack = &(this->stack);
	}
	else
	{
		stack = &(this->redport->stack);
	}
	rval = EC_NOFRAME;
	rxbuf = &(*stack->rxbuf)[idx];
	/* check if requested index is already in buffer ? */
	if ((idx < EC_MAXBUF) && ((*stack->rxbufstat)[idx] == EC_BUF_RCVD))
	{
		l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
		/* return WKC */
		rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
		/* mark as completed */
		(*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
	}
	else
	{
		EnterCriticalSection(&(this->rx_mutex));
		/* non blocking call to retrieve frame from socket */
		if (recvpkt(stacknumber))
		{
			rval = EC_OTHERFRAME;
			ehp = (ec_etherheadert*)(stack->tempbuf);
			/* check if it is an EtherCAT frame */
			if (ehp->etype == htons(ETH_P_ECAT))
			{
				ecp = (ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
				l = etohs(ecp->elength) & 0x0fff;
				idxf = ecp->index;
				/* found index equals reqested index ? */
				if (idxf == idx)
				{
					/* yes, put it in the buffer array (strip ethernet header) */
					memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
					/* return WKC */
					rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
					/* mark as completed */
					(*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
					/* store MAC source word 1 for redundant routing info */
					(*stack->rxsa)[idx] = ntohs(ehp->sa1);
				}
				else
				{
					/* check if index exist and someone is waiting for it */
					if (idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
					{
						rxbuf = &(*stack->rxbuf)[idxf];
						/* put it in the buffer array (strip ethernet header) */
						memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
						/* mark as received */
						(*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
						(*stack->rxsa)[idxf] = ntohs(ehp->sa1);
					}
					else
					{
						/* strange things happend */
					}
				}
			}
		}
		LeaveCriticalSection(&(this->rx_mutex));

	}

	/* WKC if mathing frame found */
	return rval;
}

/** Non blocking read of socket. Put frame in temporary buffer.
* @param[in] port        = port context struct
* @param[in] stacknumber = 0=primary 1=secondary stack
* @return >0 if frame is available and read
*/
int ecx_portt::recvpkt(int stacknumber)
{
	int lp, bytesrx;
	ec_stackT *stack;
	struct pcap_pkthdr * header;
	unsigned char const * pkt_data;
	int res;

	if (!stacknumber)
	{
		stack = &(this->stack);
	}
	else
	{
		stack = &(this->redport->stack);
	}
	lp = sizeof(this->tempinbuf);

	res = pcap_next_ex(*stack->sock, &header, &pkt_data);
	if (res <= 0)
	{
		this->tempinbufs = 0;
		return 0;
	}
	bytesrx = header->len;
	if (bytesrx > lp)
	{
		bytesrx = lp;
	}
	memcpy(*stack->tempbuf, pkt_data, bytesrx);
	this->tempinbufs = bytesrx;
	return (bytesrx > 0);
}
/** Blocking redundant receive frame function. If redundant mode is not active then
* it skips the secondary stack and redundancy functions. In redundant mode it waits
* for both (primary and secondary) frames to come in. The result goes in an decision
* tree that decides, depending on the route of the packet and its possible missing arrival,
* how to reroute the original packet to get the data in an other try.
*
* @param[in] port        = port context struct
* @param[in] idx = requested index of frame
* @param[in] tvs = timeout
* @return Workcounter if a frame is found with corresponding index, otherwise
* EC_NOFRAME.
*/
int ecx_portt::waitinframe_red(int idx, osal_timert *timer)
{
	osal_timert timer2;
	int wkc = EC_NOFRAME;
	int wkc2 = EC_NOFRAME;
	int primrx, secrx;

	/* if not in redundant mode then always assume secondary is OK */
	if (this->redstate == ECT_RED_NONE)
		wkc2 = 0;
	do
	{
		/* only read frame if not already in */
		if (wkc <= EC_NOFRAME)
			wkc = inframe(idx, 0);
		/* only try secondary if in redundant mode */
		if (this->redstate != ECT_RED_NONE)
		{
			/* only read frame if not already in */
			if (wkc2 <= EC_NOFRAME)
				wkc2 = inframe(idx, 1);
		}
		/* wait for both frames to arrive or timeout */
	} while (((wkc <= EC_NOFRAME) || (wkc2 <= EC_NOFRAME)) && !osal_timer_is_expired(timer));
	/* only do redundant functions when in redundant mode */
	if (this->redstate != ECT_RED_NONE)
	{
		/* primrx if the reveived MAC source on primary socket */
		primrx = 0;
		if (wkc > EC_NOFRAME) primrx = this->rxsa[idx];
		/* secrx if the reveived MAC source on psecondary socket */
		secrx = 0;
		if (wkc2 > EC_NOFRAME) secrx = this->redport->rxsa[idx];

		/* primary socket got secondary frame and secondary socket got primary frame */
		/* normal situation in redundant mode */
		if (((primrx == RX_SEC) && (secrx == RX_PRIM)))
		{
			/* copy secondary buffer to primary */
			memcpy(&(this->rxbuf[idx]), &(this->redport->rxbuf[idx]), this->txbuflength[idx] - ETH_HEADERSIZE);
			wkc = wkc2;
		}
		/* primary socket got nothing or primary frame, and secondary socket got secondary frame */
		/* we need to resend TX packet */
		if (((primrx == 0) && (secrx == RX_SEC)) ||
			((primrx == RX_PRIM) && (secrx == RX_SEC)))
		{
			/* If both primary and secondary have partial connection retransmit the primary received
			* frame over the secondary socket. The result from the secondary received frame is a combined
			* frame that traversed all slaves in standard order. */
			if ((primrx == RX_PRIM) && (secrx == RX_SEC))
			{
				/* copy primary rx to tx buffer */
				memcpy(&(this->txbuf[idx][ETH_HEADERSIZE]), &(this->rxbuf[idx]), this->txbuflength[idx] - ETH_HEADERSIZE);
			}
			osal_timer_start(&timer2, EC_TIMEOUTRET);
			/* resend secondary tx */
			outframe(idx, 1);
			do
			{
				/* retrieve frame */
				wkc2 = inframe(idx, 1);
			} while ((wkc2 <= EC_NOFRAME) && !osal_timer_is_expired(&timer2));
			if (wkc2 > EC_NOFRAME)
			{
				/* copy secondary result to primary rx buffer */
				memcpy(&(this->rxbuf[idx]), &(this->redport->rxbuf[idx]), this->txbuflength[idx] - ETH_HEADERSIZE);
				wkc = wkc2;
			}
		}
	}

	/* return WKC or EC_NOFRAME */
	return wkc;
}
/** Blocking receive frame function. Calls ec_waitinframe_red().
* @param[in] port        = port context struct
* @param[in] idx = requested index of frame
* @param[in] timeout = timeout in us
* @return Workcounter if a frame is found with corresponding index, otherwise
* EC_NOFRAME.
*/
int ecx_portt::waitinframe(int idx, int timeout)
{
	int wkc;
	osal_timert timer;

	osal_timer_start(&timer, timeout);
	wkc = waitinframe_red(idx, &timer);

	return wkc;
}
/** Blocking send and recieve frame function. Used for non processdata frames.
* A datagram is build into a frame and transmitted via this function. It waits
* for an answer and returns the workcounter. The function retries if time is
* left and the result is WKC=0 or no frame received.
*
* The function calls ec_outframe_red() and ec_waitinframe_red().
*
* @param[in] port        = port context struct
* @param[in] idx      = index of frame
* @param[in] timeout  = timeout in us
* @return Workcounter or EC_NOFRAME
*/
int ecx_portt::srconfirm(int idx, int timeout)
{
	int wkc = EC_NOFRAME;
	osal_timert timer1, timer2;

	osal_timer_start(&timer1, timeout);
	do
	{
		/* tx frame on primary and if in redundant mode a dummy on secondary */
		outframe_red(idx);
		if (timeout < EC_TIMEOUTRET)
		{
			osal_timer_start(&timer2, timeout);
		}
		else
		{
			/* normally use partial timout for rx */
			osal_timer_start(&timer2, EC_TIMEOUTRET);
		}
		/* get frame from primary or if in redundant mode possibly from secondary */
		wkc = waitinframe_red(idx, &timer2);
		/* wait for answer with WKC>=0 or otherwise retry until timeout */
	} while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired(&timer1));
	/* if nothing received, clear buffer index status so it can be used again */
	if (wkc <= EC_NOFRAME)
	{
		ec_setbufstat(idx, EC_BUF_EMPTY);
	}

	return wkc;
}
#ifdef EC_VER1
int ec::setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
	return ecx_portt::getInstance()->setupdatagram (frame, com, idx, ADP, ADO, length, data);
}

int ec::adddatagram (void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
	return ecx_portt::getInstance()->adddatagram (frame, com, idx, more, ADP, ADO, length, data);
}

int ec::BWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->BWR (ADP, ADO, length, data, timeout);
}

int ec::BRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->BRD(ADP, ADO, length, data, timeout);
}

int ec::APRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->APRD(ADP, ADO, length, data, timeout);
}

int ec::ARMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->ARMW(ADP, ADO, length, data, timeout);
}

int ec::FRMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->FRMW(ADP, ADO, length, data, timeout);
}

uint16 ec::APRDw(uint16 ADP, uint16 ADO, int timeout)
{
	uint16 w;

	w = 0;
	ec::APRD(ADP, ADO, sizeof(w), &w, timeout);

	return w;
}

int ec::FPRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->FPRD(ADP, ADO, length, data, timeout);
}

uint16 ec::FPRDw(uint16 ADP, uint16 ADO, int timeout)
{
	uint16 w;

	w = 0;
	ec::FPRD(ADP, ADO, sizeof(w), &w, timeout);
	return w;
}

int ec::APWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->APWR(ADP, ADO, length, data, timeout);
}

int ec::APWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
	return ec::APWR(ADP, ADO, sizeof(data), &data, timeout);
}

int ec::FPWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->FPWR(ADP, ADO, length, data, timeout);
}

int ec::FPWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
	return ec::FPWR(ADP, ADO, sizeof(data), &data, timeout);
}

int ec::LRW(uint32 LogAdr, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->LRW(LogAdr, length, data, timeout);
}

int ec::LRD(uint32 LogAdr, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->LRD(LogAdr, length, data, timeout);
}

int ec::LWR(uint32 LogAdr, uint16 length, void *data, int timeout)
{
	return ecx_portt::getInstance()->LWR(LogAdr, length, data, timeout);
}

int ec::LRWDC(uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout)
{
	return ecx_portt::getInstance()->LRWDC(LogAdr, length, data, DCrs, DCtime, timeout);
}
#endif
