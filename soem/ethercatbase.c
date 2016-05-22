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
	void* p = static_cast<uint8 *>(frame)+sizeof(EtherNetHeader);
	/* Ethernet header is preset and fixed in frame buffers EtherCAT header needs to be added after that */
	p = EC_Header::set(p, com, idx, ADP, ADO, length);

	p = ec_bufT::writedatagramdata(static_cast<ec_bufT*>(p), static_cast<ec_cmdtype>(com), length, data);

	/* set WKC to zero */
	p = WorkCounter::clear(p);

	/* set size of frame in buffer array */
	txbuflength[idx] = static_cast<uint8*>(p)-static_cast<uint8*>(frame);

	return 0;
}
uint16 EC_Header::getElength()
{
	return etohs(elength_) & 0x0fff;
}
void* EC_Header::set(void *ecatHeader, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length)
{
	EC_Header* self = static_cast<EC_Header*>(ecatHeader);
	self->elength_ = htoes(EC_ECATTYPE + sizeof(EC_Header)+length);
	self->command_ = com;
	self->index_ = idx;
	self->ADP_ = htoes(ADP);
	self->ADO_ = htoes(ADO);
	self->dlength_ = htoes(length);
	//return self + 1;
	return static_cast<uint8*>(ecatHeader) + sizeof(EC_Header);
}
void* EC_Header::add(void *ecatHeader, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, int prevlength)
{
	EC_Header* self = static_cast<EC_Header*>(ecatHeader);
	/* add new datagram to ethernet frame size */
	self->elength_ = htoes(etohs(self->elength_) + sizeof(EC_Header)+length);
	/* add "datagram follows" flag to previous subframe dlength */
	self->dlength_ = htoes(etohs(self->dlength_) | EC_DATAGRAMFOLLOWS);
	/* set new EtherCAT header position */

	size_t offset = (prevlength - sizeof(EtherNetHeader)) - sizeof(self->elength_);
	self = reinterpret_cast<EC_Header*>(static_cast<uint8*>(ecatHeader)+offset);
	self->command_ = com;
	self->index_ = idx;
	self->ADP_ = htoes(ADP);
	self->ADO_ = htoes(ADO);
	if (more)
	{
		/* this is not the last datagram to add */
		self->dlength_ = htoes(length | EC_DATAGRAMFOLLOWS);
	}
	else
	{
		/* this is the last datagram in the frame */
		self->dlength_ = htoes(length);
	}
	return reinterpret_cast<uint8*>(self)+sizeof(EC_Header);
}
/** Write data to EtherCAT datagram.
*
* @param[out] datagramdata   = data part of datagram
* @param[in]  com            = command
* @param[in]  length         = length of databuffer
* @param[in]  data           = databuffer to be copied into datagram
*/
void* ec_bufT::writedatagramdata(void *datagramdata, ec_cmdtype com, uint16 length, const void * data)
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
	return static_cast<uint8*>(datagramdata)+length;
}
ec_cmdtype ec_bufT::getCommand()const
{
	const EC_Header*  header = reinterpret_cast<const EC_Header* >(this);
	return header->getCommand();
}
void ec_bufT::copyHeader(void *dst, size_t datalength)const
{
	memcpy(dst, &(buf_[sizeof(EC_Header)]), datalength);
}
int ec_bufT::getWorkCounterFromTempBuf(size_t elength)const
{
	return (buf_[elength] + (static_cast<uint16>(buf_[elength + 1]) << 8));
}
int ec_bufT::getWorkCounter(size_t datalength)const
{
	int dst = 0;
	memcpy(&dst, &(buf_[sizeof(EC_Header)+datalength]), sizeof(WorkCounter));
	return dst;
}
int64 ec_bufT::getDCTime(size_t DCtO)const
{
	uint64 DCtE = 0;
	memcpy(&DCtE, &(buf_[DCtO]), sizeof(int64));
	return etohll(DCtE);
}
int ec_bufT::setupdatagram(uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data, int* txbuflength)
{
	void* p = reinterpret_cast<uint8 *>(this) + sizeof(EtherNetHeader);
	/* Ethernet header is preset and fixed in frame buffers EtherCAT header needs to be added after that */
	p = EC_Header::set(p, com, idx, ADP, ADO, length);

	p = ec_bufT::writedatagramdata(static_cast<ec_bufT*>(p), static_cast<ec_cmdtype>(com), length, data);

	/* set WKC to zero */
	p = WorkCounter::clear(p);

	/* set size of frame in buffer array */
	*txbuflength = static_cast<uint8*>(p)-reinterpret_cast<uint8*>(this);

	return 0;
}
int ecx_portt::setupdatagram(uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
	return txbuf[idx].setupdatagram(com, idx, ADP, ADO, length, data, &txbuflength[idx]);
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
	
	/* copy previous frame size */
	void* p = static_cast<uint8 *>(frame)+sizeof(EtherNetHeader);
	uint16 prevlength = txbuflength[idx];
	int rval = static_cast<uint8*>(p)-static_cast<uint8*>(frame);
	p = EC_Header::add(p, com, idx, more, ADP, ADO, length, prevlength);


	p = ec_bufT::writedatagramdata(p, static_cast<ec_cmdtype>(com), length, data);
	/* set WKC to zero */
	p = WorkCounter::clear(p);
	/* set size of frame in buffer array */
	txbuflength[idx] = static_cast<uint8*>(p)-static_cast<uint8*>(frame);

	/* return offset to data in rx frame
			14 bytes smaller than tx frame due to stripping of ethernet header */
	return rval;
}

int ecx_portt::adddatagram(uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
	return txbuf[idx].adddatagram(com, idx, more, ADP, ADO, length, data, &txbuflength[idx]);
}

int ec_bufT::adddatagram(uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data, int* txbuflength)
{
	/* copy previous frame size */
	void* p = reinterpret_cast<uint8*>(this) + sizeof(EtherNetHeader);
	uint16 prevlength = *txbuflength;
	int rval = static_cast<uint8*>(p)-reinterpret_cast<uint8*>(this);
	p = EC_Header::add(p, com, idx, more, ADP, ADO, length, prevlength);

	p = ec_bufT::writedatagramdata(p, static_cast<ec_cmdtype>(com), length, data);
	/* set WKC to zero */
	p = WorkCounter::clear(p);
	/* set size of frame in buffer array */
	*txbuflength = static_cast<uint8*>(p)-reinterpret_cast<uint8*>(this);

	/* return offset to data in rx frame
	14 bytes smaller than tx frame due to stripping of ethernet header */
	return rval;
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
	setupdatagram(&(txbuf[idx]), EC_CMD_BWR, idx, ADP, ADO, length, data);
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
	setupdatagram(EC_CMD_BRD, idx, ADP, ADO, length, data);
	/* send data and wait for answer */
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
		/* copy datagram to data buffer */
		rxbuf[idx].copyHeader(data, length);
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
	setupdatagram(EC_CMD_APRD, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
		rxbuf[idx].copyHeader(data, length);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_ARMW, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
		rxbuf[idx].copyHeader(data, length);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_FRMW, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
		rxbuf[idx].copyHeader(data, length);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_FPRD, idx, ADP, ADO, length, data);
	wkc = srconfirm(idx, timeout);
	if (wkc > 0)
	{
		rxbuf[idx].copyHeader(data, length);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_APWR, idx, ADP, ADO, length, data);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_FPWR, idx, ADP, ADO, length, data);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_LRW, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	wkc = srconfirm(idx, timeout);
	if ((wkc > 0) && (rxbuf[idx].getCommand() == EC_CMD_LRW))
	{
		rxbuf[idx].copyHeader(data, length);
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
	setupdatagram(&(txbuf[idx]), EC_CMD_LRD, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	wkc = srconfirm(idx, timeout);
	if ((wkc > 0) && (rxbuf[idx].getCommand() == EC_CMD_LRD))
	{
		rxbuf[idx].copyHeader(data, length);
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
	txbuf[idx].setupdatagram(EC_CMD_LWR, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data, &txbuflength[idx]);
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
	uint8 idx = getindex();
	/* LRW in first datagram */
	setupdatagram(&(txbuf[idx]), EC_CMD_LRW, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
	/* FPRMW in second datagram */
	uint64 DCtE = htoell(*DCtime);
	uint16 DCtO = adddatagram(EC_CMD_FRMW, idx, FALSE, DCrs, ECT_REG_DCSYSTIME, sizeof(DCtime), &DCtE);
	int wkc = srconfirm(idx, timeout);
	if ((wkc > 0) && (rxbuf[idx].getCommand() == EC_CMD_LRW))
	{
		rxbuf[idx].copyHeader(data, length);
		wkc = rxbuf[idx].getWorkCounter(length);
		*DCtime = rxbuf[idx].getDCTime(DCtO);
		
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
int ecx_portt::setupnic(const char *ifname, pcap_t **psock)
{
	/* we use pcap socket to send RAW packets in windows user mode*/
	*psock = pcap_open(ifname, 65536, PCAP_OPENFLAG_PROMISCUOUS |
		PCAP_OPENFLAG_MAX_RESPONSIVENESS |
		PCAP_OPENFLAG_NOCAPTURE_LOCAL, -1, NULL, errbuf);
	if (NULL == *psock)
	{
		printf("interface %s could not open with pcap\n", ifname);
		return 0;
	}

	for (int i = 0; i < EC_MAXBUF; i++)
	{
		ec_setupheader(&(this->txbuf[i]));
		this->rxbufstat[i] = EC_BUF_EMPTY;
	}
	ec_setupheader(&(this->txbuf2));

	return 1;
}
int ecx_portt::setupnicPrimary(const char *ifname)
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
	return setupnic(ifname, &(this->sockhandle));
}
int ecx_portt::init_redundant(ecx_redportt *redport, const char *ifname, char *if2name)
{

	redport = redport;
	setupnicPrimary(ifname);
	int rval = setupnicSecondary(if2name);
	/* prepare "dummy" BRD tx frame for redundant operation */
	EtherNetHeader *ehp = (EtherNetHeader *)&(txbuf2);
	ehp->sa1 = oshw_htons(secMAC[0]);
	int zbuf = 0;
	setupdatagram(&(txbuf2), EC_CMD_BRD, 0, 0x0000, 0x0000, 2, &zbuf);
	txbuflength2 = sizeof(EtherNetHeader) + sizeof(EC_Header) + sizeof(WorkCounter) + 2;
	return rval;
}
int ecx_portt::setupnicSecondary(const char *ifname)
{
	int rval = 0;
	
	/* secondary port struct available? */
	if (this->redport)
	{
		/* when using secondary socket it is automatically a redundant setup */
		pcap_t **psock = &(this->redport->sockhandle);
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
		rval = setupnic(ifname, psock);
	}
	else
	{
		/* fail */
	}
	return rval;
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
	EnterCriticalSection(&(this->getindex_mutex));

	int idx = this->lastidx + 1;
	/* index can't be larger than buffer array */
	if (idx >= EC_MAXBUF)
	{
		idx = 0;
	}
	int cnt = 0;
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
	rval = pcap_sendpacket(*stack->sock, reinterpret_cast<const u_char *>((*stack->txbuf)[idx].getEtherNetHeader()), lp);
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
	EtherNetHeader *ehp = txbuf[idx].getEtherNetHeader();
	/* rewrite MAC source address 1 to primary */
	ehp->sa1 = htons(priMAC[1]);
	/* transmit over primary socket*/
	int rval = outframe(idx, 0);
	if (this->redstate != ECT_RED_NONE)
	{
		EnterCriticalSection(&(this->tx_mutex));
		ehp = (EtherNetHeader *)&(this->txbuf2);
		/* use dummy frame for secondary socket transmit (BRD) */
		EC_Header *datagramP = txbuf2.getECATHeader();
		/* write index to frame */
		datagramP->setIndex(idx);
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
	ec_stackT *stack = (stacknumber == 0)?
						&(this->stack):
						&(this->redport->stack);
	int rval = EC_NOFRAME;
	ec_bufT *rxbuf = &(*stack->rxbuf)[idx];
	/* check if requested index is already in buffer ? */
	if ((idx < EC_MAXBUF) && ((*stack->rxbufstat)[idx] == EC_BUF_RCVD))
	{
		/* return WKC */
		rval = rxbuf->retreive();
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
			EtherNetHeader *ehp = stack->tempbuf->getEtherNetHeader();
			/* check if it is an EtherCAT frame */
			if (ehp->isEtherCATFrame())
			{
				EC_Header *ecp = stack->tempbuf->getECATHeader();
				uint8 idxf = ecp->getIndex();
				/* found index equals reqested index ? */
				if (idxf == idx)
				{
					/* yes, put it in the buffer array (strip ethernet header) */
					memcpy(rxbuf, ecp, (*stack->txbuflength)[idx] - sizeof(EtherNetHeader));
					/* return WKC */
					uint16 l = ecp->getElength();
					rval = rxbuf->getWorkCounterFromTempBuf(l);
					/* mark as completed */
					(*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
					/* store MAC source word 1 for redundant routing info */
					(*stack->rxsa)[idx] = ntohs(ehp->sa1);
				}
				/* check if index exist and someone is waiting for it */
				else if(idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
				{
					rxbuf = &(*stack->rxbuf)[idxf];
					/* put it in the buffer array (strip ethernet header) */
					memcpy(rxbuf, ecp, (*stack->txbuflength)[idxf] - sizeof(EtherNetHeader));
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
	ec_stackT *stack = (stacknumber == 0)?
						&(this->stack):
						&(this->redport->stack);
	int lp = sizeof(this->tempinbuf);

	struct pcap_pkthdr * header;
	const u_char * pkt_data;
	int res = pcap_next_ex(*stack->sock, &header, &pkt_data);
	if (res <= 0)
	{
		this->tempinbufs = 0;
		return 0;
	}
	int bytesrx = header->len;
	if (bytesrx > lp)
	{
		bytesrx = lp;
	}
	memcpy(stack->tempbuf->head(), pkt_data, bytesrx);
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
			memcpy(&(this->rxbuf[idx]), &(this->redport->rxbuf[idx]), this->txbuflength[idx] - sizeof(EtherNetHeader));
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
				memcpy(this->txbuf[idx].getECATHeader(), &(this->rxbuf[idx]), this->txbuflength[idx] - sizeof(EtherNetHeader));
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
				memcpy(&(this->rxbuf[idx]), &(this->redport->rxbuf[idx]), this->txbuflength[idx] - sizeof(EtherNetHeader));
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

#define MAX_FPRD_MULTI 64
int ecx_portt::FPRD_multi(int n, uint16 *configlst, ec_alstatust *slstatlst, int timeout)
{
	int sldatapos[MAX_FPRD_MULTI];

	uint8 idx = getindex();
	for (int slcnt = 0; slcnt < n; ++slcnt)
	{
		if (slcnt == 0)
		{
			txbuf[idx].setupdatagram(EC_CMD_FPRD, idx,
				*(configlst + slcnt), ECT_REG_ALSTAT, sizeof(ec_alstatust), slstatlst + slcnt, &txbuflength[idx]);
			sldatapos[slcnt] = sizeof(EC_Header);
		}
		else
		{
			boolean more = (slcnt == n - 1) ? FALSE : TRUE;
			sldatapos[slcnt] = txbuf[idx].adddatagram(EC_CMD_FPRD, idx, more,
				*(configlst + slcnt), ECT_REG_ALSTAT, sizeof(ec_alstatust), slstatlst + slcnt, &txbuflength[idx]);
		}
	}
	int wkc = srconfirm(idx, timeout);
	if (wkc >= 0)
	{
		for (int slcnt = 0; slcnt < n; slcnt++)
		{
			memcpy(slstatlst + slcnt, (static_cast<uint8*>(rxbuf[idx].head()) + sldatapos[slcnt]), sizeof(ec_alstatust));
		}
	}
	setbufstat(idx, EC_BUF_EMPTY);
	return wkc;
}
void ecx_portt::receive_processdata(int idx, uint16 DCtO, uint16 DCl, uint16 stacklen, int wkc2, void* stackdata, int64* DCtime, int* wkc, int *valid_wkc, boolean* first)
{
	rxbuf[idx].receive_processdata(DCtO, DCl, stacklen, wkc2, stackdata, DCtime, wkc, valid_wkc, first);
}
boolean EtherNetHeader::isEtherCATFrame()const
{
	return etype == htons(ETH_P_ECAT);
}

void ec_bufT::receive_processdata(uint16 DCtO, uint16 DCl, uint16 stacklen, int wkc2, void* stackdata, int64* DCtime, int* wkc, int *valid_wkc, boolean* first)
{
	ec_cmdtype command = getCommand();

	switch (command)
	{
	case EC_CMD_LRD:/* fall-through */
	case EC_CMD_LRW:
		if (*first)
		{
			uint16 le_wkc = 0;
			memcpy(stackdata, &(buf_[sizeof(EC_Header)]), DCl);
			memcpy(&le_wkc, &(buf_[sizeof(EC_Header) + DCl]), sizeof(WorkCounter));
			*wkc = etohs(le_wkc);
			int64 le_DCtime = 0;
			memcpy(&le_DCtime, &(buf_[DCtO]), sizeof(le_DCtime));
			*(DCtime) = etohll(le_DCtime);
			*first = FALSE;
		}
		else
		{
			/* copy input data back to process data buffer */
			memcpy(stackdata, &(buf_[sizeof(EC_Header)]), stacklen);
			*wkc += wkc2;
		}
		*valid_wkc = 1;
		break;
	case EC_CMD_LWR:
		if (*first)
		{
			uint16 le_wkc = 0;
			memcpy(&le_wkc, &(buf_[sizeof(EC_Header) + DCl]), sizeof(WorkCounter));
			/* output WKC counts 2 times when using LRW, emulate the same for LWR */
			*wkc = etohs(le_wkc) * 2;
			int64 le_DCtime = 0;
			memcpy(&le_DCtime, &(buf_[DCtO]), sizeof(le_DCtime));
			*(DCtime) = etohll(le_DCtime);
			*first = FALSE;
		}
		else
		{
			/* output WKC counts 2 times when using LRW, emulate the same for LWR */
			*wkc += wkc2 * 2;
		}
		*valid_wkc = 1;
		break;
	default:
		//nop
		break;
	}
}

#ifdef EC_VER1
//int ec::setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
//{
//	return ecx_portt::getInstance()->setupdatagram(frame, com, idx, ADP, ADO, length, data);
//}

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
