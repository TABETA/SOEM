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
void ecx::writedatagramdata(void *datagramdata, ec_cmdtype com, uint16 length, const void * data)
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
int ecx::setupdatagram(ecx_portt *port, void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
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
   ecx::writedatagramdata(&frameP[ETH_HEADERSIZE + EC_HEADERSIZE], static_cast<ec_cmdtype>(com), length, data);
   /* set WKC to zero */
   frameP[ETH_HEADERSIZE + EC_HEADERSIZE + length] = 0x00;
   frameP[ETH_HEADERSIZE + EC_HEADERSIZE + length + 1] = 0x00;
   /* set size of frame in buffer array */
   port->txbuflength[idx] = ETH_HEADERSIZE + EC_HEADERSIZE + EC_WKCSIZE + length;

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
int ecx::adddatagram(ecx_portt *port, void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
   ec_comt *datagramP;
   uint8 *frameP;
   uint16 prevlength;

   frameP = static_cast<uint8 *>(frame);
   /* copy previous frame size */
   prevlength = port->txbuflength[idx];
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
   ecx::writedatagramdata(&frameP[prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE], static_cast<ec_cmdtype>(com), length, data);
   /* set WKC to zero */
   frameP[prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE + length] = 0x00;
   frameP[prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE + length + 1] = 0x00;
   /* set size of frame in buffer array */
   port->txbuflength[idx] = prevlength + EC_HEADERSIZE - EC_ELENGTHSIZE + EC_WKCSIZE + length;

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
int ecx::BWR (ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   uint8 idx;
   int wkc;

   /* get fresh index */
   idx = ecx_getindex (port);
   /* setup datagram */
   ecx::setupdatagram (port, &(port->txbuf[idx]), EC_CMD_BWR, idx, ADP, ADO, length, data);
   /* send data and wait for answer */
   wkc = ecx_srconfirm (port, idx, timeout);
   /* clear buffer status */
   ecx_setbufstat (port, idx, EC_BUF_EMPTY);

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
int ecx::BRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   uint8 idx;
   int wkc;

   /* get fresh index */
   idx = ecx_getindex(port);
   /* setup datagram */
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_BRD, idx, ADP, ADO, length, data);
   /* send data and wait for answer */
   wkc = ecx_srconfirm (port, idx, timeout);
   if (wkc > 0)
   {
      /* copy datagram to data buffer */
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   /* clear buffer status */
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::APRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   int wkc;
   uint8 idx;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_APRD, idx, ADP, ADO, length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   if (wkc > 0)
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::ARMW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   int wkc;
   uint8 idx;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_ARMW, idx, ADP, ADO, length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   if (wkc > 0)
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::FRMW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   int wkc;
   uint8 idx;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_FRMW, idx, ADP, ADO, length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   if (wkc > 0)
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
uint16 ecx::APRDw(ecx_portt *port, uint16 ADP, uint16 ADO, int timeout)
{
   uint16 w;

   w = 0;
   ecx::APRD(port, ADP, ADO, sizeof(w), &w, timeout);

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
int ecx::FPRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   int wkc;
   uint8 idx;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_FPRD, idx, ADP, ADO, length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   if (wkc > 0)
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
uint16 ecx::FPRDw(ecx_portt *port, uint16 ADP, uint16 ADO, int timeout)
{
   uint16 w;

   w = 0;
   ecx::FPRD(port, ADP, ADO, sizeof(w), &w, timeout);
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
int ecx::APWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   uint8 idx;
   int wkc;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_APWR, idx, ADP, ADO, length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::APWRw(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
   return ecx::APWR(port, ADP, ADO, sizeof(data), &data, timeout);
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
int ecx::FPWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   int wkc;
   uint8 idx;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_FPWR, idx, ADP, ADO, length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::FPWRw(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
   return ecx::FPWR(port, ADP, ADO, sizeof(data), &data, timeout);
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
int ecx::LRW(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout)
{
   uint8 idx;
   int wkc;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_LRW, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   if ((wkc > 0) && (port->rxbuf[idx][EC_CMDOFFSET] == EC_CMD_LRW))
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::LRD(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout)
{
   uint8 idx;
   int wkc;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_LRD, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   if ((wkc > 0) && (port->rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LRD))
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::LWR(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout)
{
   uint8 idx;
   int wkc;

   idx = ecx_getindex(port);
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_LWR, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
   wkc = ecx_srconfirm(port, idx, timeout);
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

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
int ecx::LRWDC(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout)
{
   uint16 DCtO;
   uint8 idx;
   int wkc;
   uint64 DCtE;

   idx = ecx_getindex(port);
   /* LRW in first datagram */
   ecx::setupdatagram(port, &(port->txbuf[idx]), EC_CMD_LRW, idx, LO_WORD(LogAdr), HI_WORD(LogAdr), length, data);
   /* FPRMW in second datagram */
   DCtE = htoell(*DCtime);
   DCtO = ecx::adddatagram(port, &(port->txbuf[idx]), EC_CMD_FRMW, idx, FALSE, DCrs, ECT_REG_DCSYSTIME, sizeof(DCtime), &DCtE);
   wkc = ecx_srconfirm(port, idx, timeout);
   if ((wkc > 0) && (port->rxbuf[idx][EC_CMDOFFSET] == EC_CMD_LRW))
   {
      memcpy(data, &(port->rxbuf[idx][EC_HEADERSIZE]), length);
      memcpy(&wkc, &(port->rxbuf[idx][EC_HEADERSIZE + length]), EC_WKCSIZE);
      memcpy(&DCtE, &(port->rxbuf[idx][DCtO]), sizeof(*DCtime));
      *DCtime = etohll(DCtE);
   }
   ecx_setbufstat(port, idx, EC_BUF_EMPTY);

   return wkc;
}

#ifdef EC_VER1
int ec::setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
   return ecx::setupdatagram (&ecx_port, frame, com, idx, ADP, ADO, length, data);
}

int ec::adddatagram (void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
   return ecx::adddatagram (&ecx_port, frame, com, idx, more, ADP, ADO, length, data);
}

int ec::BWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   return ecx::BWR (&ecx_port, ADP, ADO, length, data, timeout);
}

int ec::BRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   return ecx::BRD(&ecx_port, ADP, ADO, length, data, timeout);
}

int ec::APRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   return ecx::APRD(&ecx_port, ADP, ADO, length, data, timeout);
}

int ec::ARMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   return ecx::ARMW(&ecx_port, ADP, ADO, length, data, timeout);
}

int ec::FRMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   return ecx::FRMW(&ecx_port, ADP, ADO, length, data, timeout);
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
   return ecx::FPRD(&ecx_port, ADP, ADO, length, data, timeout);
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
   return ecx::APWR(&ecx_port, ADP, ADO, length, data, timeout);
}

int ec::APWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
   return ec::APWR(ADP, ADO, sizeof(data), &data, timeout);
}

int ec::FPWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
   return ecx::FPWR(&ecx_port, ADP, ADO, length, data, timeout);
}

int ec::FPWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout)
{
   return ec::FPWR(ADP, ADO, sizeof(data), &data, timeout);
}

int ec::LRW(uint32 LogAdr, uint16 length, void *data, int timeout)
{
   return ecx::LRW(&ecx_port, LogAdr, length, data, timeout);
}

int ec::LRD(uint32 LogAdr, uint16 length, void *data, int timeout)
{
   return ecx::LRD(&ecx_port, LogAdr, length, data, timeout);
}

int ec::LWR(uint32 LogAdr, uint16 length, void *data, int timeout)
{
   return ecx::LWR(&ecx_port, LogAdr, length, data, timeout);
}

int ec::LRWDC(uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout)
{
   return ecx::LRWDC(&ecx_port, LogAdr, length, data, DCrs, DCtime, timeout);
}
#endif