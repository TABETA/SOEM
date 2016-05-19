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

struct ecx_contextt;
class ecx
{
public:
	static int setupdatagram(ecx_portt *port, void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data);
	static int adddatagram(ecx_portt *port, void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data);
	static int BWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int BRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int APRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int ARMW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int FRMW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static uint16 APRDw(ecx_portt *port, uint16 ADP, uint16 ADO, int timeout);
	static int FPRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static uint16 FPRDw(ecx_portt *port, uint16 ADP, uint16 ADO, int timeout);
	static int APWRw(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 data, int timeout);
	static int APWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int FPWRw(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 data, int timeout);
	static int FPWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
	static int LRW(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout);
	static int LRD(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout);
	static int LWR(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout);
	static int LRWDC(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout);
private:
	static void writedatagramdata(void *datagramdata, ec_cmdtype com, uint16 length, const void * data);
};
//#ifdef EC_VER1
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
};
//#endif


#endif
