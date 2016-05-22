/*
 * Simple Open EtherCAT Master Library
 *
 * File    : ethercatmain.c
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

/**
 * \file
 * \brief
 * Main EtherCAT functions.
 *
 * Initialisation, state set and read, mailbox primitives, EEPROM primitives,
 * SII reading and processdata exchange.
 *
 * Defines ec_slave[]. All slave information is put in this structure.
 * Needed for most user interaction with slaves.
 */

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatconfig.h"

/** delay in us for eeprom ready loop */
#define EC_LOCALDELAY  200

/** record for ethercat eeprom communications */
PACKED_BEGIN
typedef struct PACKED
{
   uint16    comm;
   uint16    addr;
   uint16    d2;
} ec_eepromt;
PACKED_END

/** mailbox error structure */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert   MbxHeader;
   uint16          Type;
   uint16          Detail;
} ec_mbxerrort;
PACKED_END

/** emergency request structure */
PACKED_BEGIN
typedef struct PACKED
{
   ec_mbxheadert   MbxHeader;
   uint16          CANOpen;
   uint16          ErrorCode;
   uint8           ErrorReg;
   uint8           bData;
   uint16          w1,w2;
} ec_emcyt;
PACKED_END

#ifdef EC_VER1
/** Main slave data array.
 *  Each slave found on the network gets its own record.
 *  ec_slave[0] is reserved for the master. Structure gets filled
 *  in by the configuration function ec_config().
 */
ec_slavet               ec_slave[EC_MAXSLAVE];
/** number of slaves found on the network */
int                     ec_slavecount;
/** slave group structure */
ec_groupt               ec_group[EC_MAXGROUP];

/** cache for EEPROM read functions */
static uint8            ec_esibuf[EC_MAXEEPBUF];
/** bitmap for filled cache buffer bytes */
static uint32           ec_esimap[EC_MAXEEPBITMAP];
/** current slave for EEPROM cache buffer */
static ec_eringt        ec_elist;
static ec_idxstackT     ec_idxstack;

/** SyncManager Communication Type struct to store data of one slave */
static ec_SMcommtypet   ec_SMcommtype;
/** PDO assign struct to store data of one slave */
static ec_PDOassignt    ec_PDOassign;
/** PDO description struct to store data of one slave */
static ec_PDOdesct      ec_PDOdesc;

/** buffer for EEPROM SM data */
static ec_eepromSMt     ec_SM;
/** buffer for EEPROM FMMU data */
static ec_eepromFMMUt   ec_FMMU;
/** Global variable TRUE if error available in error stack */
boolean                 EcatError = FALSE;

int64                   ec_DCtime;


ecx_contextt* ecx_contextt::getInstance()
{
	static ecx_contextt ins;
	return &ins;
}
ecx_contextt::ecx_contextt():
	port(ecx_portt::getInstance()),
	slavelist(&ec_slave[0]),
	slavecount(&ec_slavecount),
	maxslave(EC_MAXSLAVE),
	grouplist(&ec_group[0]),
	maxgroup(EC_MAXGROUP),
	esibuf(&ec_esibuf[0]),
	esimap(&ec_esimap[0]),
	esislave(0),
	elist(&ec_elist),
	idxstack(&ec_idxstack),
	ecaterror(&EcatError),
	DCtO(0),
	DCl(0),
	DCtime(&ec_DCtime),
	SMcommtype(&ec_SMcommtype),
	PDOassign(&ec_PDOassign),
	PDOdesc(&ec_PDOdesc),
	eepSM(&ec_SM),
	eepFMMU(&ec_FMMU),
	FOEhook(NULL){}

#endif

/** Create list over available network adapters.
 *
 * @return First element in list over available network adapters.
 */
ec_adaptert * ec_find_adapters (void)
{
   ec_adaptert * ret_adapter;

   ret_adapter = oshw_find_adapters ();

   return ret_adapter;
}

/** Free dynamically allocated list over available network adapters.
 *
 * @param[in] adapter = Struct holding adapter name, description and pointer to next.
 */
void ec_free_adapters (ec_adaptert * adapter)
{
   oshw_free_adapters (adapter);
}

/** Pushes an error on the error list.
 *
 * @param[in] context        = context struct
 * @param[in] Ec pointer describing the error.
 */
void ecx_contextt::pusherror( const ec_errort *Ec)
{
   elist->Error[elist->head] = *Ec;
   elist->Error[elist->head].Signal = TRUE;
   elist->head++;
   if (elist->head > EC_MAXELIST)
   {
      elist->head = 0;
   }
   if (elist->head == elist->tail)
   {
      elist->tail++;
   }
   if (elist->tail > EC_MAXELIST)
   {
      elist->tail = 0;
   }
   *(ecaterror) = TRUE;
}

/** Pops an error from the list.
 *
 * @param[in] context        = context struct
 * @param[out] Ec = Struct describing the error.
 * @return TRUE if an error was popped.
 */
boolean ecx_contextt::poperror( ec_errort *Ec)
{
   boolean notEmpty = (elist->head != elist->tail);

   *Ec = elist->Error[elist->tail];
   elist->Error[elist->tail].Signal = FALSE;
   if (notEmpty)
   {
      elist->tail++;
      if (elist->tail > EC_MAXELIST)
      {
         elist->tail = 0;
      }
   }
   else
   {
      *(ecaterror) = FALSE;
   }
   return notEmpty;
}

/** Check if error list has entries.
 *
 * @param[in] context        = context struct
 * @return TRUE if error list contains entries.
 */
boolean ecx_contextt::iserror()
{
   return (elist->head != elist->tail);
}

/** Report packet error
 *
 * @param[in]  context        = context struct
 * @param[in]  Slave      = Slave number
 * @param[in]  Index      = Index that generated error
 * @param[in]  SubIdx     = Subindex that generated error
 * @param[in]  ErrorCode  = Error code
 */
void ecx_contextt::packeterror( uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
   ec_errort Ec;

   memset(&Ec, 0, sizeof(Ec));
   Ec.Time = osal_current_time();
   Ec.Slave = Slave;
   Ec.Index = Index;
   Ec.SubIdx = SubIdx;
   *(ecaterror) = TRUE;
   Ec.Etype = EC_ERR_TYPE_PACKET_ERROR;
   Ec.ErrorCode = ErrorCode;
   pusherror(&Ec);
}

/** Report Mailbox Error
 *
 * @param[in]  context        = context struct
 * @param[in]  Slave        = Slave number
 * @param[in]  Detail       = Following EtherCAT specification
 */
void ecx_contextt::mbxerror( uint16 Slave,uint16 Detail)
{
   ec_errort Ec;

   memset(&Ec, 0, sizeof(Ec));
   Ec.Time = osal_current_time();
   Ec.Slave = Slave;
   Ec.Index = 0;
   Ec.SubIdx = 0;
   Ec.Etype = EC_ERR_TYPE_MBX_ERROR;
   Ec.ErrorCode = Detail;
   pusherror(&Ec);
}

/** Report Mailbox Emergency Error
 *
 * @param[in]  context        = context struct
 * @param[in]  Slave      = Slave number
 * @param[in]  ErrorCode  = Following EtherCAT specification
 * @param[in]  ErrorReg
 * @param[in]  b1
 * @param[in]  w1
 * @param[in]  w2
 */
void ecx_contextt::mbxemergencyerror( uint16 Slave,uint16 ErrorCode,uint16 ErrorReg,
    uint8 b1, uint16 w1, uint16 w2)
{
   ec_errort Ec;

   memset(&Ec, 0, sizeof(Ec));
   Ec.Time = osal_current_time();
   Ec.Slave = Slave;
   Ec.Index = 0;
   Ec.SubIdx = 0;
   Ec.Etype = EC_ERR_TYPE_EMERGENCY;
   Ec.ErrorCode = ErrorCode;
   Ec.ErrorReg = (uint8)ErrorReg;
   Ec.b1 = b1;
   Ec.w1 = w1;
   Ec.w2 = w2;
   pusherror(&Ec);
}

/** Initialise lib in single NIC mode
 * @param[in]  context = context struct
 * @param[in] ifname   = Dev name, f.e. "eth0"
 * @return >0 if OK
 */
int ecx_contextt::init( const char * ifname)
{
	return port->setupnicPrimary(ifname);
}

/** Initialise lib in redundant NIC mode
 * @param[in]  context  = context struct
 * @param[in]  redport  = pointer to redport, redundant port data
 * @param[in]  ifname   = Primary Dev name, f.e. "eth0"
 * @param[in]  if2name  = Secondary Dev name, f.e. "eth1"
 * @return >0 if OK
 */
int ecx_contextt::init_redundant(ecx_redportt *redport, const char *ifname, char *if2name)
{
   return port->init_redundant(redport, ifname, if2name);
}

/** Close lib.
 * @param[in]  context        = context struct
 */
void ecx_contextt::close()
{
   port->closenic();
};

/** Read one byte from slave EEPROM via cache.
 *  If the cache location is empty then a read request is made to the slave.
 *  Depending on the slave capabillities the request is 4 or 8 bytes.
 *  @param[in] context = context struct
 *  @param[in] slave   = slave number
 *  @param[in] address = eeprom address in bytes (slave uses words)
 *  @return requested byte, if not available then 0xff
 */
uint8 ecx_contextt::siigetbyte( uint16 slave, uint16 address)
{
   uint16 configadr, eadr;
   uint64 edat;
   uint16 mapw, mapb;
   int lp,cnt;
   uint8 retval;

   retval = 0xff;
   if (slave != esislave) /* not the same slave? */
   {
      memset(esimap, 0x00, EC_MAXEEPBITMAP * sizeof(uint32)); /* clear esibuf cache map */
      esislave = slave;
   }
   if (address < EC_MAXEEPBUF)
   {
      mapw = address >> 5;
      mapb = address - (mapw << 5);
      if (esimap[mapw] & (uint32)(1 << mapb))
      {
         /* byte is already in buffer */
         retval = esibuf[address];
      }
      else
      {
         /* byte is not in buffer, put it there */
         configadr = slavelist[slave].configadr;
         eeprom2master(slave); /* set eeprom control to master */
         eadr = address >> 1;
         edat = readeepromFP(configadr, eadr, EC_TIMEOUTEEP);
         /* 8 byte response */
         if (slavelist[slave].eep_8byte)
         {
            put_unaligned64(edat, &(esibuf[eadr << 1]));
            cnt = 8;
         }
         /* 4 byte response */
         else
         {
            put_unaligned32(edat, &(esibuf[eadr << 1]));
            cnt = 4;
         }
         /* find bitmap location */
         mapw = eadr >> 4;
         mapb = (eadr << 1) - (mapw << 5);
         for(lp = 0 ; lp < cnt ; lp++)
         {
            /* set bitmap for each byte that is read */
            esimap[mapw] |= (1 << mapb);
            mapb++;
            if (mapb > 31)
            {
               mapb = 0;
               mapw++;
            }
         }
         retval = esibuf[address];
      }
   }

   return retval;
}

/** Find SII section header in slave EEPROM.
 *  @param[in]  context        = context struct
 *  @param[in] slave   = slave number
 *  @param[in] cat     = section category
 *  @return byte address of section at section length entry, if not available then 0
 */
int16 ecx_contextt::siifind( uint16 slave, uint16 cat)
{
   int16 a;
   uint16 p;
   uint8 eectl = slavelist[slave].eep_pdi;

   a = ECT_SII_START << 1;
   /* read first SII section category */
   p = siigetbyte(slave, a++);
   p += (siigetbyte(slave, a++) << 8);
   /* traverse SII while category is not found and not EOF */
   while ((p != cat) && (p != 0xffff))
   {
      /* read section length */
      p = siigetbyte(slave, a++);
      p += (siigetbyte(slave, a++) << 8);
      /* locate next section category */
      a += p << 1;
      /* read section category */
      p = siigetbyte(slave, a++);
      p += (siigetbyte(slave, a++) << 8);
   }
   if (p != cat)
   {
      a = 0;
   }
   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }

   return a;
}

/** Get string from SII string section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[out] str     = requested string, 0x00 if not found
 *  @param[in]  slave   = slave number
 *  @param[in]  Sn      = string number
 */
void ecx_contextt::siistring( char *str, uint16 slave, uint16 Sn)
{
   uint16 a,i,j,l,n,ba;
   char *ptr;
   uint8 eectl = slavelist[slave].eep_pdi;

   ptr = str;
   a = siifind(slave, ECT_SII_STRING); /* find string section */
   if (a > 0)
   {
      ba = a + 2; /* skip SII section header */
      n = siigetbyte(slave, ba++); /* read number of strings in section */
      if (Sn <= n) /* is req string available? */
      {
         for (i = 1; i <= Sn; i++) /* walk through strings */
         {
            l = siigetbyte(slave, ba++); /* length of this string */
            if (i < Sn)
            {
               ba += l;
            }
            else
            {
               ptr = str;
               for (j = 1; j <= l; j++) /* copy one string */
               {
                  if(j <= EC_MAXNAME)
                  {
                     *ptr = (char)siigetbyte(slave, ba++);
                     ptr++;
                  }
                  else
                  {
                     ba++;
                  }
               }
            }
         }
         *ptr = 0; /* add zero terminator */
      }
      else
      {
         ptr = str;
         *ptr = 0; /* empty string */
      }
   }
   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }
}

/** Get FMMU data from SII FMMU section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] FMMU    = FMMU struct from SII, max. 4 FMMU's
 *  @return number of FMMU's defined in section
 */
uint16 ecx_contextt::siiFMMU( uint16 slave, ec_eepromFMMUt* FMMU)
{
   uint16  a;
   uint8 eectl = slavelist[slave].eep_pdi;

   FMMU->nFMMU = 0;
   FMMU->FMMU0 = 0;
   FMMU->FMMU1 = 0;
   FMMU->FMMU2 = 0;
   FMMU->FMMU3 = 0;
   FMMU->Startpos = siifind(slave, ECT_SII_FMMU);

   if (FMMU->Startpos > 0)
   {
      a = FMMU->Startpos;
      FMMU->nFMMU = siigetbyte(slave, a++);
      FMMU->nFMMU += (siigetbyte(slave, a++) << 8);
      FMMU->nFMMU *= 2;
      FMMU->FMMU0 = siigetbyte(slave, a++);
      FMMU->FMMU1 = siigetbyte(slave, a++);
      if (FMMU->nFMMU > 2)
      {
         FMMU->FMMU2 = siigetbyte(slave, a++);
         FMMU->FMMU3 = siigetbyte(slave, a++);
      }
   }
   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }

   return FMMU->nFMMU;
}

/** Get SM data from SII SM section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] SM      = first SM struct from SII
 *  @return number of SM's defined in section
 */
uint16 ecx_contextt::siiSM( uint16 slave, ec_eepromSMt* SM)
{
   uint16 a,w;
   uint8 eectl = slavelist[slave].eep_pdi;

   SM->nSM = 0;
   SM->Startpos = siifind(slave, ECT_SII_SM);
   if (SM->Startpos > 0)
   {
      a = SM->Startpos;
      w = siigetbyte(slave, a++);
      w += (siigetbyte(slave, a++) << 8);
      SM->nSM = (w / 4);
      SM->PhStart = siigetbyte(slave, a++);
      SM->PhStart += (siigetbyte(slave, a++) << 8);
      SM->Plength = siigetbyte(slave, a++);
      SM->Plength += (siigetbyte(slave, a++) << 8);
      SM->Creg = siigetbyte(slave, a++);
      SM->Sreg = siigetbyte(slave, a++);
      SM->Activate = siigetbyte(slave, a++);
      SM->PDIctrl = siigetbyte(slave, a++);
   }
   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }

   return SM->nSM;
}

/** Get next SM data from SII SM section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] SM      = first SM struct from SII
 *  @param[in]  n       = SM number
 *  @return >0 if OK
 */
uint16 ecx_contextt::siiSMnext( uint16 slave, ec_eepromSMt* SM, uint16 n)
{
   uint16 a;
   uint16 retVal = 0;
   uint8 eectl = slavelist[slave].eep_pdi;

   if (n < SM->nSM)
   {
      a = SM->Startpos + 2 + (n * 8);
      SM->PhStart = siigetbyte(slave, a++);
      SM->PhStart += (siigetbyte(slave, a++) << 8);
      SM->Plength = siigetbyte(slave, a++);
      SM->Plength += (siigetbyte(slave, a++) << 8);
      SM->Creg = siigetbyte(slave, a++);
      SM->Sreg = siigetbyte(slave, a++);
      SM->Activate = siigetbyte(slave, a++);
      SM->PDIctrl = siigetbyte(slave, a++);
      retVal = 1;
   }
   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }

   return retVal;
}

/** Get PDO data from SII PDO section in slave EEPROM.
 *  @param[in]  context = context struct
 *  @param[in]  slave   = slave number
 *  @param[out] PDO     = PDO struct from SII
 *  @param[in]  t       = 0=RXPDO 1=TXPDO
 *  @return mapping size in bits of PDO
 */
int ecx_contextt::siiPDO( uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
   uint16 a , w, c, e, er, Size;
   uint8 eectl = slavelist[slave].eep_pdi;

   Size = 0;
   PDO->nPDO = 0;
   PDO->Length = 0;
   PDO->Index[1] = 0;
   for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
   if (t > 1)
      t = 1;
   PDO->Startpos = siifind(slave, ECT_SII_PDO + t);
   if (PDO->Startpos > 0)
   {
      a = PDO->Startpos;
      w = siigetbyte(slave, a++);
      w += (siigetbyte(slave, a++) << 8);
      PDO->Length = w;
      c = 1;
      /* traverse through all PDOs */
      do
      {
         PDO->nPDO++;
         PDO->Index[PDO->nPDO] = siigetbyte(slave, a++);
         PDO->Index[PDO->nPDO] += (siigetbyte(slave, a++) << 8);
         PDO->BitSize[PDO->nPDO] = 0;
         c++;
         e = siigetbyte(slave, a++);
         PDO->SyncM[PDO->nPDO] = siigetbyte(slave, a++);
         a += 4;
         c += 2;
         if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
         {
            /* read all entries defined in PDO */
            for (er = 1; er <= e; er++)
            {
               c += 4;
               a += 5;
               PDO->BitSize[PDO->nPDO] += siigetbyte(slave, a++);
               a += 2;
            }
            PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
            Size += PDO->BitSize[PDO->nPDO];
            c++;
         }
         else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
         {
            c += 4 * e;
            a += 8 * e;
            c++;
         }
         if (PDO->nPDO >= (EC_MAXEEPDO - 1))
         {
            c = PDO->Length; /* limit number of PDO entries in buffer */
         }
      }
      while (c < PDO->Length);
   }
   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }

   return (Size);
}


int ecx_contextt::FPRD_multi(int n, uint16 *configlst, ec_alstatust *slstatlst, int timeout)
{
	return port->FPRD_multi(n, configlst, slstatlst, timeout);
}

#define MAX_FPRD_MULTI 64
/** Read all slave states in ec_slave.
 * @param[in] context = context struct
 * @return lowest state found
 */
int ecx_contextt::readstate()
{
   uint16 lslave;
   ec_alstatust sl[MAX_FPRD_MULTI];
   uint16 slca[MAX_FPRD_MULTI];

   uint16 lowest = 0xff;
   slavelist[0].ALstatuscode = 0;
   uint16 fslave = 1;
   do
   {
      lslave = *(slavecount);
      if ((lslave - fslave) >= MAX_FPRD_MULTI)
      {
         lslave = fslave + MAX_FPRD_MULTI - 1;
      }
	  for (uint16 slave = fslave; slave <= lslave; slave++)
      {
         const ec_alstatust zero = {0, 0, 0};

		 uint16 configadr = slavelist[slave].configadr;
         slca[slave - fslave] = configadr;
         sl[slave - fslave] = zero;
      }
      FPRD_multi((lslave - fslave) + 1, &(slca[0]), &(sl[0]), EC_TIMEOUTRET3);
	  for (uint16 slave = fslave; slave <= lslave; slave++)
      {
		  uint16 configadr = slavelist[slave].configadr;
		  uint16 rval = etohs(sl[slave - fslave].alstatus);
		  slavelist[slave].ALstatuscode = etohs(sl[slave - fslave].alstatuscode);
		  lowest = min(lowest, (rval & 0xf));
		  slavelist[slave].state = rval;
		  slavelist[0].ALstatuscode |= slavelist[slave].ALstatuscode;
      }
      fslave = lslave + 1;
   } while(lslave < *(slavecount));
   slavelist[0].state = lowest;

   return lowest;
}

/** Write slave state, if slave = 0 then write to all slaves.
 * The function does not check if the actual state is changed.
 * @param[in]  context        = context struct
 * @param[in] slave    = Slave number, 0 = master
 * @return Workcounter or EC_NOFRAME
 */
int ecx_contextt::writestate( uint16 slave)
{
   int ret;
   uint16 configadr, slstate;

   if (slave == 0)
   {
      slstate = htoes(slavelist[slave].state);
      ret = port->BWR(0, ECT_REG_ALCTL, sizeof(slstate),
	            &slstate, EC_TIMEOUTRET3);
   }
   else
   {
      configadr = slavelist[slave].configadr;

      ret = port->FPWRw(configadr, ECT_REG_ALCTL,
	        htoes(slavelist[slave].state), EC_TIMEOUTRET3);
   }
   return ret;
}

/** Check actual slave state.
 * This is a blocking function.
 * @param[in] context     = context struct
 * @param[in] slave       = Slave number, 0 = all slaves
 * @param[in] reqstate    = Requested state
 * @param[in] timeout     = Timout value in us
 * @return Requested state, or found state after timeout.
 */
uint16 ecx_contextt::statecheck( uint16 slave, uint16 reqstate, int timeout)
{
   uint16 configadr, state, rval;
   ec_alstatust slstat;
   osal_timert timer;

   if ( slave > *(slavecount) )
   {
      return 0;
   }
   osal_timer_start(&timer, timeout);
   configadr = slavelist[slave].configadr;
   do
   {
      if (slave < 1)
      {
         rval = 0;
         port->BRD(0, ECT_REG_ALSTAT, sizeof(rval), &rval , EC_TIMEOUTRET);
         rval = etohs(rval);
      }
      else
      {
         slstat.alstatus = 0;
         slstat.alstatuscode = 0;
         port->FPRD(configadr, ECT_REG_ALSTAT, sizeof(slstat), &slstat, EC_TIMEOUTRET);
         rval = etohs(slstat.alstatus);
         slavelist[slave].ALstatuscode = etohs(slstat.alstatuscode);
      }
      state = rval & 0x000f; /* read slave status */
      if (state != reqstate)
      {
         osal_usleep(1000);
      }
   }
   while ((state != reqstate) && (osal_timer_is_expired(&timer) == FALSE));
   slavelist[slave].state = rval;

   return state;
}

/** Get index of next mailbox counter value.
 * Used for Mailbox Link Layer.
 * @param[in] cnt     = Mailbox counter value [0..7]
 * @return next mailbox counter value
 */
uint8 ec_nextmbxcnt(uint8 cnt)
{
   cnt++;
   if (cnt > 7)
   {
      cnt = 1; /* wrap around to 1, not 0 */
   }

   return cnt;
}

/** Clear mailbox buffer.
 * @param[out] Mbx     = Mailbox buffer to clear
 */
void ec_clearmbx(ec_mbxbuft *Mbx)
{
    memset(Mbx, 0x00, EC_MAXMBX);
}

/** Check if IN mailbox of slave is empty.
 * @param[in] context  = context struct
 * @param[in] slave    = Slave number
 * @param[in] timeout  = Timeout in us
 * @return >0 is success
 */
int ecx_contextt::mbxempty( uint16 slave, int timeout)
{
   uint16 configadr;
   uint8 SMstat;
   int wkc;
   osal_timert timer;

   osal_timer_start(&timer, timeout);
   configadr = slavelist[slave].configadr;
   do
   {
      SMstat = 0;
      wkc = port->FPRD(configadr, ECT_REG_SM0STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
      SMstat = etohs(SMstat);
      if (((SMstat & 0x08) != 0) && (timeout > EC_LOCALDELAY))
      {
         osal_usleep(EC_LOCALDELAY);
      }
   }
   while (((wkc <= 0) || ((SMstat & 0x08) != 0)) && (osal_timer_is_expired(&timer) == FALSE));

   if ((wkc > 0) && ((SMstat & 0x08) == 0))
   {
      return 1;
   }

   return 0;
}

/** Write IN mailbox to slave.
 * @param[in]  context    = context struct
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 */
int ecx_contextt::mbxsend( uint16 slave,ec_mbxbuft *mbx, int timeout)
{
   uint16 mbxwo,mbxl,configadr;
   int wkc;

   wkc = 0;
   configadr = slavelist[slave].configadr;
   mbxl = slavelist[slave].mbx_l;
   if ((mbxl > 0) && (mbxl <= EC_MAXMBX))
   {
      if (mbxempty(slave, timeout))
      {
         mbxwo = slavelist[slave].mbx_wo;
         /* write slave in mailbox */
         wkc = port->FPWR(configadr, mbxwo, mbxl, mbx, EC_TIMEOUTRET3);
      }
      else
      {
         wkc = 0;
      }
   }

   return wkc;
}

/** Read OUT mailbox from slave.
 * Supports Mailbox Link Layer with repeat requests.
 * @param[in]  context    = context struct
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 */
int ecx_contextt::mbxreceive( uint16 slave, ec_mbxbuft *mbx, int timeout)
{
   uint16 mbxro,mbxl,configadr;
   int wkc=0;
   int wkc2;
   uint16 SMstat;
   uint8 SMcontr;
   ec_mbxheadert *mbxh;
   ec_emcyt *EMp;
   ec_mbxerrort *MBXEp;

   configadr = slavelist[slave].configadr;
   mbxl = slavelist[slave].mbx_rl;
   if ((mbxl > 0) && (mbxl <= EC_MAXMBX))
   {
      osal_timert timer;

      osal_timer_start(&timer, timeout);
      wkc = 0;
      do /* wait for read mailbox available */
      {
         SMstat = 0;
         wkc = port->FPRD(configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
         SMstat = etohs(SMstat);
         if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
         {
            osal_usleep(EC_LOCALDELAY);
         }
      }
      while (((wkc <= 0) || ((SMstat & 0x08) == 0)) && (osal_timer_is_expired(&timer) == FALSE));

      if ((wkc > 0) && ((SMstat & 0x08) > 0)) /* read mailbox available ? */
      {
         mbxro = slavelist[slave].mbx_ro;
         mbxh = (ec_mbxheadert *)mbx;
         do
         {
            wkc = port->FPRD(configadr, mbxro, mbxl, mbx, EC_TIMEOUTRET); /* get mailbox */
            if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x00)) /* Mailbox error response? */
            {
               MBXEp = (ec_mbxerrort *)mbx;
               mbxerror(slave, etohs(MBXEp->Detail));
               wkc = 0; /* prevent emergency to cascade up, it is already handled. */
            }
            else if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x03)) /* CoE response? */
            {
               EMp = (ec_emcyt *)mbx;
               if ((etohs(EMp->CANOpen) >> 12) == 0x01) /* Emergency request? */
               {
                  mbxemergencyerror(slave, etohs(EMp->ErrorCode), EMp->ErrorReg,
                          EMp->bData, etohs(EMp->w1), etohs(EMp->w2));
                  wkc = 0; /* prevent emergency to cascade up, it is already handled. */
               }
            }
            else
            {
               if (wkc <= 0) /* read mailbox lost */
               {
                  SMstat ^= 0x0200; /* toggle repeat request */
                  SMstat = htoes(SMstat);
                  wkc2 = port->FPWR(configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
                  SMstat = etohs(SMstat);
                  do /* wait for toggle ack */
                  {
                     wkc2 = port->FPRD(configadr, ECT_REG_SM1CONTR, sizeof(SMcontr), &SMcontr, EC_TIMEOUTRET);
                   } while (((wkc2 <= 0) || ((SMcontr & 0x02) != (HI_BYTE(SMstat) & 0x02))) && (osal_timer_is_expired(&timer) == FALSE));
                  do /* wait for read mailbox available */
                  {
                     wkc2 = port->FPRD(configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
                     SMstat = etohs(SMstat);
                     if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
                     {
                        osal_usleep(EC_LOCALDELAY);
                     }
                  } while (((wkc2 <= 0) || ((SMstat & 0x08) == 0)) && (osal_timer_is_expired(&timer) == FALSE));
               }
            }
         } while ((wkc <= 0) && (osal_timer_is_expired(&timer) == FALSE)); /* if WKC<=0 repeat */
      }
      else /* no read mailbox available */
      {
          wkc = 0;
      }
   }

   return wkc;
}

/** Dump complete EEPROM data from slave in buffer.
 * @param[in]  context  = context struct
 * @param[in]  slave    = Slave number
 * @param[out] esibuf   = EEPROM data buffer, make sure it is big enough.
 */
void ecx_contextt::esidump( uint16 slave, uint8 *esibuf)
{
   int address, incr;
   uint16 configadr;
   uint64 *p64;
   uint16 *p16;
   uint64 edat;
   uint8 eectl = slavelist[slave].eep_pdi;

   eeprom2master(slave); /* set eeprom control to master */
   configadr = slavelist[slave].configadr;
   address = ECT_SII_START;
   p16=(uint16*)esibuf;
   if (slavelist[slave].eep_8byte)
   {
      incr = 4;
   }
   else
   {
      incr = 2;
   }
   do
   {
      edat = readeepromFP(configadr, address, EC_TIMEOUTEEP);
      p64 = (uint64*)p16;
      *p64 = edat;
      p16 += incr;
      address += incr;
   } while ((address <= (EC_MAXEEPBUF >> 1)) && ((uint32)edat != 0xffffffff));

   if (eectl)
   {
      eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
   }
}

/** Read EEPROM from slave bypassing cache.
 * @param[in] context   = context struct
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout   = Timeout in us.
 * @return EEPROM data 32bit
 */
uint32 ecx_contextt::readeeprom( uint16 slave, uint16 eeproma, int timeout)
{
   uint16 configadr;

   eeprom2master(slave); /* set eeprom control to master */
   configadr = slavelist[slave].configadr;

   return ((uint32)readeepromFP(configadr, eeproma, timeout));
}

/** Write EEPROM to slave bypassing cache.
 * @param[in] context   = context struct
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 */
int ecx_contextt::writeeeprom( uint16 slave, uint16 eeproma, uint16 data, int timeout)
{
   uint16 configadr;

   eeprom2master(slave); /* set eeprom control to master */
   configadr = slavelist[slave].configadr;
   return (writeeepromFP(configadr, eeproma, data, timeout));
}

/** Set eeprom control to master. Only if set to PDI.
 * @param[in] context   = context struct
 * @param[in] slave     = Slave number
 * @return >0 if OK
 */
int ecx_contextt::eeprom2master( uint16 slave)
{
   int wkc = 1, cnt = 0;
   uint16 configadr;
   uint8 eepctl;

   if ( slavelist[slave].eep_pdi )
   {
      configadr = slavelist[slave].configadr;
      eepctl = 2;
      do
      {
         wkc = port->FPWR(configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
      eepctl = 0;
      cnt = 0;
      do
      {
         wkc = port->FPWR(configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
      slavelist[slave].eep_pdi = 0;
   }

   return wkc;
}

/** Set eeprom control to PDI. Only if set to master.
 * @param[in]  context        = context struct
 * @param[in] slave     = Slave number
 * @return >0 if OK
 */
int ecx_contextt::eeprom2pdi( uint16 slave)
{
   int wkc = 1, cnt = 0;
   uint16 configadr;
   uint8 eepctl;

   if ( !slavelist[slave].eep_pdi )
   {
      configadr = slavelist[slave].configadr;
      eepctl = 1;
      do
      {
         wkc = port->FPWR(configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to PDI */
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
      slavelist[slave].eep_pdi = 1;
   }

   return wkc;
}

uint16 ecx_contextt::eeprom_waitnotbusyAP( uint16 aiadr,uint16 *estat, int timeout)
{
   int wkc, cnt = 0, retval = 0;
   osal_timert timer;

   osal_timer_start(&timer, timeout);
   do
   {
      if (cnt++)
      {
         osal_usleep(EC_LOCALDELAY);
      }
      *estat = 0;
      wkc=port->APRD(aiadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
      *estat = etohs(*estat);
   }
   while (((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (osal_timer_is_expired(&timer) == FALSE)); /* wait for eeprom ready */
   if ((*estat & EC_ESTAT_BUSY) == 0)
   {
      retval = 1;
   }

   return retval;
}

/** Read EEPROM from slave bypassing cache. APRD method.
 * @param[in] context     = context struct
 * @param[in] aiadr       = auto increment address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ecx_contextt::readeepromAP( uint16 aiadr, uint16 eeproma, int timeout)
{
   uint16 estat;
   uint32 edat32;
   uint64 edat64;
   ec_eepromt ed;
   int wkc, cnt, nackcnt = 0;

   edat64 = 0;
   edat32 = 0;
   if (eeprom_waitnotbusyAP(aiadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc = port->APWR(aiadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }

      do
      {
         ed.comm = htoes(EC_ECMD_READ);
         ed.addr = htoes(eeproma);
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc = port->APWR(aiadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY);
            estat = 0x0000;
            if (eeprom_waitnotbusyAP(aiadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  if (estat & EC_ESTAT_R64)
                  {
                     cnt = 0;
                     do
                     {
                        wkc = port->APRD(aiadr, ECT_REG_EEPDAT, sizeof(edat64), &edat64, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                  }
                  else
                  {
                     cnt = 0;
                     do
                     {
                        wkc = port->APRD(aiadr, ECT_REG_EEPDAT, sizeof(edat32), &edat32, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                     edat64=(uint64)edat32;
                  }
               }
            }
         }
      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return edat64;
}

/** Write EEPROM to slave bypassing cache. APWR method.
 * @param[in] context   = context struct
 * @param[in] aiadr     = configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 */
int ecx_contextt::writeeepromAP( uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
{
   uint16 estat;
   ec_eepromt ed;
   int wkc, rval = 0, cnt = 0, nackcnt = 0;

   if (eeprom_waitnotbusyAP(aiadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc = port->APWR(aiadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }
      do
      {
         cnt = 0;
         do
         {
            wkc = port->APWR(aiadr, ECT_REG_EEPDAT, sizeof(data), &data, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));

         ed.comm = EC_ECMD_WRITE;
         ed.addr = eeproma;
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc = port->APWR(aiadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY * 2);
            estat = 0x0000;
            if (eeprom_waitnotbusyAP(aiadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  rval = 1;
               }
            }
         }

      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return rval;
}

uint16 ecx_contextt::eeprom_waitnotbusyFP( uint16 configadr,uint16 *estat, int timeout)
{
   int wkc, cnt = 0, retval = 0;
   osal_timert timer;

   osal_timer_start(&timer, timeout);
   do
   {
      if (cnt++)
      {
         osal_usleep(EC_LOCALDELAY);
      }
      *estat = 0;
      wkc=port->FPRD(configadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
      *estat = etohs(*estat);
   }
   while (((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (osal_timer_is_expired(&timer) == FALSE)); /* wait for eeprom ready */
   if ((*estat & EC_ESTAT_BUSY) == 0)
   {
      retval = 1;
   }

   return retval;
}

/** Read EEPROM from slave bypassing cache. FPRD method.
 * @param[in] context     = context struct
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ecx_contextt::readeepromFP( uint16 configadr, uint16 eeproma, int timeout)
{
   uint16 estat;
   uint32 edat32;
   uint64 edat64;
   ec_eepromt ed;
   int wkc, cnt, nackcnt = 0;

   edat64 = 0;
   edat32 = 0;
   if (eeprom_waitnotbusyFP(configadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc=port->FPWR(configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }

      do
      {
         ed.comm = htoes(EC_ECMD_READ);
         ed.addr = htoes(eeproma);
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc=port->FPWR(configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY);
            estat = 0x0000;
            if (eeprom_waitnotbusyFP(configadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  if (estat & EC_ESTAT_R64)
                  {
                     cnt = 0;
                     do
                     {
                        wkc=port->FPRD(configadr, ECT_REG_EEPDAT, sizeof(edat64), &edat64, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                  }
                  else
                  {
                     cnt = 0;
                     do
                     {
                        wkc=port->FPRD(configadr, ECT_REG_EEPDAT, sizeof(edat32), &edat32, EC_TIMEOUTRET);
                     }
                     while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
                     edat64=(uint64)edat32;
                  }
               }
            }
         }
      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return edat64;
}

/** Write EEPROM to slave bypassing cache. FPWR method.
 * @param[in]  context        = context struct
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] data        = 16bit data
 * @param[in] timeout     = Timeout in us.
 * @return >0 if OK
 */
int ecx_contextt::writeeepromFP( uint16 configadr, uint16 eeproma, uint16 data, int timeout)
{
   uint16 estat;
   ec_eepromt ed;
   int wkc, rval = 0, cnt = 0, nackcnt = 0;

   if (eeprom_waitnotbusyFP(configadr, &estat, timeout))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc = port->FPWR(configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }
      do
      {
         cnt = 0;
         do
         {
            wkc = port->FPWR(configadr, ECT_REG_EEPDAT, sizeof(data), &data, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         ed.comm = EC_ECMD_WRITE;
         ed.addr = eeproma;
         ed.d2   = 0x0000;
         cnt = 0;
         do
         {
            wkc = port->FPWR(configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
         }
         while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
         if (wkc)
         {
            osal_usleep(EC_LOCALDELAY * 2);
            estat = 0x0000;
            if (eeprom_waitnotbusyFP(configadr, &estat, timeout))
            {
               if (estat & EC_ESTAT_NACK)
               {
                  nackcnt++;
                  osal_usleep(EC_LOCALDELAY * 5);
               }
               else
               {
                  nackcnt = 0;
                  rval = 1;
               }
            }
         }
      }
      while ((nackcnt > 0) && (nackcnt < 3));
   }

   return rval;
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 1, make request to slave.
 * @param[in] context     = context struct
 * @param[in] slave       = Slave number
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 */
void ecx_contextt::readeeprom1( uint16 slave, uint16 eeproma)
{
   uint16 configadr, estat;
   ec_eepromt ed;
   int wkc, cnt = 0;

   eeprom2master(slave); /* set eeprom control to master */
   configadr = slavelist[slave].configadr;
   if (eeprom_waitnotbusyFP(configadr, &estat, EC_TIMEOUTEEP))
   {
      if (estat & EC_ESTAT_EMASK) /* error bits are set */
      {
         estat = htoes(EC_ECMD_NOP); /* clear error bits */
         wkc = port->FPWR(configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET3);
      }
      ed.comm = htoes(EC_ECMD_READ);
      ed.addr = htoes(eeproma);
      ed.d2   = 0x0000;
      do
      {
         wkc = port->FPWR(configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
   }
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 2, actual read from slave.
 * @param[in]  context        = context struct
 * @param[in] slave       = Slave number
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 32bit
 */
uint32 ecx_contextt::readeeprom2( uint16 slave, int timeout)
{
   uint16 estat, configadr;
   uint32 edat;
   int wkc, cnt = 0;

   configadr = slavelist[slave].configadr;
   edat = 0;
   estat = 0x0000;
   if (eeprom_waitnotbusyFP(configadr, &estat, timeout))
   {
      do
      {
          wkc = port->FPRD(configadr, ECT_REG_EEPDAT, sizeof(edat), &edat, EC_TIMEOUTRET);
      }
      while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
   }

   return edat;
}

/** Push index of segmented LRD/LWR/LRW combination.
 * @param[in]  context        = context struct
 * @param[in] idx         = Used datagram index.
 * @param[in] data        = Pointer to process data segment.
 * @param[in] length      = Length of data segment in bytes.
 */
void ecx_contextt::pushindex( uint8 idx, void *data, uint16 length)
{
   if(idxstack->pushed < EC_MAXBUF)
   {
      idxstack->idx[idxstack->pushed] = idx;
      idxstack->data[idxstack->pushed] = data;
      idxstack->length[idxstack->pushed] = length;
      idxstack->pushed++;
   }
}

/** Pull index of segmented LRD/LWR/LRW combination.
 * @param[in]  context        = context struct
 * @return Stack location, -1 if stack is empty.
 */
int ecx_contextt::pullindex()
{
   int rval = -1;
   if(idxstack->pulled < idxstack->pushed)
   {
      rval = idxstack->pulled;
      idxstack->pulled++;
   }

   return rval;
}

/** Transmit processdata to slaves.
 * Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
 * Both the input and output processdata are transmitted.
 * The outputs with the actual data, the inputs have a placeholder.
 * The inputs are gathered with the receive processdata function.
 * In contrast to the base LRW function this function is non-blocking.
 * If the processdata does not fit in one datagram, multiple are used.
 * In order to recombine the slave response, a stack is used.
 * @param[in]  context        = context struct
 * @param[in]  group          = group number
 * @return >0 if processdata is transmitted.
 */
int ecx_contextt::send_processdata_group( uint8 group)
{
   uint32 LogAdr;
   uint16 w1, w2;
   int length, sublength;
   uint8 idx;
   int wkc;
   uint8* data;
   boolean first=FALSE;
   uint16 currentsegment = 0;

   wkc = 0;
   if(grouplist[group].hasdc)
   {
      first = TRUE;
   }
   length = grouplist[group].Obytes + grouplist[group].Ibytes;
   LogAdr = grouplist[group].logstartaddr;
   if (length)
   {
      if(!group)
      {
         idxstack->pushed = 0;
         idxstack->pulled = 0;
      }
      wkc = 1;
      /* LRW blocked by one or more slaves ? */
      if (grouplist[group].blockLRW)
      {
         /* if inputs available generate LRD */
         if(grouplist[group].Ibytes)
         {
            currentsegment = grouplist[group].Isegment;
            data = grouplist[group].inputs;
            length = grouplist[group].Ibytes;
            LogAdr += grouplist[group].Obytes;
            /* segment transfer if needed */
            do
            {
               if(currentsegment == grouplist[group].Isegment)
               {
                  sublength = grouplist[group].IOsegment[currentsegment++] - grouplist[group].Ioffset;
               }
               else
               {
                  sublength = grouplist[group].IOsegment[currentsegment++];
               }
               /* get new index */
               idx = port->getindex();
               w1 = LO_WORD(LogAdr);
               w2 = HI_WORD(LogAdr);
               port->setupdatagram(EC_CMD_LRD, idx, w1, w2, sublength, data);
               if(first)
               {
                  DCl = sublength;
                  /* FPRMW in second datagram */
                  DCtO = port->adddatagram(EC_CMD_FRMW, idx, FALSE,
                                           slavelist[grouplist[group].DCnext].configadr,
                                           ECT_REG_DCSYSTIME, sizeof(int64), DCtime);
                  first = FALSE;
               }
               /* send frame */
               port->outframe_red(idx);
               /* push index and data pointer on stack */
               pushindex(idx, data, sublength);
               length -= sublength;
               LogAdr += sublength;
               data += sublength;
            } while (length && (currentsegment < grouplist[group].nsegments));
         }
         /* if outputs available generate LWR */
         if(grouplist[group].Obytes)
         {
            data = grouplist[group].outputs;
            length = grouplist[group].Obytes;
            LogAdr = grouplist[group].logstartaddr;
            currentsegment = 0;
            /* segment transfer if needed */
            do
            {
               sublength = grouplist[group].IOsegment[currentsegment++];
               if((length - sublength) < 0)
               {
                  sublength = length;
               }
               /* get new index */
               idx = port->getindex();
               w1 = LO_WORD(LogAdr);
               w2 = HI_WORD(LogAdr);
               port->setupdatagram(EC_CMD_LWR, idx, w1, w2, sublength, data);
               if(first)
               {
                  DCl = sublength;
                  /* FPRMW in second datagram */
                  DCtO = port->adddatagram(EC_CMD_FRMW, idx, FALSE,
                                           slavelist[grouplist[group].DCnext].configadr,
                                           ECT_REG_DCSYSTIME, sizeof(int64), DCtime);
                  first = FALSE;
               }
               /* send frame */
               port->outframe_red(idx);
               /* push index and data pointer on stack */
               pushindex(idx, data, sublength);
               length -= sublength;
               LogAdr += sublength;
               data += sublength;
            } while (length && (currentsegment < grouplist[group].nsegments));
         }
      }
      /* LRW can be used */
      else
      {
         if (grouplist[group].Obytes)
         {
            data = grouplist[group].outputs;
         }
         else
         {
            data = grouplist[group].inputs;
         }
         /* segment transfer if needed */
         do
         {
            sublength = grouplist[group].IOsegment[currentsegment++];
            /* get new index */
            idx = port->getindex();
            w1 = LO_WORD(LogAdr);
            w2 = HI_WORD(LogAdr);
            port->setupdatagram(EC_CMD_LRW, idx, w1, w2, sublength, data);
            if(first)
            {
               DCl = sublength;
               /* FPRMW in second datagram */
               DCtO = port->adddatagram(EC_CMD_FRMW, idx, FALSE,
                                        slavelist[grouplist[group].DCnext].configadr,
                                        ECT_REG_DCSYSTIME, sizeof(int64), DCtime);
               first = FALSE;
            }
            /* send frame */
            port->outframe_red(idx);
            /* push index and data pointer on stack */
            pushindex(idx, data, sublength);
            length -= sublength;
            LogAdr += sublength;
            data += sublength;
         } while (length && (currentsegment < grouplist[group].nsegments));
      }
   }

   return wkc;
}

/** Receive processdata from slaves.
 * Second part from ec_send_processdata().
 * Received datagrams are recombined with the processdata with help from the stack.
 * If a datagram contains input processdata it copies it to the processdata structure.
 * @param[in]  context        = context struct
 * @param[in]  group          = group number
 * @param[in]  timeout        = Timeout in us.
 * @return Work counter.
 */
int ecx_contextt::receive_processdata_group( uint8 group, int timeout)
{
   int wkc = 0;
   int valid_wkc = 0;
   boolean first = FALSE;

   if(grouplist[group].hasdc)
   {
      first = TRUE;
   }
   /* get first index */
   int pos = pullindex();
   /* read the same number of frames as send */
   while (pos >= 0)
   {
      int idx = idxstack->idx[pos];
	  int wkc2 = port->waitinframe(idxstack->idx[pos], timeout);
      /* check if there is input data in frame */
      if (wkc2 > EC_NOFRAME)
      {
		  port->receive_processdata(idx, DCtO, DCl, idxstack->length[pos], wkc2, idxstack->data[pos], DCtime, &wkc, &valid_wkc, &first);
      }
      /* release buffer */
      port->setbufstat(idx, EC_BUF_EMPTY);
      /* get next index */
      pos = pullindex();
   }
   /* if no frames has arrived */
   if (valid_wkc == 0)
   {
      return EC_NOFRAME;
   }
   return wkc;
}


int ecx_contextt::send_processdata()
{
   return send_processdata_group(0);
}

int ecx_contextt::receive_processdata( int timeout)
{
   return receive_processdata_group(0, timeout);
}
/** Report SDO error.
*
* @param[in]  context    = context struct
* @param[in]  Slave      = Slave number
* @param[in]  Index      = Index that generated error
* @param[in]  SubIdx     = Subindex that generated error
* @param[in]  AbortCode  = Abortcode, see EtherCAT documentation for list
*/
void ecx_contextt::SDOerror(uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode)
{
	ec_errort Ec;

	memset(&Ec, 0, sizeof(Ec));
	Ec.Time = osal_current_time();
	Ec.Slave = Slave;
	Ec.Index = Index;
	Ec.SubIdx = SubIdx;
	*(ecaterror) = TRUE;
	Ec.Etype = EC_ERR_TYPE_SDO_ERROR;
	Ec.AbortCode = AbortCode;
	pusherror(&Ec);
}



/** Report SDO info error
*
* @param[in]  context    = context struct
* @param[in]  Slave      = Slave number
* @param[in]  Index      = Index that generated error
* @param[in]  SubIdx     = Subindex that generated error
* @param[in]  AbortCode  = Abortcode, see EtherCAT documentation for list
*/
void ecx_contextt::SDOinfoerror(uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode)
{
	ec_errort Ec;

	memset(&Ec, 0, sizeof(Ec));
	Ec.Slave = Slave;
	Ec.Index = Index;
	Ec.SubIdx = SubIdx;
	*(ecaterror) = TRUE;
	Ec.Etype = EC_ERR_TYPE_SDOINFO_ERROR;
	Ec.AbortCode = AbortCode;
	pusherror(&Ec);
}

/** SDO structure, not to be confused with EcSDOserviceT */
PACKED_BEGIN
typedef struct PACKED
{
	ec_mbxheadert   MbxHeader;
	uint16          CANOpen;
	uint8           Command;
	uint16          Index;
	uint8           SubIndex;
	union
	{
		uint8   bdata[0x200]; /* variants for easy data access */
		uint16  wdata[0x100];
		uint32  ldata[0x80];
	};
} ec_SDOt;
PACKED_END

/** CoE SDO read, blocking. Single subindex or Complete Access.
*
* Only a "normal" upload request is issued. If the requested parameter is <= 4bytes
* then a "expedited" response is returned, otherwise a "normal" response. If a "normal"
* response is larger than the mailbox size then the response is segmented. The function
* will combine all segments and copy them to the parameter buffer.
*
* @param[in]  context    = context struct
* @param[in]  slave      = Slave number
* @param[in]  index      = Index to read
* @param[in]  subindex   = Subindex to read, must be 0 or 1 if CA is used.
* @param[in]  CA         = FALSE = single subindex. TRUE = Complete Access, all subindexes read.
* @param[in,out] psize   = Size in bytes of parameter buffer, returns bytes read from SDO.
* @param[out] p          = Pointer to parameter buffer
* @param[in]  timeout    = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::SDOread(uint16 slave, uint16 index, uint8 subindex,
	boolean CA, int *psize, void *p, int timeout)
{
	ec_SDOt *SDOp, *aSDOp;
	uint16 bytesize, Framedatasize;
	int wkc;
	int32 SDOlen;
	uint8 *bp;
	uint8 *hp;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt, toggle;
	boolean NotLast;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
	wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSDOp = (ec_SDOt *)&MbxIn;
	SDOp = (ec_SDOt *)&MbxOut;
	SDOp->MbxHeader.length = htoes(0x000a);
	SDOp->MbxHeader.address = htoes(0x0000);
	SDOp->MbxHeader.priority = 0x00;
	/* get new mailbox count value, used as session handle */
	cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
	slavelist[slave].mbx_cnt = cnt;
	SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
	SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* number 9bits service upper 4 bits (SDO request) */
	if (CA)
	{
		SDOp->Command = ECT_SDO_UP_REQ_CA; /* upload request complete access */
	}
	else
	{
		SDOp->Command = ECT_SDO_UP_REQ; /* upload request normal */
	}
	SDOp->Index = htoes(index);
	if (CA && (subindex > 1))
	{
		subindex = 1;
	}
	SDOp->SubIndex = subindex;
	SDOp->ldata[0] = 0;
	/* send CoE request to slave */
	wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
	if (wkc > 0) /* succeeded to place mailbox in slave ? */
	{
		/* clean mailboxbuffer */
		ec_clearmbx(&MbxIn);
		/* read slave response */
		wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
		if (wkc > 0) /* succeeded to read slave response ? */
		{
			/* slave response should be CoE, SDO response and the correct index */
			if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
				((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
				(aSDOp->Index == SDOp->Index))
			{
				if ((aSDOp->Command & 0x02) > 0)
				{
					/* expedited frame response */
					bytesize = 4 - ((aSDOp->Command >> 2) & 0x03);
					if (*psize >= bytesize) /* parameter buffer big enough ? */
					{
						/* copy parameter in parameter buffer */
						memcpy(p, &aSDOp->ldata[0], bytesize);
						/* return the real parameter size */
						*psize = bytesize;
					}
					else
					{
						wkc = 0;
						packeterror(slave, index, subindex, 3); /*  data container too small for type */
					}
				}
				else
				{ /* normal frame response */
					SDOlen = etohl(aSDOp->ldata[0]);
					/* Does parameter fit in parameter buffer ? */
					if (SDOlen <= *psize)
					{
						bp = static_cast<uint8_t*>(p);
						hp = static_cast<uint8_t*>(p);
						/* calculate mailbox transfer size */
						Framedatasize = (etohs(aSDOp->MbxHeader.length) - 10);
						if (Framedatasize < SDOlen) /* transfer in segments? */
						{
							/* copy parameter data in parameter buffer */
							memcpy(hp, &aSDOp->ldata[1], Framedatasize);
							/* increment buffer pointer */
							hp += Framedatasize;
							*psize = Framedatasize;
							NotLast = TRUE;
							toggle = 0x00;
							while (NotLast) /* segmented transfer */
							{
								SDOp = (ec_SDOt *)&MbxOut;
								SDOp->MbxHeader.length = htoes(0x000a);
								SDOp->MbxHeader.address = htoes(0x0000);
								SDOp->MbxHeader.priority = 0x00;
								cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
								slavelist[slave].mbx_cnt = cnt;
								SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
								SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* number 9bits service upper 4 bits (SDO request) */
								SDOp->Command = ECT_SDO_SEG_UP_REQ + toggle; /* segment upload request */
								SDOp->Index = htoes(index);
								SDOp->SubIndex = subindex;
								SDOp->ldata[0] = 0;
								/* send segmented upload request to slave */
								wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
								/* is mailbox transfered to slave ? */
								if (wkc > 0)
								{
									ec_clearmbx(&MbxIn);
									/* read slave response */
									wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
									/* has slave responded ? */
									if (wkc > 0)
									{
										/* slave response should be CoE, SDO response */
										if ((((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
											((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
											((aSDOp->Command & 0xe0) == 0x00)))
										{
											/* calculate mailbox transfer size */
											Framedatasize = etohs(aSDOp->MbxHeader.length) - 3;
											if ((aSDOp->Command & 0x01) > 0)
											{ /* last segment */
												NotLast = FALSE;
												if (Framedatasize == 7)
													/* substract unused bytes from frame */
													Framedatasize = Framedatasize - ((aSDOp->Command & 0x0e) >> 1);
												/* copy to parameter buffer */
												memcpy(hp, &(aSDOp->Index), Framedatasize);
											}
											else /* segments follow */
											{
												/* copy to parameter buffer */
												memcpy(hp, &(aSDOp->Index), Framedatasize);
												/* increment buffer pointer */
												hp += Framedatasize;
											}
											/* update parametersize */
											*psize += Framedatasize;
										}
										/* unexpected frame returned from slave */
										else
										{
											NotLast = FALSE;
											if ((aSDOp->Command) == ECT_SDO_ABORT) /* SDO abort frame received */
												SDOerror(slave, index, subindex, etohl(aSDOp->ldata[0]));
											else
												packeterror(slave, index, subindex, 1); /* Unexpected frame returned */
											wkc = 0;
										}
									}
								}
								toggle = toggle ^ 0x10; /* toggle bit for segment request */
							}
						}
						/* non segmented transfer */
						else
						{
							/* copy to parameter buffer */
							memcpy(bp, &aSDOp->ldata[1], SDOlen);
							*psize = SDOlen;
						}
					}
					/* parameter buffer too small */
					else
					{
						wkc = 0;
						packeterror(slave, index, subindex, 3); /*  data container too small for type */
					}
				}
			}
			/* other slave response */
			else
			{
				if ((aSDOp->Command) == ECT_SDO_ABORT) /* SDO abort frame received */
				{
					SDOerror(slave, index, subindex, etohl(aSDOp->ldata[0]));
				}
				else
				{
					packeterror(slave, index, subindex, 1); /* Unexpected frame returned */
				}
				wkc = 0;
			}
		}
	}
	return wkc;
}

/** CoE SDO write, blocking. Single subindex or Complete Access.
*
* A "normal" download request is issued, unless we have
* small data, then a "expedited" transfer is used. If the parameter is larger than
* the mailbox size then the download is segmented. The function will split the
* parameter data in segments and send them to the slave one by one.
*
* @param[in]  context    = context struct
* @param[in]  Slave      = Slave number
* @param[in]  Index      = Index to write
* @param[in]  SubIndex   = Subindex to write, must be 0 or 1 if CA is used.
* @param[in]  CA         = FALSE = single subindex. TRUE = Complete Access, all subindexes written.
* @param[in]  psize      = Size in bytes of parameter buffer.
* @param[out] p          = Pointer to parameter buffer
* @param[in]  Timeout    = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
	boolean CA, int psize, void *p, int Timeout)
{
	ec_SDOt *SDOp, *aSDOp;
	int wkc, maxdata;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt, toggle;
	uint16 framedatasize;
	boolean  NotLast;
	uint8 *hp;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
	wkc = mbxreceive(Slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSDOp = (ec_SDOt *)&MbxIn;
	SDOp = (ec_SDOt *)&MbxOut;
	maxdata = slavelist[Slave].mbx_l - 0x10; /* data section=mailbox size - 6 mbx - 2 CoE - 8 sdo req */
	/* if small data use expedited transfer */
	if ((psize <= 4) && !CA)
	{
		SDOp->MbxHeader.length = htoes(0x000a);
		SDOp->MbxHeader.address = htoes(0x0000);
		SDOp->MbxHeader.priority = 0x00;
		/* get new mailbox counter, used for session handle */
		cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
		slavelist[Slave].mbx_cnt = cnt;
		SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
		SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* number 9bits service upper 4 bits */
		SDOp->Command = ECT_SDO_DOWN_EXP | (((4 - psize) << 2) & 0x0c); /* expedited SDO download transfer */
		SDOp->Index = htoes(Index);
		SDOp->SubIndex = SubIndex;
		hp = static_cast<uint8_t*>(p);
		/* copy parameter data to mailbox */
		memcpy(&SDOp->ldata[0], hp, psize);
		/* send mailbox SDO download request to slave */
		wkc = mbxsend(Slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
		if (wkc > 0)
		{
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = mbxreceive(Slave, (ec_mbxbuft *)&MbxIn, Timeout);
			if (wkc > 0)
			{
				/* response should be CoE, SDO response, correct index and subindex */
				if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
					((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
					(aSDOp->Index == SDOp->Index) &&
					(aSDOp->SubIndex == SDOp->SubIndex))
				{
					/* all OK */
				}
				/* unexpected response from slave */
				else
				{
					if (aSDOp->Command == ECT_SDO_ABORT) /* SDO abort frame received */
					{
						SDOerror(Slave, Index, SubIndex, etohl(aSDOp->ldata[0]));
					}
					else
					{
						packeterror(Slave, Index, SubIndex, 1); /* Unexpected frame returned */
					}
					wkc = 0;
				}
			}
		}
	}
	else
	{
		framedatasize = psize;
		NotLast = FALSE;
		if (framedatasize > maxdata)
		{
			framedatasize = maxdata;  /*  segmented transfer needed  */
			NotLast = TRUE;
		}
		SDOp->MbxHeader.length = htoes(0x0a + framedatasize);
		SDOp->MbxHeader.address = htoes(0x0000);
		SDOp->MbxHeader.priority = 0x00;
		/* get new mailbox counter, used for session handle */
		cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
		slavelist[Slave].mbx_cnt = cnt;
		SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
		SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* number 9bits service upper 4 bits */
		if (CA)
		{
			SDOp->Command = ECT_SDO_DOWN_INIT_CA; /* Complete Access, normal SDO init download transfer */
		}
		else
		{
			SDOp->Command = ECT_SDO_DOWN_INIT; /* normal SDO init download transfer */
		}
		SDOp->Index = htoes(Index);
		SDOp->SubIndex = SubIndex;
		if (CA && (SubIndex > 1))
		{
			SDOp->SubIndex = 1;
		}
		SDOp->ldata[0] = htoel(psize);
		hp = static_cast<uint8_t*>(p);
		/* copy parameter data to mailbox */
		memcpy(&SDOp->ldata[1], hp, framedatasize);
		hp += framedatasize;
		psize -= framedatasize;
		/* send mailbox SDO download request to slave */
		wkc = mbxsend(Slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
		if (wkc > 0)
		{
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = mbxreceive(Slave, (ec_mbxbuft *)&MbxIn, Timeout);
			if (wkc > 0)
			{
				/* response should be CoE, SDO response, correct index and subindex */
				if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
					((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
					(aSDOp->Index == SDOp->Index) &&
					(aSDOp->SubIndex == SDOp->SubIndex))
				{
					/* all ok */
					maxdata += 7;
					toggle = 0;
					/* repeat while segments left */
					while (NotLast)
					{
						SDOp = (ec_SDOt *)&MbxOut;
						framedatasize = psize;
						NotLast = FALSE;
						SDOp->Command = 0x01; /* last segment */
						if (framedatasize > maxdata)
						{
							framedatasize = maxdata;  /*  more segments needed  */
							NotLast = TRUE;
							SDOp->Command = 0x00; /* segments follow */
						}
						if (!NotLast && (framedatasize < 7))
						{
							SDOp->MbxHeader.length = htoes(0x0a); /* minimum size */
							SDOp->Command = 0x01 + ((7 - framedatasize) << 1); /* last segment reduced octets */
						}
						else
						{
							SDOp->MbxHeader.length = htoes(framedatasize + 3); /* data + 2 CoE + 1 SDO */
						}
						SDOp->MbxHeader.address = htoes(0x0000);
						SDOp->MbxHeader.priority = 0x00;
						/* get new mailbox counter value */
						cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
						slavelist[Slave].mbx_cnt = cnt;
						SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
						SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* number 9bits service upper 4 bits (SDO request) */
						SDOp->Command = SDOp->Command + toggle; /* add toggle bit to command byte */
						/* copy parameter data to mailbox */
						memcpy(&SDOp->Index, hp, framedatasize);
						/* update parameter buffer pointer */
						hp += framedatasize;
						psize -= framedatasize;
						/* send SDO download request */
						wkc = mbxsend(Slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
						if (wkc > 0)
						{
							ec_clearmbx(&MbxIn);
							/* read slave response */
							wkc = mbxreceive(Slave, (ec_mbxbuft *)&MbxIn, Timeout);
							if (wkc > 0)
							{
								if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
									((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
									((aSDOp->Command & 0xe0) == 0x20))
								{
									/* all OK, nothing to do */
								}
								else
								{
									if (aSDOp->Command == ECT_SDO_ABORT) /* SDO abort frame received */
									{
										SDOerror(Slave, Index, SubIndex, etohl(aSDOp->ldata[0]));
									}
									else
									{
										packeterror(Slave, Index, SubIndex, 1); /* Unexpected frame returned */
									}
									wkc = 0;
									NotLast = FALSE;
								}
							}
						}
						toggle = toggle ^ 0x10; /* toggle bit for segment request */
					}
				}
				/* unexpected response from slave */
				else
				{
					if (aSDOp->Command == ECT_SDO_ABORT) /* SDO abort frame received */
					{
						SDOerror(Slave, Index, SubIndex, etohl(aSDOp->ldata[0]));
					}
					else
					{
						packeterror(Slave, Index, SubIndex, 1); /* Unexpected frame returned */
					}
					wkc = 0;
				}
			}
		}
	}

	return wkc;
}

/** CoE RxPDO write, blocking.
*
* A RxPDO download request is issued.
*
* @param[in]  context       = context struct
* @param[in]  Slave         = Slave number
* @param[in]  RxPDOnumber   = Related RxPDO number
* @param[in]  psize         = Size in bytes of PDO buffer.
* @param[out] p             = Pointer to PDO buffer
* @return Workcounter from last slave response
*/
int ecx_contextt::RxPDO(uint16 Slave, uint16 RxPDOnumber, int psize, void *p)
{
	ec_SDOt *SDOp;
	int wkc, maxdata;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;
	uint16 framedatasize;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
	wkc = mbxreceive(Slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	SDOp = (ec_SDOt *)&MbxOut;
	maxdata = slavelist[Slave].mbx_l - 0x08; /* data section=mailbox size - 6 mbx - 2 CoE */
	framedatasize = psize;
	if (framedatasize > maxdata)
	{
		framedatasize = maxdata;  /*  limit transfer */
	}
	SDOp->MbxHeader.length = htoes(0x02 + framedatasize);
	SDOp->MbxHeader.address = htoes(0x0000);
	SDOp->MbxHeader.priority = 0x00;
	/* get new mailbox counter, used for session handle */
	cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
	slavelist[Slave].mbx_cnt = cnt;
	SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
	SDOp->CANOpen = htoes((RxPDOnumber & 0x01ff) + (ECT_COES_RXPDO << 12)); /* number 9bits service upper 4 bits */
	/* copy PDO data to mailbox */
	memcpy(&SDOp->Command, p, framedatasize);
	/* send mailbox RxPDO request to slave */
	wkc = mbxsend(Slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);

	return wkc;
}

/** CoE TxPDO read remote request, blocking.
*
* A RxPDO download request is issued.
*
* @param[in]  context       = context struct
* @param[in]  slave         = Slave number
* @param[in]  TxPDOnumber   = Related TxPDO number
* @param[in,out] psize      = Size in bytes of PDO buffer, returns bytes read from PDO.
* @param[out] p             = Pointer to PDO buffer
* @param[in]  timeout       = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::TxPDO(uint16 slave, uint16 TxPDOnumber, int *psize, void *p, int timeout)
{
	ec_SDOt *SDOp, *aSDOp;
	int wkc;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;
	uint16 framedatasize;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
	wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSDOp = (ec_SDOt *)&MbxIn;
	SDOp = (ec_SDOt *)&MbxOut;
	SDOp->MbxHeader.length = htoes(0x02);
	SDOp->MbxHeader.address = htoes(0x0000);
	SDOp->MbxHeader.priority = 0x00;
	/* get new mailbox counter, used for session handle */
	cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
	slavelist[slave].mbx_cnt = cnt;
	SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
	SDOp->CANOpen = htoes((TxPDOnumber & 0x01ff) + (ECT_COES_TXPDO_RR << 12)); /* number 9bits service upper 4 bits */
	wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
	if (wkc > 0)
	{
		/* clean mailboxbuffer */
		ec_clearmbx(&MbxIn);
		/* read slave response */
		wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
		if (wkc > 0) /* succeeded to read slave response ? */
		{
			/* slave response should be CoE, TxPDO */
			if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
				((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_TXPDO))
			{
				/* TxPDO response */
				framedatasize = (aSDOp->MbxHeader.length - 2);
				if (*psize >= framedatasize) /* parameter buffer big enough ? */
				{
					/* copy parameter in parameter buffer */
					memcpy(p, &aSDOp->Command, framedatasize);
					/* return the real parameter size */
					*psize = framedatasize;
				}
				/* parameter buffer too small */
				else
				{
					wkc = 0;
					packeterror(slave, 0, 0, 3); /*  data container too small for type */
				}
			}
			/* other slave response */
			else
			{
				if ((aSDOp->Command) == ECT_SDO_ABORT) /* SDO abort frame received */
				{
					SDOerror(slave, 0, 0, etohl(aSDOp->ldata[0]));
				}
				else
				{
					packeterror(slave, 0, 0, 1); /* Unexpected frame returned */
				}
				wkc = 0;
			}
		}
	}

	return wkc;
}

/** Read PDO assign structure
* @param[in]  context       = context struct
* @param[in]  Slave         = Slave number
* @param[in]  PDOassign     = PDO assign object
* @return total bitlength of PDO assign
*/
int ecx_contextt::readPDOassign(uint16 Slave, uint16 PDOassign)
{
	uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
	uint8 subcnt;
	int wkc, bsize = 0, rdl;
	int32 rdat2;

	rdl = sizeof(rdat); rdat = 0;
	/* read PDO assign subindex 0 ( = number of PDO's) */
	wkc = SDOread(Slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
	rdat = etohs(rdat);
	/* positive result from slave ? */
	if ((wkc > 0) && (rdat > 0))
	{
		/* number of available sub indexes */
		nidx = rdat;
		bsize = 0;
		/* read all PDO's */
		for (idxloop = 1; idxloop <= nidx; idxloop++)
		{
			rdl = sizeof(rdat); rdat = 0;
			/* read PDO assign */
			wkc = SDOread(Slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
			/* result is index of PDO */
			idx = etohl(rdat);
			if (idx > 0)
			{
				rdl = sizeof(subcnt); subcnt = 0;
				/* read number of subindexes of PDO */
				wkc = SDOread(Slave, idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
				subidx = subcnt;
				/* for each subindex */
				for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
				{
					rdl = sizeof(rdat2); rdat2 = 0;
					/* read SDO that is mapped in PDO */
					wkc = SDOread(Slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
					rdat2 = etohl(rdat2);
					/* extract bitlength of SDO */
					if (LO_BYTE(rdat2) < 0xff)
					{
						bsize += LO_BYTE(rdat2);
					}
					else
					{
						rdl = sizeof(rdat); rdat = htoes(0xff);
						/* read Object Entry in Object database */
						//                  wkc = ec_readOEsingle(idx, (uint8)SubCount, pODlist, pOElist);
						bsize += etohs(rdat);
					}
				}
			}
		}
	}
	/* return total found bitlength (PDO) */
	return bsize;
}

/** Read PDO assign structure in Complete Access mode
* @param[in]  context       = context struct
* @param[in]  Slave         = Slave number
* @param[in]  PDOassign     = PDO assign object
* @return total bitlength of PDO assign
*/
int ecx_contextt::readPDOassignCA(uint16 Slave, uint16 pdoAssign)
{
	uint16 idxloop, nidx, subidxloop, idx, subidx;
	int wkc, bsize = 0, rdl;

	/* find maximum size of PDOassign buffer */
	rdl = sizeof(ec_PDOassignt);
	PDOassign->n = 0;
	/* read rxPDOassign in CA mode, all subindexes are read in one struct */
	wkc = SDOread(Slave, pdoAssign, 0x00, TRUE, &rdl, PDOassign, EC_TIMEOUTRXM);
	/* positive result from slave ? */
	if ((wkc > 0) && (PDOassign->n > 0))
	{
		nidx = PDOassign->n;
		bsize = 0;
		/* for each PDO do */
		for (idxloop = 1; idxloop <= nidx; idxloop++)
		{
			/* get index from PDOassign struct */
			idx = etohs(PDOassign->index[idxloop - 1]);
			if (idx > 0)
			{
				rdl = sizeof(ec_PDOdesct); PDOdesc->n = 0;
				/* read SDO's that are mapped in PDO, CA mode */
				wkc = SDOread(Slave, idx, 0x00, TRUE, &rdl, PDOdesc, EC_TIMEOUTRXM);
				subidx = PDOdesc->n;
				/* extract all bitlengths of SDO's */
				for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
				{
					bsize += LO_BYTE(etohl(PDOdesc->PDO[subidxloop - 1]));
				}
			}
		}
	}

	/* return total found bitlength (PDO) */
	return bsize;
}

/** CoE read PDO mapping.
*
* CANopen has standard indexes defined for PDO mapping. This function
* tries to read them and collect a full input and output mapping size
* of designated slave.
*
* Principal structure in slave:\n
* 1C00:00 is number of SM defined\n
* 1C00:01 SM0 type -> 1C10\n
* 1C00:02 SM1 type -> 1C11\n
* 1C00:03 SM2 type -> 1C12\n
* 1C00:04 SM3 type -> 1C13\n
* Type 0 = unused, 1 = mailbox in, 2 = mailbox out,
* 3 = outputs (RxPDO), 4 = inputs (TxPDO).
*
* 1C12:00 is number of PDO's defined for SM2\n
* 1C12:01 PDO assign SDO #1 -> f.e. 1A00\n
* 1C12:02 PDO assign SDO #2 -> f.e. 1A04\
*
* 1A00:00 is number of object defined for this PDO\n
* 1A00:01 object mapping #1, f.e. 60100710 (SDO 6010 SI 07 bitlength 0x10)
*
* @param[in]  context = context struct
* @param[in]  Slave   = Slave number
* @param[out] Osize   = Size in bits of output mapping (rxPDO) found
* @param[out] Isize   = Size in bits of input mapping (txPDO) found
* @return >0 if mapping succesful.
*/
int ecx_contextt::readPDOmap(uint16 Slave, int *Osize, int *Isize)
{
	int wkc, rdl;
	int retVal = 0;
	uint8 nSM, iSM, tSM;
	int Tsize;
	uint8 SMt_bug_add;

	*Isize = 0;
	*Osize = 0;
	SMt_bug_add = 0;
	rdl = sizeof(nSM); nSM = 0;
	/* read SyncManager Communication Type object count */
	wkc = SDOread(Slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
	/* positive result from slave ? */
	if ((wkc > 0) && (nSM > 2))
	{
		/* limit to maximum number of SM defined, if true the slave can't be configured */
		if (nSM > EC_MAXSM)
			nSM = EC_MAXSM;
		/* iterate for every SM type defined */
		for (iSM = 2; iSM < nSM; iSM++)
		{
			rdl = sizeof(tSM); tSM = 0;
			/* read SyncManager Communication Type */
			wkc = SDOread(Slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
			if (wkc > 0)
			{
				// start slave bug prevention code, remove if possible
				if ((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
				{
					SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
				}
				if (tSM)
				{
					tSM += SMt_bug_add; // only add if SMt > 0
				}
				if ((iSM == 2) && (tSM == 0)) // SM2 has type 0, this is a bug in the slave!
				{
					tSM = 3;
				}
				if ((iSM == 3) && (tSM == 0)) // SM3 has type 0, this is a bug in the slave!
				{
					tSM = 4;
				}
				// end slave bug prevention code

				slavelist[Slave].SMtype[iSM] = tSM;
				/* check if SM is unused -> clear enable flag */
				if (tSM == 0)
				{
					slavelist[Slave].SM[iSM].SMflags =
						htoel(etohl(slavelist[Slave].SM[iSM].SMflags) & EC_SMENABLEMASK);
				}
				if ((tSM == 3) || (tSM == 4))
				{
					/* read the assign PDO */
					Tsize = readPDOassign(Slave, ECT_SDO_PDOASSIGN + iSM);
					/* if a mapping is found */
					if (Tsize)
					{
						slavelist[Slave].SM[iSM].SMlength = htoes((Tsize + 7) / 8);
						if (tSM == 3)
						{
							/* we are doing outputs */
							*Osize += Tsize;
						}
						else
						{
							/* we are doing inputs */
							*Isize += Tsize;
						}
					}
				}
			}
		}
	}

	/* found some I/O bits ? */
	if ((*Isize > 0) || (*Osize > 0))
	{
		retVal = 1;
	}

	return retVal;
}

/** CoE read PDO mapping in Complete Access mode (CA).
*
* CANopen has standard indexes defined for PDO mapping. This function
* tries to read them and collect a full input and output mapping size
* of designated slave. Slave has to support CA, otherwise use ec_readPDOmap().
*
* @param[in]  context = context struct
* @param[in]  Slave   = Slave number
* @param[out] Osize   = Size in bits of output mapping (rxPDO) found
* @param[out] Isize   = Size in bits of input mapping (txPDO) found
* @return >0 if mapping succesful.
*/
int ecx_contextt::readPDOmapCA(uint16 Slave, int *Osize, int *Isize)
{
	int wkc, rdl;
	int retVal = 0;
	uint8 nSM, iSM, tSM;
	int Tsize;
	uint8 SMt_bug_add;

	*Isize = 0;
	*Osize = 0;
	SMt_bug_add = 0;
	rdl = sizeof(ec_SMcommtypet);
	SMcommtype->n = 0;
	/* read SyncManager Communication Type object count Complete Access*/
	wkc = SDOread(Slave, ECT_SDO_SMCOMMTYPE, 0x00, TRUE, &rdl, SMcommtype, EC_TIMEOUTRXM);
	/* positive result from slave ? */
	if ((wkc > 0) && (SMcommtype->n > 2))
	{
		nSM = SMcommtype->n;
		/* limit to maximum number of SM defined, if true the slave can't be configured */
		if (nSM > EC_MAXSM)
		{
			nSM = EC_MAXSM;
			packeterror(Slave, 0, 0, 10); /* #SM larger than EC_MAXSM */
		}
		/* iterate for every SM type defined */
		for (iSM = 2; iSM < nSM; iSM++)
		{
			tSM = SMcommtype->SMtype[iSM];

			// start slave bug prevention code, remove if possible
			if ((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
			{
				SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
			}
			if (tSM)
			{
				tSM += SMt_bug_add; // only add if SMt > 0
			}
			// end slave bug prevention code

			slavelist[Slave].SMtype[iSM] = tSM;
			/* check if SM is unused -> clear enable flag */
			if (tSM == 0)
			{
				slavelist[Slave].SM[iSM].SMflags =
					htoel(etohl(slavelist[Slave].SM[iSM].SMflags) & EC_SMENABLEMASK);
			}
			if ((tSM == 3) || (tSM == 4))
			{
				/* read the assign PDO */
				Tsize = readPDOassignCA(Slave, ECT_SDO_PDOASSIGN + iSM);
				/* if a mapping is found */
				if (Tsize)
				{
					slavelist[Slave].SM[iSM].SMlength = htoes((Tsize + 7) / 8);
					if (tSM == 3)
					{
						/* we are doing outputs */
						*Osize += Tsize;
					}
					else
					{
						/* we are doing inputs */
						*Isize += Tsize;
					}
				}
			}
		}
	}

	/* found some I/O bits ? */
	if ((*Isize > 0) || (*Osize > 0))
	{
		retVal = 1;
	}
	return retVal;
}

/** SDO service structure */
PACKED_BEGIN
struct ec_SDOservicet PACKED
{
	ec_mbxheadert   MbxHeader;
	uint16          CANOpen;
	uint8           Opcode;
	uint8           Reserved;
	uint16          Fragments;
	union
	{
		uint8   bdata[0x200]; /* variants for easy data access */
		uint16  wdata[0x100];
		uint32  ldata[0x80];
	};
};
PACKED_END
/** CoE read Object Description List.
*
* @param[in]  context  = context struct
* @param[in]  Slave    = Slave number.
* @param[out] pODlist  = resulting Object Description list.
* @return Workcounter of slave response.
*/
int ecx_contextt::readODlist(uint16 Slave, ec_ODlistt *pODlist)
{
	ec_SDOservicet *SDOp, *aSDOp;
	ec_mbxbuft MbxIn, MbxOut;
	int wkc;
	uint16 x, n, i, sp, offset;
	boolean stop;
	uint8 cnt;
	boolean First;

	pODlist->Slave = Slave;
	pODlist->Entries = 0;
	ec_clearmbx(&MbxIn);
	/* clear pending out mailbox in slave if available. Timeout is set to 0 */
	wkc = mbxreceive(Slave, &MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSDOp = (ec_SDOservicet*)&MbxIn;
	SDOp = (ec_SDOservicet*)&MbxOut;
	SDOp->MbxHeader.length = htoes(0x0008);
	SDOp->MbxHeader.address = htoes(0x0000);
	SDOp->MbxHeader.priority = 0x00;
	/* Get new mailbox counter value */
	cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
	slavelist[Slave].mbx_cnt = cnt;
	SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
	SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOINFO << 12)); /* number 9bits service upper 4 bits */
	SDOp->Opcode = ECT_GET_ODLIST_REQ; /* get object description list request */
	SDOp->Reserved = 0;
	SDOp->Fragments = 0; /* fragments left */
	SDOp->wdata[0] = htoes(0x01); /* all objects */
	/* send get object description list request to slave */
	wkc = mbxsend(Slave, &MbxOut, EC_TIMEOUTTXM);
	/* mailbox placed in slave ? */
	if (wkc > 0)
	{
		x = 0;
		sp = 0;
		First = TRUE;
		offset = 1; /* offset to skip info header in first frame, otherwise set to 0 */
		do
		{
			stop = TRUE; /* assume this is last iteration */
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = mbxreceive(Slave, &MbxIn, EC_TIMEOUTRXM);
			/* got response ? */
			if (wkc > 0)
			{
				/* response should be CoE and "get object description list response" */
				if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
					((aSDOp->Opcode & 0x7f) == ECT_GET_ODLIST_RES))
				{
					if (First)
					{
						/* extract number of indexes from mailbox data size */
						n = (etohs(aSDOp->MbxHeader.length) - (6 + 2)) / 2;
					}
					else
					{
						/* extract number of indexes from mailbox data size */
						n = (etohs(aSDOp->MbxHeader.length) - 6) / 2;
					}
					/* check if indexes fit in buffer structure */
					if ((sp + n) > EC_MAXODLIST)
					{
						n = EC_MAXODLIST + 1 - sp;
						SDOinfoerror(Slave, 0, 0, 0xf000000); /* Too many entries for master buffer */
						stop = TRUE;
					}
					/* trim to maximum number of ODlist entries defined */
					if ((pODlist->Entries + n) > EC_MAXODLIST)
					{
						n = EC_MAXODLIST - pODlist->Entries;
					}
					pODlist->Entries += n;
					/* extract indexes one by one */
					for (i = 0; i < n; i++)
					{
						pODlist->Index[sp + i] = etohs(aSDOp->wdata[i + offset]);
					}
					sp += n;
					/* check if more fragments will follow */
					if (aSDOp->Fragments > 0)
					{
						stop = FALSE;
					}
					First = FALSE;
					offset = 0;
				}
				/* got unexpected response from slave */
				else
				{
					if ((aSDOp->Opcode & 0x7f) == ECT_SDOINFO_ERROR) /* SDO info error received */
					{
						SDOinfoerror(Slave, 0, 0, etohl(aSDOp->ldata[0]));
						stop = TRUE;
					}
					else
					{
						packeterror(Slave, 0, 0, 1); /* Unexpected frame returned */
					}
					wkc = 0;
					x += 20;
				}
			}
			x++;
		} while ((x <= 128) && !stop);
	}
	return wkc;
}

/** CoE read Object Description. Adds textual description to object indexes.
*
* @param[in]  context       = context struct
* @param[in] Item           = Item number in ODlist.
* @param[in,out] pODlist    = referencing Object Description list.
* @return Workcounter of slave response.
*/
int ecx_contextt::readODdescription(uint16 Item, ec_ODlistt *pODlist)
{
	ec_SDOservicet *SDOp, *aSDOp;
	int wkc;
	uint16  n, Slave;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;

	Slave = pODlist->Slave;
	pODlist->DataType[Item] = 0;
	pODlist->ObjectCode[Item] = 0;
	pODlist->MaxSub[Item] = 0;
	pODlist->Name[Item][0] = 0;
	ec_clearmbx(&MbxIn);
	/* clear pending out mailbox in slave if available. Timeout is set to 0 */
	wkc = mbxreceive(Slave, &MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSDOp = (ec_SDOservicet*)&MbxIn;
	SDOp = (ec_SDOservicet*)&MbxOut;
	SDOp->MbxHeader.length = htoes(0x0008);
	SDOp->MbxHeader.address = htoes(0x0000);
	SDOp->MbxHeader.priority = 0x00;
	/* Get new mailbox counter value */
	cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
	slavelist[Slave].mbx_cnt = cnt;
	SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
	SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOINFO << 12)); /* number 9bits service upper 4 bits */
	SDOp->Opcode = ECT_GET_OD_REQ; /* get object description request */
	SDOp->Reserved = 0;
	SDOp->Fragments = 0; /* fragments left */
	SDOp->wdata[0] = htoes(pODlist->Index[Item]); /* Data of Index */
	/* send get object description request to slave */
	wkc = mbxsend(Slave, &MbxOut, EC_TIMEOUTTXM);
	/* mailbox placed in slave ? */
	if (wkc > 0)
	{
		ec_clearmbx(&MbxIn);
		/* read slave response */
		wkc = mbxreceive(Slave, &MbxIn, EC_TIMEOUTRXM);
		/* got response ? */
		if (wkc > 0)
		{
			if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
				((aSDOp->Opcode & 0x7f) == ECT_GET_OD_RES))
			{
				n = (etohs(aSDOp->MbxHeader.length) - 12); /* length of string(name of object) */
				if (n > EC_MAXNAME)
				{
					n = EC_MAXNAME; /* max chars */
				}
				pODlist->DataType[Item] = etohs(aSDOp->wdata[1]);
				pODlist->ObjectCode[Item] = aSDOp->bdata[5];
				pODlist->MaxSub[Item] = aSDOp->bdata[4];

				strncpy(pODlist->Name[Item], (char *)&aSDOp->bdata[6], n);
				pODlist->Name[Item][n] = 0x00; /* String terminator */
			}
			/* got unexpected response from slave */
			else
			{
				if (((aSDOp->Opcode & 0x7f) == ECT_SDOINFO_ERROR)) /* SDO info error received */
				{
					SDOinfoerror(Slave, pODlist->Index[Item], 0, etohl(aSDOp->ldata[0]));
				}
				else
				{
					packeterror(Slave, pODlist->Index[Item], 0, 1); /* Unexpected frame returned */
				}
				wkc = 0;
			}
		}
	}

	return wkc;
}

/** CoE read SDO service object entry, single subindex.
* Used in ec_readOE().
*
* @param[in]  context       = context struct
* @param[in] Item           = Item in ODlist.
* @param[in] SubI           = Subindex of item in ODlist.
* @param[in] pODlist        = Object description list for reference.
* @param[out] pOElist       = resulting object entry structure.
* @return Workcounter of slave response.
*/
int ecx_contextt::readOEsingle(uint16 Item, uint8 SubI, ec_ODlistt *pODlist, ec_OElistt *pOElist)
{
	ec_SDOservicet *SDOp, *aSDOp;
	int wkc;
	uint16 Index, Slave;
	int16 n;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;

	wkc = 0;
	Slave = pODlist->Slave;
	Index = pODlist->Index[Item];
	ec_clearmbx(&MbxIn);
	/* clear pending out mailbox in slave if available. Timeout is set to 0 */
	wkc = mbxreceive(Slave, &MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSDOp = (ec_SDOservicet*)&MbxIn;
	SDOp = (ec_SDOservicet*)&MbxOut;
	SDOp->MbxHeader.length = htoes(0x000a);
	SDOp->MbxHeader.address = htoes(0x0000);
	SDOp->MbxHeader.priority = 0x00;
	/* Get new mailbox counter value */
	cnt = ec_nextmbxcnt(slavelist[Slave].mbx_cnt);
	slavelist[Slave].mbx_cnt = cnt;
	SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
	SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOINFO << 12)); /* number 9bits service upper 4 bits */
	SDOp->Opcode = ECT_GET_OE_REQ; /* get object entry description request */
	SDOp->Reserved = 0;
	SDOp->Fragments = 0;      /* fragments left */
	SDOp->wdata[0] = htoes(Index);      /* Index */
	SDOp->bdata[2] = SubI;       /* SubIndex */
	SDOp->bdata[3] = 1 + 2 + 4; /* get access rights, object category, PDO */
	/* send get object entry description request to slave */
	wkc = mbxsend( Slave, &MbxOut, EC_TIMEOUTTXM);
	/* mailbox placed in slave ? */
	if (wkc > 0)
	{
		ec_clearmbx(&MbxIn);
		/* read slave response */
		wkc = mbxreceive(Slave, &MbxIn, EC_TIMEOUTRXM);
		/* got response ? */
		if (wkc > 0)
		{
			if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
				((aSDOp->Opcode & 0x7f) == ECT_GET_OE_RES))
			{
				pOElist->Entries++;
				n = (etohs(aSDOp->MbxHeader.length) - 16); /* length of string(name of object) */
				if (n > EC_MAXNAME)
				{
					n = EC_MAXNAME; /* max string length */
				}
				if (n < 0)
				{
					n = 0;
				}
				pOElist->ValueInfo[SubI] = aSDOp->bdata[3];
				pOElist->DataType[SubI] = etohs(aSDOp->wdata[2]);
				pOElist->BitLength[SubI] = etohs(aSDOp->wdata[3]);
				pOElist->ObjAccess[SubI] = etohs(aSDOp->wdata[4]);

				strncpy(pOElist->Name[SubI], (char *)&aSDOp->wdata[5], n);
				pOElist->Name[SubI][n] = 0x00; /* string terminator */
			}
			/* got unexpected response from slave */
			else
			{
				if (((aSDOp->Opcode & 0x7f) == ECT_SDOINFO_ERROR)) /* SDO info error received */
				{
					SDOinfoerror(Slave, Index, SubI, etohl(aSDOp->ldata[0]));
				}
				else
				{
					packeterror(Slave, Index, SubI, 1); /* Unexpected frame returned */
				}
				wkc = 0;
			}
		}
	}

	return wkc;
}

/** CoE read SDO service object entry.
*
* @param[in] context        = context struct
* @param[in] Item           = Item in ODlist.
* @param[in] pODlist        = Object description list for reference.
* @param[out] pOElist       = resulting object entry structure.
* @return Workcounter of slave response.
*/
int ecx_contextt::readOE(uint16 Item, ec_ODlistt *pODlist, ec_OElistt *pOElist)
{
	uint16 SubCount;
	int wkc;
	uint8 SubI;

	wkc = 0;
	pOElist->Entries = 0;
	SubI = pODlist->MaxSub[Item];
	/* for each entry found in ODlist */
	for (SubCount = 0; SubCount <= SubI; SubCount++)
	{
		/* read subindex of entry */
		wkc = readOEsingle(Item, (uint8)SubCount, pODlist, pOElist);
	}

	return wkc;
}

void ecx_contextt::init_context()
{
	int lp;
	*(slavecount) = 0;
	/* clean ec_slave array */
	memset(slavelist, 0x00, sizeof(ec_slavet)* maxslave);
	memset(grouplist, 0x00, sizeof(ec_groupt)* maxgroup);
	/* clear slave eeprom cache, does not actually read any eeprom */
	siigetbyte(0, EC_MAXEEPBUF);
	for (lp = 0; lp < maxgroup; lp++)
	{
		grouplist[lp].logstartaddr = lp << 16; /* default start address per group entry */
	}
}

int ecx_contextt::ecx_detect_slaves()
{
	uint8  b;
	uint16 w;
	int    wkc;

	/* make special pre-init register writes to enable MAC[1] local administered bit *
	* setting for old netX100 slaves */
	b = 0x00;
	port->BWR(0x0000, ECT_REG_DLALIAS, sizeof(b), &b, EC_TIMEOUTRET3);     /* Ignore Alias register */
	b = EC_STATE_INIT | EC_STATE_ACK;
	port->BWR(0x0000, ECT_REG_ALCTL, sizeof(b), &b, EC_TIMEOUTRET3);       /* Reset all slaves to Init */
	/* netX100 should now be happy */
	port->BWR(0x0000, ECT_REG_ALCTL, sizeof(b), &b, EC_TIMEOUTRET3);       /* Reset all slaves to Init */
	wkc = port->BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);  /* detect number of slaves */
	if (wkc > 0)
	{
		/* this is strictly "less than" since the master is "slave 0" */
		if (wkc < EC_MAXSLAVE)
		{
			*(slavecount) = wkc;
		}
		else
		{
			EC_PRINT("Error: too many slaves on network: num_slaves=%d, EC_MAXSLAVE=%d\n",
				wkc, EC_MAXSLAVE);
			return -2;
		}
	}
	return wkc;
}

void ecx_contextt::ecx_set_slaves_to_default()
{
	uint8 b;
	uint16 w;
	uint8 zbuf[64];
	memset(&zbuf, 0x00, sizeof(zbuf));
	b = 0x00;
	port->BWR(0x0000, ECT_REG_DLPORT, sizeof(b), &b, EC_TIMEOUTRET3);     /* deact loop manual */
	w = htoes(0x0004);
	port->BWR(0x0000, ECT_REG_IRQMASK, sizeof(w), &w, EC_TIMEOUTRET3);     /* set IRQ mask */
	port->BWR(0x0000, ECT_REG_RXERR, 8, &zbuf, EC_TIMEOUTRET3);  /* reset CRC counters */
	port->BWR(0x0000, ECT_REG_FMMU0, 16 * 3, &zbuf, EC_TIMEOUTRET3);  /* reset FMMU's */
	port->BWR(0x0000, ECT_REG_SM0, 8 * 4, &zbuf, EC_TIMEOUTRET3);  /* reset SyncM */
	port->BWR(0x0000, ECT_REG_DCSYSTIME, 4, &zbuf, EC_TIMEOUTRET3);  /* reset system time+ofs */
	w = htoes(0x1000);
	port->BWR(0x0000, ECT_REG_DCSPEEDCNT, sizeof(w), &w, EC_TIMEOUTRET3);     /* DC speedstart */
	w = htoes(0x0c00);
	port->BWR(0x0000, ECT_REG_DCTIMEFILT, sizeof(w), &w, EC_TIMEOUTRET3);     /* DC filt expr */
	b = 0x00;
	port->BWR(0x0000, ECT_REG_DLALIAS, sizeof(b), &b, EC_TIMEOUTRET3);     /* Ignore Alias register */
	b = EC_STATE_INIT | EC_STATE_ACK;
	port->BWR(0x0000, ECT_REG_ALCTL, sizeof(b), &b, EC_TIMEOUTRET3);     /* Reset all slaves to Init */
	b = 2;
	port->BWR(0x0000, ECT_REG_EEPCFG, sizeof(b), &b, EC_TIMEOUTRET3);     /* force Eeprom from PDI */
	b = 0;
	port->BWR(0x0000, ECT_REG_EEPCFG, sizeof(b), &b, EC_TIMEOUTRET3);     /* set Eeprom to master */
}

#ifdef EC_VER1
int ecx_contextt::config_from_table(uint16 slave)
{
	ec_slavet *csl = &(slavelist[slave]);
	return EtherCATConfig::find(csl, csl->eep_man, csl->eep_id);
}
#else
int ecx_contextt::config_from_table(uint16 slave)
{
	return 0;
}
#endif

/* If slave has SII and same slave ID done before, use previous data.
* This is safe because SII is constant for same slave ID.
*/
int ecx_contextt::lookup_prev_sii(uint16 slave)
{
	int i, nSM;
	if ((slave > 1) && (*(slavecount) > 0))
	{
		i = 1;
		while (((slavelist[i].eep_man != slavelist[slave].eep_man) ||
			(slavelist[i].eep_id != slavelist[slave].eep_id) ||
			(slavelist[i].eep_rev != slavelist[slave].eep_rev)) &&
			(i < slave))
		{
			i++;
		}
		if (i < slave)
		{
			slavelist[slave].CoEdetails = slavelist[i].CoEdetails;
			slavelist[slave].FoEdetails = slavelist[i].FoEdetails;
			slavelist[slave].EoEdetails = slavelist[i].EoEdetails;
			slavelist[slave].SoEdetails = slavelist[i].SoEdetails;
			if (slavelist[i].blockLRW > 0)
			{
				slavelist[slave].blockLRW = 1;
				slavelist[0].blockLRW++;
			}
			slavelist[slave].Ebuscurrent = slavelist[i].Ebuscurrent;
			slavelist[0].Ebuscurrent += slavelist[slave].Ebuscurrent;
			memcpy(slavelist[slave].name, slavelist[i].name, EC_MAXNAME + 1);
			for (nSM = 0; nSM < EC_MAXSM; nSM++)
			{
				slavelist[slave].SM[nSM].StartAddr = slavelist[i].SM[nSM].StartAddr;
				slavelist[slave].SM[nSM].SMlength = slavelist[i].SM[nSM].SMlength;
				slavelist[slave].SM[nSM].SMflags = slavelist[i].SM[nSM].SMflags;
			}
			slavelist[slave].FMMU0func = slavelist[i].FMMU0func;
			slavelist[slave].FMMU1func = slavelist[i].FMMU1func;
			slavelist[slave].FMMU2func = slavelist[i].FMMU2func;
			slavelist[slave].FMMU3func = slavelist[i].FMMU3func;
			EC_PRINT("Copy SII slave %d from %d.\n", slave, i);
			return 1;
		}
	}
	return 0;
}

/** Enumerate and init all slaves.
*
* @param[in] context      = context struct
* @param[in] usetable     = TRUE when using configtable to init slaves, FALSE otherwise
* @return Workcounter of slave discover datagram = number of slaves found
*/
int ecx_contextt::config_init(uint8 usetable)
{
	uint16 slave, ADPh, configadr, ssigen;
	uint16 topology, estat;
	int16 topoc, slavec, aliasadr;
	uint8 b, h;
	uint8 SMc;
	uint32 eedat;
	int wkc, cindex, nSM;

	EC_PRINT("ec_config_init %d\n", usetable);
	init_context();
	wkc = ecx_detect_slaves();
	if (wkc > 0)
	{
		ecx_set_slaves_to_default();
		for (slave = 1; slave <= *(slavecount); slave++)
		{
			ADPh = (uint16)(1 - slave);
			slavelist[slave].Itype =
				etohs(port->APRDw(ADPh, ECT_REG_PDICTL, EC_TIMEOUTRET3)); /* read interface type of slave */
			/* a node offset is used to improve readibility of network frames */
			/* this has no impact on the number of addressable slaves (auto wrap around) */
			port->APWRw(ADPh, ECT_REG_STADR, htoes(slave + EC_NODEOFFSET), EC_TIMEOUTRET3); /* set node address of slave */
			if (slave == 1)
			{
				b = 1; /* kill non ecat frames for first slave */
			}
			else
			{
				b = 0; /* pass all frames for following slaves */
			}
			port->APWRw(ADPh, ECT_REG_DLCTL, htoes(b), EC_TIMEOUTRET3); /* set non ecat frame behaviour */
			configadr = etohs(port->APRDw(ADPh, ECT_REG_STADR, EC_TIMEOUTRET3));
			slavelist[slave].configadr = configadr;
			port->FPRD(configadr, ECT_REG_ALIAS, sizeof(aliasadr), &aliasadr, EC_TIMEOUTRET3);
			slavelist[slave].aliasadr = etohs(aliasadr);
			port->FPRD(configadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET3);
			estat = etohs(estat);
			if (estat & EC_ESTAT_R64) /* check if slave can read 8 byte chunks */
			{
				slavelist[slave].eep_8byte = 1;
			}
			readeeprom1(slave, ECT_SII_MANUF); /* Manuf */
		}
		for (slave = 1; slave <= *(slavecount); slave++)
		{
			slavelist[slave].eep_man =
				etohl(readeeprom2(slave, EC_TIMEOUTEEP)); /* Manuf */
			readeeprom1(slave, ECT_SII_ID); /* ID */
		}
		for (slave = 1; slave <= *(slavecount); slave++)
		{
			slavelist[slave].eep_id =
				etohl(readeeprom2(slave, EC_TIMEOUTEEP)); /* ID */
			readeeprom1(slave, ECT_SII_REV); /* revision */
		}
		for (slave = 1; slave <= *(slavecount); slave++)
		{
			slavelist[slave].eep_rev =
				etohl(readeeprom2(slave, EC_TIMEOUTEEP)); /* revision */
			readeeprom1(slave, ECT_SII_RXMBXADR); /* write mailbox address + mailboxsize */
		}
		for (slave = 1; slave <= *(slavecount); slave++)
		{
			eedat = etohl(readeeprom2(slave, EC_TIMEOUTEEP)); /* write mailbox address and mailboxsize */
			slavelist[slave].mbx_wo = (uint16)LO_WORD(eedat);
			slavelist[slave].mbx_l = (uint16)HI_WORD(eedat);
			if (slavelist[slave].mbx_l > 0)
			{
				readeeprom1(slave, ECT_SII_TXMBXADR); /* read mailbox offset */
			}
		}
		for (slave = 1; slave <= *(slavecount); slave++)
		{
			if (slavelist[slave].mbx_l > 0)
			{
				eedat = etohl(readeeprom2(slave, EC_TIMEOUTEEP)); /* read mailbox offset */
				slavelist[slave].mbx_ro = (uint16)LO_WORD(eedat); /* read mailbox offset */
				slavelist[slave].mbx_rl = (uint16)HI_WORD(eedat); /*read mailbox length */
				if (slavelist[slave].mbx_rl == 0)
				{
					slavelist[slave].mbx_rl = slavelist[slave].mbx_l;
				}
				readeeprom1(slave, ECT_SII_MBXPROTO);
			}
			configadr = slavelist[slave].configadr;
			if ((etohs(port->FPRDw(configadr, ECT_REG_ESCSUP, EC_TIMEOUTRET3)) & 0x04) > 0)  /* Support DC? */
			{
				slavelist[slave].hasdc = TRUE;
			}
			else
			{
				slavelist[slave].hasdc = FALSE;
			}
			topology = etohs(port->FPRDw(configadr, ECT_REG_DLSTAT, EC_TIMEOUTRET3)); /* extract topology from DL status */
			h = 0;
			b = 0;
			if ((topology & 0x0300) == 0x0200) /* port0 open and communication established */
			{
				h++;
				b |= 0x01;
			}
			if ((topology & 0x0c00) == 0x0800) /* port1 open and communication established */
			{
				h++;
				b |= 0x02;
			}
			if ((topology & 0x3000) == 0x2000) /* port2 open and communication established */
			{
				h++;
				b |= 0x04;
			}
			if ((topology & 0xc000) == 0x8000) /* port3 open and communication established */
			{
				h++;
				b |= 0x08;
			}
			/* ptype = Physical type*/
			slavelist[slave].ptype =
				LO_BYTE(etohs(port->FPRDw(configadr, ECT_REG_PORTDES, EC_TIMEOUTRET3)));
			slavelist[slave].topology = h;
			slavelist[slave].activeports = b;
			/* 0=no links, not possible             */
			/* 1=1 link  , end of line              */
			/* 2=2 links , one before and one after */
			/* 3=3 links , split point              */
			/* 4=4 links , cross point              */
			/* search for parent */
			slavelist[slave].parent = 0; /* parent is master */
			if (slave > 1)
			{
				topoc = 0;
				slavec = slave - 1;
				do
				{
					topology = slavelist[slavec].topology;
					if (topology == 1)
					{
						topoc--; /* endpoint found */
					}
					if (topology == 3)
					{
						topoc++; /* split found */
					}
					if (topology == 4)
					{
						topoc += 2; /* cross found */
					}
					if (((topoc >= 0) && (topology > 1)) ||
						(slavec == 1)) /* parent found */
					{
						slavelist[slave].parent = slavec;
						slavec = 1;
					}
					slavec--;
				} while (slavec > 0);
			}
			(void)statecheck(slave, EC_STATE_INIT, EC_TIMEOUTSTATE); //* check state change Init */

			/* set default mailbox configuration if slave has mailbox */
			if (slavelist[slave].mbx_l>0)
			{
				slavelist[slave].SMtype[0] = 1;
				slavelist[slave].SMtype[1] = 2;
				slavelist[slave].SMtype[2] = 3;
				slavelist[slave].SMtype[3] = 4;
				slavelist[slave].SM[0].StartAddr = htoes(slavelist[slave].mbx_wo);
				slavelist[slave].SM[0].SMlength = htoes(slavelist[slave].mbx_l);
				slavelist[slave].SM[0].SMflags = EtherCATConfig::getDefaultMBXSM0();
				slavelist[slave].SM[1].StartAddr = htoes(slavelist[slave].mbx_ro);
				slavelist[slave].SM[1].SMlength = htoes(slavelist[slave].mbx_rl);
				slavelist[slave].SM[1].SMflags = EtherCATConfig::getDefaultMBXSM1();
				slavelist[slave].mbx_proto = readeeprom2(slave, EC_TIMEOUTEEP);
			}
			cindex = 0;
			/* use configuration table ? */
			if (usetable == 1)
			{
				cindex = config_from_table(slave);
			}
			/* slave not in configuration table, find out via SII */
			if (!cindex && !lookup_prev_sii(slave))
			{
				ssigen = siifind(slave, ECT_SII_GENERAL);
				/* SII general section */
				if (ssigen)
				{
					slavelist[slave].CoEdetails = siigetbyte(slave, ssigen + 0x07);
					slavelist[slave].FoEdetails = siigetbyte(slave, ssigen + 0x08);
					slavelist[slave].EoEdetails = siigetbyte(slave, ssigen + 0x09);
					slavelist[slave].SoEdetails = siigetbyte(slave, ssigen + 0x0a);
					if ((siigetbyte(slave, ssigen + 0x0d) & 0x02) > 0)
					{
						slavelist[slave].blockLRW = 1;
						slavelist[0].blockLRW++;
					}
					slavelist[slave].Ebuscurrent = siigetbyte(slave, ssigen + 0x0e);
					slavelist[slave].Ebuscurrent += siigetbyte(slave, ssigen + 0x0f) << 8;
					slavelist[0].Ebuscurrent += slavelist[slave].Ebuscurrent;
				}
				/* SII strings section */
				if (siifind(slave, ECT_SII_STRING) > 0)
				{
					siistring(slavelist[slave].name, slave, 1);
				}
				/* no name for slave found, use constructed name */
				else
				{
					sprintf(slavelist[slave].name, "? M:%8.8x I:%8.8x",
						(unsigned int)slavelist[slave].eep_man,
						(unsigned int)slavelist[slave].eep_id);
				}
				/* SII SM section */
				nSM = siiSM(slave, eepSM);
				if (nSM>0)
				{
					slavelist[slave].SM[0].StartAddr = htoes(eepSM->PhStart);
					slavelist[slave].SM[0].SMlength = htoes(eepSM->Plength);
					slavelist[slave].SM[0].SMflags =
						htoel((eepSM->Creg) + (eepSM->Activate << 16));
					SMc = 1;
					while ((SMc < EC_MAXSM) && siiSMnext(slave, eepSM, SMc))
					{
						slavelist[slave].SM[SMc].StartAddr = htoes(eepSM->PhStart);
						slavelist[slave].SM[SMc].SMlength = htoes(eepSM->Plength);
						slavelist[slave].SM[SMc].SMflags =
							htoel((eepSM->Creg) + (eepSM->Activate << 16));
						SMc++;
					}
				}
				/* SII FMMU section */
				if (siiFMMU(slave, eepFMMU))
				{
					if (eepFMMU->FMMU0 != 0xff)
					{
						slavelist[slave].FMMU0func = eepFMMU->FMMU0;
					}
					if (eepFMMU->FMMU1 != 0xff)
					{
						slavelist[slave].FMMU1func = eepFMMU->FMMU1;
					}
					if (eepFMMU->FMMU2 != 0xff)
					{
						slavelist[slave].FMMU2func = eepFMMU->FMMU2;
					}
					if (eepFMMU->FMMU3 != 0xff)
					{
						slavelist[slave].FMMU3func = eepFMMU->FMMU3;
					}
				}
			}

			if (slavelist[slave].mbx_l > 0)
			{
				if (slavelist[slave].SM[0].StartAddr == 0x0000) /* should never happen */
				{
					EC_PRINT("Slave %d has no proper mailbox in configuration, try default.\n", slave);
					slavelist[slave].SM[0].StartAddr = htoes(0x1000);
					slavelist[slave].SM[0].SMlength = htoes(0x0080);
					slavelist[slave].SM[0].SMflags = EtherCATConfig::getDefaultMBXSM0();
					slavelist[slave].SMtype[0] = 1;
				}
				if (slavelist[slave].SM[1].StartAddr == 0x0000) /* should never happen */
				{
					EC_PRINT("Slave %d has no proper mailbox out configuration, try default.\n", slave);
					slavelist[slave].SM[1].StartAddr = htoes(0x1080);
					slavelist[slave].SM[1].SMlength = htoes(0x0080);
					slavelist[slave].SM[1].SMflags = EtherCATConfig::getDefaultMBXSM1();
					slavelist[slave].SMtype[1] = 2;
				}
				/* program SM0 mailbox in and SM1 mailbox out for slave */
				/* writing both SM in one datagram will solve timing issue in old NETX */
				port->FPWR(configadr, ECT_REG_SM0, sizeof(ec_smt)* 2,
					&(slavelist[slave].SM[0]), EC_TIMEOUTRET3);
			}
			/* some slaves need eeprom available to PDI in init->preop transition */
			eeprom2pdi(slave);
			/* request pre_op for slave */
			port->FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP | EC_STATE_ACK), EC_TIMEOUTRET3); /* set preop status */
		}
	}
	return wkc;
}

/* If slave has SII mapping and same slave ID done before, use previous mapping.
* This is safe because SII mapping is constant for same slave ID.
*/
int ecx_contextt::lookup_mapping(uint16 slave, int *Osize, int *Isize)
{
	int i, nSM;
	if ((slave > 1) && (*(slavecount) > 0))
	{
		i = 1;
		while (((slavelist[i].eep_man != slavelist[slave].eep_man) ||
			(slavelist[i].eep_id != slavelist[slave].eep_id) ||
			(slavelist[i].eep_rev != slavelist[slave].eep_rev)) &&
			(i < slave))
		{
			i++;
		}
		if (i < slave)
		{
			for (nSM = 0; nSM < EC_MAXSM; nSM++)
			{
				slavelist[slave].SM[nSM].SMlength = slavelist[i].SM[nSM].SMlength;
				slavelist[slave].SMtype[nSM] = slavelist[i].SMtype[nSM];
			}
			*Osize = slavelist[i].Obits;
			*Isize = slavelist[i].Ibits;
			slavelist[slave].Obits = *Osize;
			slavelist[slave].Ibits = *Isize;
			EC_PRINT("Copy mapping slave %d from %d.\n", slave, i);
			return 1;
		}
	}
	return 0;
}

int ecx_contextt::map_coe_soe(uint16 slave)
{
	int Isize, Osize;
	int rval;

	statecheck(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */

	EC_PRINT(" >Slave %d, configadr %x, state %2.2x\n",
		slave, slavelist[slave].configadr, slavelist[slave].state);

	/* execute special slave configuration hook Pre-Op to Safe-OP */
	if (slavelist[slave].PO2SOconfig) /* only if registered */
	{
		slavelist[slave].PO2SOconfig(slave);
	}
	/* if slave not found in configlist find IO mapping in slave self */
	if (!slavelist[slave].configindex)
	{
		Isize = 0;
		Osize = 0;
		if (slavelist[slave].mbx_proto & ECT_MBXPROT_COE) /* has CoE */
		{
			rval = 0;
			if (slavelist[slave].CoEdetails & ECT_COEDET_SDOCA) /* has Complete Access */
			{
				/* read PDO mapping via CoE and use Complete Access */
				rval = readPDOmapCA(slave, &Osize, &Isize);
			}
			if (!rval) /* CA not available or not succeeded */
			{
				/* read PDO mapping via CoE */
				rval = readPDOmap(slave, &Osize, &Isize);
			}
			EC_PRINT("  CoE Osize:%d Isize:%d\n", Osize, Isize);
		}
		if ((!Isize && !Osize) && (slavelist[slave].mbx_proto & ECT_MBXPROT_SOE)) /* has SoE */
		{
			/* read AT / MDT mapping via SoE */
			rval = readIDNmap(slave, &Osize, &Isize);
			slavelist[slave].SM[2].SMlength = htoes((Osize + 7) / 8);
			slavelist[slave].SM[3].SMlength = htoes((Isize + 7) / 8);
			EC_PRINT("  SoE Osize:%d Isize:%d\n", Osize, Isize);
		}
		slavelist[slave].Obits = Osize;
		slavelist[slave].Ibits = Isize;
	}

	return 1;
}

int ecx_contextt::map_sii(uint16 slave)
{
	int Isize, Osize;
	int nSM;
	ec_eepromPDOt eepPDO;

	Osize = slavelist[slave].Obits;
	Isize = slavelist[slave].Ibits;

	if (!Isize && !Osize) /* find PDO in previous slave with same ID */
	{
		(void)lookup_mapping(slave, &Osize, &Isize);
	}
	if (!Isize && !Osize) /* find PDO mapping by SII */
	{
		memset(&eepPDO, 0, sizeof(eepPDO));
		Isize = (int)siiPDO(slave, &eepPDO, 0);
		EC_PRINT("  SII Isize:%d\n", Isize);
		for (nSM = 0; nSM < EC_MAXSM; nSM++)
		{
			if (eepPDO.SMbitsize[nSM] > 0)
			{
				slavelist[slave].SM[nSM].SMlength = htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
				slavelist[slave].SMtype[nSM] = 4;
				EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
			}
		}
		Osize = (int)siiPDO(slave, &eepPDO, 1);
		EC_PRINT("  SII Osize:%d\n", Osize);
		for (nSM = 0; nSM < EC_MAXSM; nSM++)
		{
			if (eepPDO.SMbitsize[nSM] > 0)
			{
				slavelist[slave].SM[nSM].SMlength = htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
				slavelist[slave].SMtype[nSM] = 3;
				EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
			}
		}
	}
	slavelist[slave].Obits = Osize;
	slavelist[slave].Ibits = Isize;
	EC_PRINT("     ISIZE:%d %d OSIZE:%d\n",
		slavelist[slave].Ibits, Isize, slavelist[slave].Obits);

	return 1;
}

int ecx_contextt::map_sm(uint16 slave)
{
	uint16 configadr;
	int nSM;

	configadr = slavelist[slave].configadr;

	EC_PRINT("  SM programming\n");
	if (!slavelist[slave].mbx_l && slavelist[slave].SM[0].StartAddr)
	{
		port->FPWR(configadr, ECT_REG_SM0,
			sizeof(ec_smt), &(slavelist[slave].SM[0]), EC_TIMEOUTRET3);
		EC_PRINT("    SM0 Type:%d StartAddr:%4.4x Flags:%8.8x\n",
			slavelist[slave].SMtype[0],
			slavelist[slave].SM[0].StartAddr,
			slavelist[slave].SM[0].SMflags);
	}
	if (!slavelist[slave].mbx_l && slavelist[slave].SM[1].StartAddr)
	{
		port->FPWR(configadr, ECT_REG_SM1,
			sizeof(ec_smt), &slavelist[slave].SM[1], EC_TIMEOUTRET3);
		EC_PRINT("    SM1 Type:%d StartAddr:%4.4x Flags:%8.8x\n",
			slavelist[slave].SMtype[1],
			slavelist[slave].SM[1].StartAddr,
			slavelist[slave].SM[1].SMflags);
	}
	/* program SM2 to SMx */
	for (nSM = 2; nSM < EC_MAXSM; nSM++)
	{
		if (slavelist[slave].SM[nSM].StartAddr)
		{
			/* check if SM length is zero -> clear enable flag */
			if (slavelist[slave].SM[nSM].SMlength == 0)
			{
				slavelist[slave].SM[nSM].SMflags =
					htoel(etohl(slavelist[slave].SM[nSM].SMflags) & EC_SMENABLEMASK);
			}
			port->FPWR(configadr, ECT_REG_SM0 + (nSM * sizeof(ec_smt)),
				sizeof(ec_smt), &slavelist[slave].SM[nSM], EC_TIMEOUTRET3);
			EC_PRINT("    SM%d Type:%d StartAddr:%4.4x Flags:%8.8x\n", nSM,
				slavelist[slave].SMtype[nSM],
				slavelist[slave].SM[nSM].StartAddr,
				slavelist[slave].SM[nSM].SMflags);
		}
	}
	if (slavelist[slave].Ibits > 7)
	{
		slavelist[slave].Ibytes = (slavelist[slave].Ibits + 7) / 8;
	}
	if (slavelist[slave].Obits > 7)
	{
		slavelist[slave].Obytes = (slavelist[slave].Obits + 7) / 8;
	}

	return 1;
}

/** Map all PDOs in one group of slaves to IOmap.
*
* @param[in]  context    = context struct
* @param[out] pIOmap     = pointer to IOmap
* @param[in]  group      = group to map, 0 = all groups
* @return IOmap size
*/
int ecx_contextt::config_map_group(void *pIOmap, uint8 group)
{
	uint16 configadr;
	int BitCount, ByteCount, FMMUsize, FMMUdone;
	uint16 SMlength, EndAddr;
	uint8 BitPos;
	uint8 SMc, FMMUc;
	uint32 LogAddr = 0;
	uint32 oLogAddr = 0;
	uint32 diff;
	uint16 currentsegment = 0;
	uint32 segmentsize = 0;
	int thrc;

	if ((*(slavecount) > 0) && (group < maxgroup))
	{
		EC_PRINT("ec_config_map_group IOmap:%p group:%d\n", pIOmap, group);
		LogAddr = grouplist[group].logstartaddr;
		oLogAddr = LogAddr;
		BitPos = 0;
		grouplist[group].nsegments = 0;
		grouplist[group].outputsWKC = 0;
		grouplist[group].inputsWKC = 0;
		EtherCATConfig::init_runnning();
		for (uint16 slave = 1; slave <= *(slavecount); slave++)
		{
			if (!group || (group == slavelist[slave].group))
			{
				EtherCATConfig::findCoEandSoEmappingOfSlavesInMultipleThreads(this, *slavecount);
			}
		}
		/* wait for all threads to finish */
		do
		{
			thrc = EtherCATConfig::ecx_get_threadcount();
			if (thrc)
			{
				osal_usleep(1000);
			}
		} while (thrc);
		/* find SII mapping of slave and program SM */
		for (uint16 slave = 1; slave <= *(slavecount); slave++)
		{
			if (!group || (group == slavelist[slave].group))
			{
				map_sii(slave);
				map_sm(slave);
			}
		}

		/* do input mapping of slave and program FMMUs */
		for (uint16 slave = 1; slave <= *(slavecount); slave++)
		{
			configadr = slavelist[slave].configadr;

			if (!group || (group == slavelist[slave].group))
			{
				FMMUc = slavelist[slave].FMMUunused;
				SMc = 0;
				BitCount = 0;
				ByteCount = 0;
				EndAddr = 0;
				FMMUsize = 0;
				FMMUdone = 0;
				/* create output mapping */
				if (slavelist[slave].Obits)
				{
					EC_PRINT("  OUTPUT MAPPING\n");
					/* search for SM that contribute to the output mapping */
					while ((SMc < (EC_MAXSM - 1)) && (FMMUdone < ((slavelist[slave].Obits + 7) / 8)))
					{
						EC_PRINT("    FMMU %d\n", FMMUc);
						while ((SMc < (EC_MAXSM - 1)) && (slavelist[slave].SMtype[SMc] != 3)) SMc++;
						EC_PRINT("      SM%d\n", SMc);
						slavelist[slave].FMMU[FMMUc].PhysStart =
							slavelist[slave].SM[SMc].StartAddr;
						SMlength = etohs(slavelist[slave].SM[SMc].SMlength);
						ByteCount += SMlength;
						BitCount += SMlength * 8;
						EndAddr = etohs(slavelist[slave].SM[SMc].StartAddr) + SMlength;
						while ((BitCount < slavelist[slave].Obits) && (SMc < (EC_MAXSM - 1))) /* more SM for output */
						{
							SMc++;
							while ((SMc < (EC_MAXSM - 1)) && (slavelist[slave].SMtype[SMc] != 3)) SMc++;
							/* if addresses from more SM connect use one FMMU otherwise break up in mutiple FMMU */
							if (etohs(slavelist[slave].SM[SMc].StartAddr) > EndAddr)
							{
								break;
							}
							EC_PRINT("      SM%d\n", SMc);
							SMlength = etohs(slavelist[slave].SM[SMc].SMlength);
							ByteCount += SMlength;
							BitCount += SMlength * 8;
							EndAddr = etohs(slavelist[slave].SM[SMc].StartAddr) + SMlength;
						}

						/* bit oriented slave */
						if (!slavelist[slave].Obytes)
						{
							slavelist[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
							slavelist[slave].FMMU[FMMUc].LogStartbit = BitPos;
							BitPos += slavelist[slave].Obits - 1;
							if (BitPos > 7)
							{
								LogAddr++;
								BitPos -= 8;
							}
							FMMUsize = LogAddr - etohl(slavelist[slave].FMMU[FMMUc].LogStart) + 1;
							slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
							slavelist[slave].FMMU[FMMUc].LogEndbit = BitPos;
							BitPos++;
							if (BitPos > 7)
							{
								LogAddr++;
								BitPos -= 8;
							}
						}
						/* byte oriented slave */
						else
						{
							if (BitPos)
							{
								LogAddr++;
								BitPos = 0;
							}
							slavelist[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
							slavelist[slave].FMMU[FMMUc].LogStartbit = BitPos;
							BitPos = 7;
							FMMUsize = ByteCount;
							if ((FMMUsize + FMMUdone)> (int)slavelist[slave].Obytes)
							{
								FMMUsize = slavelist[slave].Obytes - FMMUdone;
							}
							LogAddr += FMMUsize;
							slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
							slavelist[slave].FMMU[FMMUc].LogEndbit = BitPos;
							BitPos = 0;
						}
						FMMUdone += FMMUsize;
						slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
						slavelist[slave].FMMU[FMMUc].FMMUtype = 2;
						slavelist[slave].FMMU[FMMUc].FMMUactive = 1;
						/* program FMMU for output */
						port->FPWR(configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut)* FMMUc),
							sizeof(ec_fmmut), &(slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);
						grouplist[group].outputsWKC++;
						if (!slavelist[slave].outputs)
						{
							slavelist[slave].outputs =
								(uint8 *)(pIOmap)+etohl(slavelist[slave].FMMU[FMMUc].LogStart);
							slavelist[slave].Ostartbit =
								slavelist[slave].FMMU[FMMUc].LogStartbit;
							EC_PRINT("    slave %d Outputs %p startbit %d\n",
								slave,
								slavelist[slave].outputs,
								slavelist[slave].Ostartbit);
						}
						FMMUc++;
					}
					slavelist[slave].FMMUunused = FMMUc;
					diff = LogAddr - oLogAddr;
					oLogAddr = LogAddr;
					if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
					{
						grouplist[group].IOsegment[currentsegment] = segmentsize;
						if (currentsegment < (EC_MAXIOSEGMENTS - 1))
						{
							currentsegment++;
							segmentsize = diff;
						}
					}
					else
					{
						segmentsize += diff;
					}
				}
			}
		}
		if (BitPos)
		{
			LogAddr++;
			oLogAddr = LogAddr;
			BitPos = 0;
			if ((segmentsize + 1) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
			{
				grouplist[group].IOsegment[currentsegment] = segmentsize;
				if (currentsegment < (EC_MAXIOSEGMENTS - 1))
				{
					currentsegment++;
					segmentsize = 1;
				}
			}
			else
			{
				segmentsize += 1;
			}
		}
		grouplist[group].outputs = static_cast<uint8_t*>(pIOmap);
		grouplist[group].Obytes = LogAddr;
		grouplist[group].nsegments = currentsegment + 1;
		grouplist[group].Isegment = currentsegment;
		grouplist[group].Ioffset = segmentsize;
		if (!group)
		{
			slavelist[0].outputs = static_cast<uint8_t*>(pIOmap);
			slavelist[0].Obytes = LogAddr; /* store output bytes in master record */
		}

		/* do input mapping of slave and program FMMUs */
		for (uint16 slave = 1; slave <= *(slavecount); slave++)
		{
			configadr = slavelist[slave].configadr;
			if (!group || (group == slavelist[slave].group))
			{
				FMMUc = slavelist[slave].FMMUunused;
				if (slavelist[slave].Obits) /* find free FMMU */
				{
					while (slavelist[slave].FMMU[FMMUc].LogStart) FMMUc++;
				}
				SMc = 0;
				BitCount = 0;
				ByteCount = 0;
				EndAddr = 0;
				FMMUsize = 0;
				FMMUdone = 0;
				/* create input mapping */
				if (slavelist[slave].Ibits)
				{
					EC_PRINT(" =Slave %d, INPUT MAPPING\n", slave);
					/* search for SM that contribute to the input mapping */
					while ((SMc < (EC_MAXSM - 1)) && (FMMUdone < ((slavelist[slave].Ibits + 7) / 8)))
					{
						EC_PRINT("    FMMU %d\n", FMMUc);
						while ((SMc < (EC_MAXSM - 1)) && (slavelist[slave].SMtype[SMc] != 4)) SMc++;
						EC_PRINT("      SM%d\n", SMc);
						slavelist[slave].FMMU[FMMUc].PhysStart =
							slavelist[slave].SM[SMc].StartAddr;
						SMlength = etohs(slavelist[slave].SM[SMc].SMlength);
						ByteCount += SMlength;
						BitCount += SMlength * 8;
						EndAddr = etohs(slavelist[slave].SM[SMc].StartAddr) + SMlength;
						while ((BitCount < slavelist[slave].Ibits) && (SMc < (EC_MAXSM - 1))) /* more SM for input */
						{
							SMc++;
							while ((SMc < (EC_MAXSM - 1)) && (slavelist[slave].SMtype[SMc] != 4)) SMc++;
							/* if addresses from more SM connect use one FMMU otherwise break up in mutiple FMMU */
							if (etohs(slavelist[slave].SM[SMc].StartAddr) > EndAddr)
							{
								break;
							}
							EC_PRINT("      SM%d\n", SMc);
							SMlength = etohs(slavelist[slave].SM[SMc].SMlength);
							ByteCount += SMlength;
							BitCount += SMlength * 8;
							EndAddr = etohs(slavelist[slave].SM[SMc].StartAddr) + SMlength;
						}

						/* bit oriented slave */
						if (!slavelist[slave].Ibytes)
						{
							slavelist[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
							slavelist[slave].FMMU[FMMUc].LogStartbit = BitPos;
							BitPos += slavelist[slave].Ibits - 1;
							if (BitPos > 7)
							{
								LogAddr++;
								BitPos -= 8;
							}
							FMMUsize = LogAddr - etohl(slavelist[slave].FMMU[FMMUc].LogStart) + 1;
							slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
							slavelist[slave].FMMU[FMMUc].LogEndbit = BitPos;
							BitPos++;
							if (BitPos > 7)
							{
								LogAddr++;
								BitPos -= 8;
							}
						}
						/* byte oriented slave */
						else
						{
							if (BitPos)
							{
								LogAddr++;
								BitPos = 0;
							}
							slavelist[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
							slavelist[slave].FMMU[FMMUc].LogStartbit = BitPos;
							BitPos = 7;
							FMMUsize = ByteCount;
							if ((FMMUsize + FMMUdone)> (int)slavelist[slave].Ibytes)
							{
								FMMUsize = slavelist[slave].Ibytes - FMMUdone;
							}
							LogAddr += FMMUsize;
							slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
							slavelist[slave].FMMU[FMMUc].LogEndbit = BitPos;
							BitPos = 0;
						}
						FMMUdone += FMMUsize;
						if (slavelist[slave].FMMU[FMMUc].LogLength)
						{
							slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
							slavelist[slave].FMMU[FMMUc].FMMUtype = 1;
							slavelist[slave].FMMU[FMMUc].FMMUactive = 1;
							/* program FMMU for input */
							port->FPWR(configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut)* FMMUc),
								sizeof(ec_fmmut), &(slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);
							/* add one for an input FMMU */
							grouplist[group].inputsWKC++;
						}
						if (!slavelist[slave].inputs)
						{
							slavelist[slave].inputs =
								(uint8 *)(pIOmap)+etohl(slavelist[slave].FMMU[FMMUc].LogStart);
							slavelist[slave].Istartbit =
								slavelist[slave].FMMU[FMMUc].LogStartbit;
							EC_PRINT("    Inputs %p startbit %d\n",
								slavelist[slave].inputs,
								slavelist[slave].Istartbit);
						}
						FMMUc++;
					}
					slavelist[slave].FMMUunused = FMMUc;
					diff = LogAddr - oLogAddr;
					oLogAddr = LogAddr;
					if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
					{
						grouplist[group].IOsegment[currentsegment] = segmentsize;
						if (currentsegment < (EC_MAXIOSEGMENTS - 1))
						{
							currentsegment++;
							segmentsize = diff;
						}
					}
					else
					{
						segmentsize += diff;
					}
				}

				eeprom2pdi(slave); /* set Eeprom control to PDI */
				port->FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP), EC_TIMEOUTRET3); /* set safeop status */

				if (slavelist[slave].blockLRW)
				{
					grouplist[group].blockLRW++;
				}
				grouplist[group].Ebuscurrent += slavelist[slave].Ebuscurrent;
			}
		}
		if (BitPos)
		{
			LogAddr++;
			oLogAddr = LogAddr;
			BitPos = 0;
			if ((segmentsize + 1) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
			{
				grouplist[group].IOsegment[currentsegment] = segmentsize;
				if (currentsegment < (EC_MAXIOSEGMENTS - 1))
				{
					currentsegment++;
					segmentsize = 1;
				}
			}
			else
			{
				segmentsize += 1;
			}
		}
		grouplist[group].IOsegment[currentsegment] = segmentsize;
		grouplist[group].nsegments = currentsegment + 1;
		grouplist[group].inputs = (uint8 *)(pIOmap)+grouplist[group].Obytes;
		grouplist[group].Ibytes = LogAddr - grouplist[group].Obytes;
		if (!group)
		{
			slavelist[0].inputs = (uint8 *)(pIOmap)+slavelist[0].Obytes;
			slavelist[0].Ibytes = LogAddr - slavelist[0].Obytes; /* store input bytes in master record */
		}

		EC_PRINT("IOmapSize %d\n", LogAddr - grouplist[group].logstartaddr);

		return (LogAddr - grouplist[group].logstartaddr);
	}

	return 0;
}

/** Reconfigure slave.
*
* @param[in] context = context struct
* @param[in] slave   = slave to reconfigure
* @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
* @return Slave state
*/
int ecx_contextt::reconfig_slave(uint16 slave, int timeout)
{
	int state, nSM, FMMUc;
	uint16 configadr;

	configadr = slavelist[slave].configadr;
	if (port->FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_INIT), timeout) <= 0)
	{
		return 0;
	}
	state = 0;
	eeprom2pdi(slave); /* set Eeprom control to PDI */
	/* check state change init */
	state = statecheck(slave, EC_STATE_INIT, EC_TIMEOUTSTATE);
	if (state == EC_STATE_INIT)
	{
		/* program all enabled SM */
		for (nSM = 0; nSM < EC_MAXSM; nSM++)
		{
			if (slavelist[slave].SM[nSM].StartAddr)
			{
				port->FPWR(configadr, ECT_REG_SM0 + (nSM * sizeof(ec_smt)),
					sizeof(ec_smt), &slavelist[slave].SM[nSM], timeout);
			}
		}
		port->FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP), timeout);
		state = statecheck(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */
		if (state == EC_STATE_PRE_OP)
		{
			/* execute special slave configuration hook Pre-Op to Safe-OP */
			if (slavelist[slave].PO2SOconfig) /* only if registered */
			{
				slavelist[slave].PO2SOconfig(slave);
			}
			port->FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP), timeout); /* set safeop status */
			state = statecheck(slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE); /* check state change safe-op */
			/* program configured FMMU */
			for (FMMUc = 0; FMMUc < slavelist[slave].FMMUunused; FMMUc++)
			{
				port->FPWR(configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut)* FMMUc),
					sizeof(ec_fmmut), &slavelist[slave].FMMU[FMMUc], timeout);
			}
		}
	}

	return state;
}
#define PORTM0 0x01
#define PORTM1 0x02
#define PORTM2 0x04
#define PORTM3 0x08

/** 1st sync pulse delay in ns here 100ms */
#define SyncDelay       ((int32)100000000)

/**
* Set DC of slave to fire sync0 at CyclTime interval with CyclShift offset.
*
* @param[in]  context        = context struct
* @param [in] slave            Slave number.
* @param [in] act              TRUE = active, FALSE = deactivated
* @param [in] CyclTime         Cycltime in ns.
* @param [in] CyclShift        CyclShift in ns.
*/
void ecx_contextt::dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
	uint8 h, RA;
	uint16 slaveh;
	int64 t, t1;
	int32 tc;

	slaveh = slavelist[slave].configadr;
	RA = 0;

	/* stop cyclic operation, ready for next trigger */
	(void)port->FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
	if (act)
	{
		RA = 1 + 2;    /* act cyclic operation and sync0, sync1 deactivated */
	}
	h = 0;
	(void)port->FPWR(slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
	t1 = 0;
	(void)port->FPRD(slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
	t1 = etohll(t1);

	/* Calculate first trigger time, always a whole multiple of CyclTime rounded up
	plus the shifttime (can be negative)
	This insures best sychronisation between slaves, slaves with the same CyclTime
	will sync at the same moment (you can use CyclShift to shift the sync) */
	if (CyclTime > 0)
	{
		t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift;
	}
	else
	{
		t = t1 + SyncDelay + CyclShift;
		/* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
	}
	t = htoell(t);
	(void)port->FPWR(slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
	tc = htoel(CyclTime);
	(void)port->FPWR(slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
	(void)port->FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */

	// update ec_slave state
	slavelist[slave].DCactive = (uint8)act;
	slavelist[slave].DCshift = CyclShift;
	slavelist[slave].DCcycle = CyclTime;
}

/**
* Set DC of slave to fire sync0 and sync1 at CyclTime interval with CyclShift offset.
*
* @param[in]  context        = context struct
* @param [in] slave            Slave number.
* @param [in] act              TRUE = active, FALSE = deactivated
* @param [in] CyclTime0        Cycltime SYNC0 in ns.
* @param [in] CyclTime1        Cycltime SYNC1 in ns. This time is a delta time in relation to
the SYNC0 fire. If CylcTime1 = 0 then SYNC1 fires a the same time
as SYNC0.
* @param [in] CyclShift        CyclShift in ns.
*/
void ecx_contextt::dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
	uint8 h, RA;
	uint16 slaveh;
	int64 t, t1;
	int32 tc;
	uint32 TrueCyclTime;

	/* Sync1 can be used as a multiple of Sync0, use true cycle time */
	TrueCyclTime = ((CyclTime1 / CyclTime0) + 1) * CyclTime0;

	slaveh = slavelist[slave].configadr;
	RA = 0;

	/* stop cyclic operation, ready for next trigger */
	(void)port->FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
	if (act)
	{
		RA = 1 + 2 + 4;    /* act cyclic operation and sync0 + sync1 */
	}
	h = 0;
	(void)port->FPWR(slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
	t1 = 0;
	(void)port->FPRD(slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
	t1 = etohll(t1);

	/* Calculate first trigger time, always a whole multiple of TrueCyclTime rounded up
	plus the shifttime (can be negative)
	This insures best sychronisation between slaves, slaves with the same CyclTime
	will sync at the same moment (you can use CyclShift to shift the sync) */
	if (CyclTime0 > 0)
	{
		t = ((t1 + SyncDelay) / TrueCyclTime) * TrueCyclTime + TrueCyclTime + CyclShift;
	}
	else
	{
		t = t1 + SyncDelay + CyclShift;
		/* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
	}
	t = htoell(t);
	(void)port->FPWR(slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
	tc = htoel(CyclTime0);
	(void)port->FPWR(slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
	tc = htoel(CyclTime1);
	(void)port->FPWR(slaveh, ECT_REG_DCCYCLE1, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC1 cycle time */
	(void)port->FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */

	// update ec_slave state
	slavelist[slave].DCactive = (uint8)act;
	slavelist[slave].DCshift = CyclShift;
	slavelist[slave].DCcycle = CyclTime0;
}

/* latched port time of slave */
int32 ecx_contextt::porttime(uint16 slave, uint8 port)
{
	int32 ts;
	switch (port)
	{
	case 0:
		ts = slavelist[slave].DCrtA;
		break;
	case 1:
		ts = slavelist[slave].DCrtB;
		break;
	case 2:
		ts = slavelist[slave].DCrtC;
		break;
	case 3:
		ts = slavelist[slave].DCrtD;
		break;
	default:
		ts = 0;
		break;
	}
	return ts;
}

/* calculate previous active port of a slave */
uint8 ecx_contextt::prevport(uint16 slave, uint8 port)
{
	uint8 pport = port;
	uint8 aport = slavelist[slave].activeports;
	switch (port)
	{
	case 0:
		if (aport & PORTM2)
			pport = 2;
		else if (aport & PORTM1)
			pport = 1;
		else if (aport & PORTM3)
			pport = 3;
		break;
	case 1:
		if (aport & PORTM3)
			pport = 3;
		else if (aport & PORTM0)
			pport = 0;
		else if (aport & PORTM2)
			pport = 2;
		break;
	case 2:
		if (aport & PORTM1)
			pport = 1;
		else if (aport & PORTM3)
			pport = 3;
		else if (aport & PORTM0)
			pport = 0;
		break;
	case 3:
		if (aport & PORTM0)
			pport = 0;
		else if (aport & PORTM2)
			pport = 2;
		else if (aport & PORTM1)
			pport = 1;
		break;
	}
	return pport;
}

/* search unconsumed ports in parent, consume and return first open port */
uint8 ecx_contextt::parentport(uint16 parent)
{
	uint8 parentport = 0;
	uint8 b;
	/* search order is important, here 3 - 1 - 2 - 0 */
	b = slavelist[parent].consumedports;
	if (b & PORTM3)
	{
		parentport = 3;
		b &= (uint8)~PORTM3;
	}
	else if (b & PORTM1)
	{
		parentport = 1;
		b &= (uint8)~PORTM1;
	}
	else if (b & PORTM2)
	{
		parentport = 2;
		b &= (uint8)~PORTM2;
	}
	else if (b & PORTM0)
	{
		parentport = 0;
		b &= (uint8)~PORTM0;
	}
	slavelist[parent].consumedports = b;
	return parentport;
}

/**
* Locate DC slaves, measure propagation delays.
*
* @param[in]  context        = context struct
* @return boolean if slaves are found with DC
*/
boolean ecx_contextt::configdc()
{
	uint16 i, slaveh, parent, child;
	uint16 parenthold = 0;
	uint16 prevDCslave = 0;
	int32 ht, dt1, dt2, dt3;
	int64 hrt;
	uint8 entryport;
	int8 nlist;
	int8 plist[4];
	int32 tlist[4];
	ec_timet mastertime;
	uint64 mastertime64;

	slavelist[0].hasdc = FALSE;
	grouplist[0].hasdc = FALSE;
	ht = 0;

	port->BWR(0, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);  /* latch DCrecvTimeA of all slaves */
	mastertime = osal_current_time();
	mastertime.sec -= 946684800UL;  /* EtherCAT uses 2000-01-01 as epoch start instead of 1970-01-01 */
	mastertime64 = (((uint64)mastertime.sec * 1000000) + (uint64)mastertime.usec) * 1000;
	for (i = 1; i <= *(slavecount); i++)
	{
		slavelist[i].consumedports = slavelist[i].activeports;
		if (slavelist[i].hasdc)
		{
			if (!slavelist[0].hasdc)
			{
				slavelist[0].hasdc = TRUE;
				slavelist[0].DCnext = i;
				slavelist[i].DCprevious = 0;
				grouplist[0].hasdc = TRUE;
				grouplist[0].DCnext = i;
			}
			else
			{
				slavelist[prevDCslave].DCnext = i;
				slavelist[i].DCprevious = prevDCslave;
			}
			/* this branch has DC slave so remove parenthold */
			parenthold = 0;
			prevDCslave = i;
			slaveh = slavelist[i].configadr;
			(void)port->FPRD(slaveh, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
			slavelist[i].DCrtA = etohl(ht);
			/* 64bit latched DCrecvTimeA of each specific slave */
			(void)port->FPRD(slaveh, ECT_REG_DCSOF, sizeof(hrt), &hrt, EC_TIMEOUTRET);
			/* use it as offset in order to set local time around 0 + mastertime */
			hrt = htoell(-etohll(hrt) + mastertime64);
			/* save it in the offset register */
			(void)port->FPWR(slaveh, ECT_REG_DCSYSOFFSET, sizeof(hrt), &hrt, EC_TIMEOUTRET);
			(void)port->FPRD(slaveh, ECT_REG_DCTIME1, sizeof(ht), &ht, EC_TIMEOUTRET);
			slavelist[i].DCrtB = etohl(ht);
			(void)port->FPRD(slaveh, ECT_REG_DCTIME2, sizeof(ht), &ht, EC_TIMEOUTRET);
			slavelist[i].DCrtC = etohl(ht);
			(void)port->FPRD(slaveh, ECT_REG_DCTIME3, sizeof(ht), &ht, EC_TIMEOUTRET);
			slavelist[i].DCrtD = etohl(ht);

			/* make list of active ports and their time stamps */
			nlist = 0;
			if (slavelist[i].activeports & PORTM0)
			{
				plist[nlist] = 0;
				tlist[nlist] = slavelist[i].DCrtA;
				nlist++;
			}
			if (slavelist[i].activeports & PORTM3)
			{
				plist[nlist] = 3;
				tlist[nlist] = slavelist[i].DCrtD;
				nlist++;
			}
			if (slavelist[i].activeports & PORTM1)
			{
				plist[nlist] = 1;
				tlist[nlist] = slavelist[i].DCrtB;
				nlist++;
			}
			if (slavelist[i].activeports & PORTM2)
			{
				plist[nlist] = 2;
				tlist[nlist] = slavelist[i].DCrtC;
				nlist++;
			}
			/* entryport is port with the lowest timestamp */
			entryport = 0;
			if ((nlist > 1) && (tlist[1] < tlist[entryport]))
			{
				entryport = 1;
			}
			if ((nlist > 2) && (tlist[2] < tlist[entryport]))
			{
				entryport = 2;
			}
			if ((nlist > 3) && (tlist[3] < tlist[entryport]))
			{
				entryport = 3;
			}
			entryport = plist[entryport];
			slavelist[i].entryport = entryport;
			/* consume entryport from activeports */
			slavelist[i].consumedports &= (uint8)~(1 << entryport);

			/* finding DC parent of current */
			parent = i;
			do
			{
				child = parent;
				parent = slavelist[parent].parent;
			} while (!((parent == 0) || (slavelist[parent].hasdc)));
			/* only calculate propagation delay if slave is not the first */
			if (parent > 0)
			{
				/* find port on parent this slave is connected to */
				slavelist[i].parentport = parentport(parent);
				if (slavelist[parent].topology == 1)
				{
					slavelist[i].parentport = slavelist[parent].entryport;
				}

				dt1 = 0;
				dt2 = 0;
				/* delta time of (parentport - 1) - parentport */
				/* note: order of ports is 0 - 3 - 1 -2 */
				/* non active ports are skipped */
				dt3 = porttime(parent, slavelist[i].parentport) -
					porttime(parent,
					prevport(parent, slavelist[i].parentport));
				/* current slave has children */
				/* those children's delays need to be subtracted */
				if (slavelist[i].topology > 1)
				{
					dt1 = porttime(i,
						prevport(i, slavelist[i].entryport)) -
						porttime(i, slavelist[i].entryport);
				}
				/* we are only interested in positive difference */
				if (dt1 > dt3) dt1 = -dt1;
				/* current slave is not the first child of parent */
				/* previous child's delays need to be added */
				if ((child - parent) > 1)
				{
					dt2 = porttime(parent,
						prevport(parent, slavelist[i].parentport)) -
						porttime(parent, slavelist[parent].entryport);
				}
				if (dt2 < 0) dt2 = -dt2;

				/* calculate current slave delay from delta times */
				/* assumption : forward delay equals return delay */
				slavelist[i].pdelay = ((dt3 - dt1) / 2) + dt2 +
					slavelist[parent].pdelay;
				ht = htoel(slavelist[i].pdelay);
				/* write propagation delay*/
				(void)port->FPWR(slaveh, ECT_REG_DCSYSDELAY, sizeof(ht), &ht, EC_TIMEOUTRET);
			}
		}
		else
		{
			slavelist[i].DCrtA = 0;
			slavelist[i].DCrtB = 0;
			slavelist[i].DCrtC = 0;
			slavelist[i].DCrtD = 0;
			parent = slavelist[i].parent;
			/* if non DC slave found on first position on branch hold root parent */
			if ((parent > 0) && (slavelist[parent].topology > 2))
				parenthold = parent;
			/* if branch has no DC slaves consume port on root parent */
			if (parenthold && (slavelist[i].topology == 1))
			{
				parentport(parenthold);
				parenthold = 0;
			}
		}
	}

	return slavelist[0].hasdc;
}


/** FoE progress hook.
*
* @param[in]  context        = context struct
* @param[in]     hook       = Pointer to hook function.
* @return 1
*/
int ecx_contextt::FOEdefinehook(void *hook)
{
	FOEhook = static_cast<FOEhook_t>(hook);
	return 1;
}

#define EC_MAXFOEDATA 512

/** FOE structure.
* Used for Read, Write, Data, Ack and Error mailbox packets.
*/
PACKED_BEGIN
struct ec_FOEt PACKED
{
	ec_mbxheadert MbxHeader;
	uint8         OpCode;
	uint8         Reserved;
	union
	{
		uint32        Password;
		uint32        PacketNumber;
		uint32        ErrorCode;
	};
	union
	{
		char          FileName[EC_MAXFOEDATA];
		uint8         Data[EC_MAXFOEDATA];
		char          ErrorText[EC_MAXFOEDATA];
	};
};
PACKED_END
/** FoE read, blocking.
*
* @param[in]  context        = context struct
* @param[in]     slave      = Slave number.
* @param[in]     filename   = Filename of file to read.
* @param[in]     password   = password.
* @param[in,out] psize      = Size in bytes of file buffer, returns bytes read from file.
* @param[out]    p          = Pointer to file buffer
* @param[in]     timeout    = Timeout per mailbox cycle in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::FOEread(uint16 slave, char *filename, uint32 password, int *psize, void *p, int timeout)
{
	ec_FOEt *FOEp, *aFOEp;
	int wkc;
	int32 dataread = 0;
	int32 buffersize, packetnumber, prevpacket = 0;
	uint16 fnsize, maxdata, segmentdata;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;
	boolean worktodo;

	buffersize = *psize;
	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
	wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aFOEp = (ec_FOEt *)&MbxIn;
	FOEp = (ec_FOEt *)&MbxOut;
	fnsize = strlen(filename);
	maxdata = slavelist[slave].mbx_l - 12;
	if (fnsize > maxdata)
	{
		fnsize = maxdata;
	}
	FOEp->MbxHeader.length = htoes(0x0006 + fnsize);
	FOEp->MbxHeader.address = htoes(0x0000);
	FOEp->MbxHeader.priority = 0x00;
	/* get new mailbox count value, used as session handle */
	cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
	slavelist[slave].mbx_cnt = cnt;
	FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
	FOEp->OpCode = ECT_FOE_READ;
	FOEp->Password = htoel(password);
	/* copy filename in mailbox */
	memcpy(&FOEp->FileName[0], filename, fnsize);
	/* send FoE request to slave */
	wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
	if (wkc > 0) /* succeeded to place mailbox in slave ? */
	{
		do
		{
			worktodo = FALSE;
			/* clean mailboxbuffer */
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
			if (wkc > 0) /* succeeded to read slave response ? */
			{
				/* slave response should be FoE */
				if ((aFOEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_FOE)
				{
					if (aFOEp->OpCode == ECT_FOE_DATA)
					{
						segmentdata = etohs(aFOEp->MbxHeader.length) - 0x0006;
						packetnumber = etohl(aFOEp->PacketNumber);
						if ((packetnumber == ++prevpacket) && (dataread + segmentdata <= buffersize))
						{
							memcpy(p, &aFOEp->Data[0], segmentdata);
							dataread += segmentdata;
							p = (uint8 *)p + segmentdata;
							if (segmentdata == maxdata)
							{
								worktodo = TRUE;
							}
							FOEp->MbxHeader.length = htoes(0x0006);
							FOEp->MbxHeader.address = htoes(0x0000);
							FOEp->MbxHeader.priority = 0x00;
							/* get new mailbox count value */
							cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
							slavelist[slave].mbx_cnt = cnt;
							FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
							FOEp->OpCode = ECT_FOE_ACK;
							FOEp->PacketNumber = htoel(packetnumber);
							/* send FoE ack to slave */
							wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
							if (wkc <= 0)
							{
								worktodo = FALSE;
							}
							if (FOEhook)
							{
								FOEhook(slave, packetnumber, dataread);
							}
						}
						else
						{
							/* FoE error */
							wkc = -EC_ERR_TYPE_FOE_BUF2SMALL;
						}
					}
					else
					{
						if (aFOEp->OpCode == ECT_FOE_ERROR)
						{
							/* FoE error */
							wkc = -EC_ERR_TYPE_FOE_ERROR;
						}
						else
						{
							/* unexpected mailbox received */
							wkc = -EC_ERR_TYPE_PACKET_ERROR;
						}
					}
				}
				else
				{
					/* unexpected mailbox received */
					wkc = -EC_ERR_TYPE_PACKET_ERROR;
				}
				*psize = dataread;
			}
		} while (worktodo);
	}

	return wkc;
}

/** FoE write, blocking.
*
* @param[in]  context        = context struct
* @param[in]  slave      = Slave number.
* @param[in]  filename   = Filename of file to write.
* @param[in]  password   = password.
* @param[in]  psize      = Size in bytes of file buffer.
* @param[out] p          = Pointer to file buffer
* @param[in]  timeout    = Timeout per mailbox cycle in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::FOEwrite(uint16 slave, char *filename, uint32 password, int psize, void *p, int timeout)
{
	ec_FOEt *FOEp, *aFOEp;
	int wkc;
	int32 packetnumber, sendpacket = 0;
	uint16 fnsize, maxdata;
	int segmentdata;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;
	boolean worktodo, dofinalzero;
	int tsize;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
	wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aFOEp = (ec_FOEt *)&MbxIn;
	FOEp = (ec_FOEt *)&MbxOut;
	dofinalzero = FALSE;
	fnsize = strlen(filename);
	maxdata = slavelist[slave].mbx_l - 12;
	if (fnsize > maxdata)
	{
		fnsize = maxdata;
	}
	FOEp->MbxHeader.length = htoes(0x0006 + fnsize);
	FOEp->MbxHeader.address = htoes(0x0000);
	FOEp->MbxHeader.priority = 0x00;
	/* get new mailbox count value, used as session handle */
	cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
	slavelist[slave].mbx_cnt = cnt;
	FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
	FOEp->OpCode = ECT_FOE_WRITE;
	FOEp->Password = htoel(password);
	/* copy filename in mailbox */
	memcpy(&FOEp->FileName[0], filename, fnsize);
	/* send FoE request to slave */
	wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
	if (wkc > 0) /* succeeded to place mailbox in slave ? */
	{
		do
		{
			worktodo = FALSE;
			/* clean mailboxbuffer */
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
			if (wkc > 0) /* succeeded to read slave response ? */
			{
				/* slave response should be FoE */
				if ((aFOEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_FOE)
				{
					switch (aFOEp->OpCode)
					{
					case ECT_FOE_ACK:
					{
										packetnumber = etohl(aFOEp->PacketNumber);
										if (packetnumber == sendpacket)
										{
											if (FOEhook)
											{
												FOEhook(slave, packetnumber, psize);
											}
											tsize = psize;
											if (tsize > maxdata)
											{
												tsize = maxdata;
											}
											if (tsize || dofinalzero)
											{
												worktodo = TRUE;
												dofinalzero = FALSE;
												segmentdata = tsize;
												psize -= segmentdata;
												/* if last packet was full size, add a zero size packet as final */
												/* EOF is defined as packetsize < full packetsize */
												if (!psize && (segmentdata == maxdata))
												{
													dofinalzero = TRUE;
												}
												FOEp->MbxHeader.length = htoes(0x0006 + segmentdata);
												FOEp->MbxHeader.address = htoes(0x0000);
												FOEp->MbxHeader.priority = 0x00;
												/* get new mailbox count value */
												cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
												slavelist[slave].mbx_cnt = cnt;
												FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
												FOEp->OpCode = ECT_FOE_DATA;
												sendpacket++;
												FOEp->PacketNumber = htoel(sendpacket);
												memcpy(&FOEp->Data[0], p, segmentdata);
												p = (uint8 *)p + segmentdata;
												/* send FoE data to slave */
												wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
												if (wkc <= 0)
												{
													worktodo = FALSE;
												}
											}
										}
										else
										{
											/* FoE error */
											wkc = -EC_ERR_TYPE_FOE_PACKETNUMBER;
										}
										break;
					}
					case ECT_FOE_BUSY:
					{
										 /* resend if data has been send before */
										 /* otherwise ignore */
										 if (sendpacket)
										 {
											 if (!psize)
											 {
												 dofinalzero = TRUE;
											 }
											 psize += segmentdata;
											 p = (uint8 *)p - segmentdata;
											 --sendpacket;
										 }
										 break;
					}
					case ECT_FOE_ERROR:
					{
										  /* FoE error */
										  if (aFOEp->ErrorCode == 0x8001)
										  {
											  wkc = -EC_ERR_TYPE_FOE_FILE_NOTFOUND;
										  }
										  else
										  {
											  wkc = -EC_ERR_TYPE_FOE_ERROR;
										  }
										  break;
					}
					default:
					{
							   /* unexpected mailbox received */
							   wkc = -EC_ERR_TYPE_PACKET_ERROR;
							   break;
					}
					}
				}
				else
				{
					/* unexpected mailbox received */
					wkc = -EC_ERR_TYPE_PACKET_ERROR;
				}
			}
		} while (worktodo);
	}

	return wkc;
}

#define EC_SOE_DATASTATE_B   0x01
#define EC_SOE_NAME_B        0x02
#define EC_SOE_ATTRIBUTE_B   0x04
#define EC_SOE_UNIT_B        0x08
#define EC_SOE_MIN_B         0x10
#define EC_SOE_MAX_B         0x20
#define EC_SOE_VALUE_B       0x40
#define EC_SOE_DEFAULT_B     0x80

#define EC_SOE_MAXNAME       60
#define EC_SOE_MAXMAPPING    64

#define EC_IDN_MDTCONFIG     24
#define EC_IDN_ATCONFIG      16

/** SoE name structure */
PACKED_BEGIN
typedef struct PACKED
{
	/** current length in bytes of list */
	uint16     currentlength;
	/** maximum length in bytes of list */
	uint16     maxlength;
	char       name[EC_SOE_MAXNAME];
} ec_SoEnamet;
PACKED_END

/** SoE list structure */
PACKED_BEGIN
typedef struct PACKED
{
	/** current length in bytes of list */
	uint16     currentlength;
	/** maximum length in bytes of list */
	uint16     maxlength;
	union
	{
		uint8   byte[8];
		uint16  word[4];
		uint32  dword[2];
		uint64  lword[1];
	};
} ec_SoElistt;
PACKED_END

/** SoE IDN mapping structure */
PACKED_BEGIN
typedef struct PACKED
{
	/** current length in bytes of list */
	uint16     currentlength;
	/** maximum length in bytes of list */
	uint16     maxlength;
	uint16     idn[EC_SOE_MAXMAPPING];
} ec_SoEmappingt;
PACKED_END

#define EC_SOE_LENGTH_1         0x00
#define EC_SOE_LENGTH_2         0x01
#define EC_SOE_LENGTH_4         0x02
#define EC_SOE_LENGTH_8         0x03
#define EC_SOE_TYPE_BINARY      0x00
#define EC_SOE_TYPE_UINT        0x01
#define EC_SOE_TYPE_INT         0x02
#define EC_SOE_TYPE_HEX         0x03
#define EC_SOE_TYPE_STRING      0x04
#define EC_SOE_TYPE_IDN         0x05
#define EC_SOE_TYPE_FLOAT       0x06
#define EC_SOE_TYPE_PARAMETER   0x07

/** SoE attribute structure */
PACKED_BEGIN
typedef struct PACKED
{
	/** evaluation factor for display purposes */
	uint32     evafactor : 16;
	/** length of IDN element(s) */
	uint32     length : 2;
	/** IDN is list */
	uint32     list : 1;
	/** IDN is command */
	uint32     command : 1;
	/** datatype */
	uint32     datatype : 3;
	uint32     reserved1 : 1;
	/** decimals to display if float datatype */
	uint32     decimals : 4;
	/** write protected in pre-op */
	uint32     wppreop : 1;
	/** write protected in safe-op */
	uint32     wpsafeop : 1;
	/** write protected in op */
	uint32     wpop : 1;
	uint32     reserved2 : 1;
} ec_SoEattributet;
PACKED_END


#define EC_SOE_MAX_DRIVES 8

/** SoE (Servo over EtherCAT) mailbox structure */
PACKED_BEGIN
typedef struct PACKED
{
	ec_mbxheadert MbxHeader;
	uint8         opCode : 3;
	uint8         incomplete : 1;
	uint8         error : 1;
	uint8         driveNo : 3;
	uint8         elementflags;
	union
	{
		uint16     idn;
		uint16     fragmentsleft;
	};
} ec_SoEt;
PACKED_END

/** SoE read AT and MTD mapping.
*
* SoE has standard indexes defined for mapping. This function
* tries to read them and collect a full input and output mapping size
* of designated slave.
*
* @param[in]  context = context struct
* @param[in]  slave   = Slave number
* @param[out] Osize   = Size in bits of output mapping (MTD) found
* @param[out] Isize   = Size in bits of input mapping (AT) found
* @return >0 if mapping succesful.
*/
int ecx_contextt::readIDNmap(uint16 slave, int *Osize, int *Isize)
{
	int retVal = 0;
	int   wkc;
	int psize;
	int driveNr;
	uint16 entries, itemcount;
	ec_SoEmappingt     SoEmapping;
	ec_SoEattributet   SoEattribute;

	*Isize = 0;
	*Osize = 0;
	for (driveNr = 0; driveNr < EC_SOE_MAX_DRIVES; driveNr++)
	{
		psize = sizeof(SoEmapping);
		/* read output mapping via SoE */
		wkc = SoEread(slave, driveNr, EC_SOE_VALUE_B, EC_IDN_MDTCONFIG, &psize, &SoEmapping, EC_TIMEOUTRXM);
		if ((wkc > 0) && (psize >= 4) && ((entries = etohs(SoEmapping.currentlength) / 2) > 0) && (entries <= EC_SOE_MAXMAPPING))
		{
			/* command word (uint16) is always mapped but not in list */
			*Osize = 16;
			for (itemcount = 0; itemcount < entries; itemcount++)
			{
				psize = sizeof(SoEattribute);
				/* read attribute of each IDN in mapping list */
				wkc = SoEread(slave, driveNr, EC_SOE_ATTRIBUTE_B, SoEmapping.idn[itemcount], &psize, &SoEattribute, EC_TIMEOUTRXM);
				if ((wkc > 0) && (!SoEattribute.list))
				{
					/* length : 0 = 8bit, 1 = 16bit .... */
					*Osize += (int)8 << SoEattribute.length;
				}
			}
		}
		psize = sizeof(SoEmapping);
		/* read input mapping via SoE */
		wkc = SoEread(slave, driveNr, EC_SOE_VALUE_B, EC_IDN_ATCONFIG, &psize, &SoEmapping, EC_TIMEOUTRXM);
		if ((wkc > 0) && (psize >= 4) && ((entries = etohs(SoEmapping.currentlength) / 2) > 0) && (entries <= EC_SOE_MAXMAPPING))
		{
			/* status word (uint16) is always mapped but not in list */
			*Isize = 16;
			for (itemcount = 0; itemcount < entries; itemcount++)
			{
				psize = sizeof(SoEattribute);
				/* read attribute of each IDN in mapping list */
				wkc = SoEread(slave, driveNr, EC_SOE_ATTRIBUTE_B, SoEmapping.idn[itemcount], &psize, &SoEattribute, EC_TIMEOUTRXM);
				if ((wkc > 0) && (!SoEattribute.list))
				{
					/* length : 0 = 8bit, 1 = 16bit .... */
					*Isize += (int)8 << SoEattribute.length;
				}
			}
		}
	}

	/* found some I/O bits ? */
	if ((*Isize > 0) || (*Osize > 0))
	{
		retVal = 1;
	}
	return retVal;
}



/** Report SoE error.
*
* @param[in]  context        = context struct
* @param[in]  Slave      = Slave number
* @param[in]  idn        = IDN that generated error
* @param[in]  Error      = Error code, see EtherCAT documentation for list
*/
void ecx_contextt::SoEerror(uint16 Slave, uint16 idn, uint16 Error)
{
	ec_errort Ec;

	memset(&Ec, 0, sizeof(Ec));
	Ec.Time = osal_current_time();
	Ec.Slave = Slave;
	Ec.Index = idn;
	Ec.SubIdx = 0;
	*(ecaterror) = TRUE;
	Ec.Etype = EC_ERR_TYPE_SOE_ERROR;
	Ec.ErrorCode = Error;
	pusherror(&Ec);
}

/** SoE read, blocking.
*
* The IDN object of the selected slave and DriveNo is read. If a response
* is larger than the mailbox size then the response is segmented. The function
* will combine all segments and copy them to the parameter buffer.
*
* @param[in]  context        = context struct
* @param[in]  slave         = Slave number
* @param[in]  driveNo       = Drive number in slave
* @param[in]  elementflags  = Flags to select what properties of IDN are to be transfered.
* @param[in]  idn           = IDN.
* @param[in,out] psize      = Size in bytes of parameter buffer, returns bytes read from SoE.
* @param[out] p             = Pointer to parameter buffer
* @param[in]  timeout       = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::SoEread(uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int *psize, void *p, int timeout)
{
	ec_SoEt *SoEp, *aSoEp;
	uint16 totalsize, framedatasize;
	int wkc;
	uint8 *bp;
	uint8 *mp;
	uint16 *errorcode;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;
	boolean NotLast;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timeout set to 0 */
	wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSoEp = (ec_SoEt *)&MbxIn;
	SoEp = (ec_SoEt *)&MbxOut;
	SoEp->MbxHeader.length = htoes(sizeof(ec_SoEt)-sizeof(ec_mbxheadert));
	SoEp->MbxHeader.address = htoes(0x0000);
	SoEp->MbxHeader.priority = 0x00;
	/* get new mailbox count value, used as session handle */
	cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
	slavelist[slave].mbx_cnt = cnt;
	SoEp->MbxHeader.mbxtype = ECT_MBXT_SOE + (cnt << 4); /* SoE */
	SoEp->opCode = ECT_SOE_READREQ;
	SoEp->incomplete = 0;
	SoEp->error = 0;
	SoEp->driveNo = driveNo;
	SoEp->elementflags = elementflags;
	SoEp->idn = htoes(idn);
	totalsize = 0;
	bp = static_cast<uint8_t*>(p);
	mp = (uint8 *)&MbxIn + sizeof(ec_SoEt);
	NotLast = TRUE;
	/* send SoE request to slave */
	wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
	if (wkc > 0) /* succeeded to place mailbox in slave ? */
	{
		while (NotLast)
		{
			/* clean mailboxbuffer */
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
			if (wkc > 0) /* succeeded to read slave response ? */
			{
				/* slave response should be SoE, ReadRes */
				if (((aSoEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_SOE) &&
					(aSoEp->opCode == ECT_SOE_READRES) &&
					(aSoEp->error == 0) &&
					(aSoEp->driveNo == driveNo) &&
					(aSoEp->elementflags == elementflags))
				{
					framedatasize = etohs(aSoEp->MbxHeader.length) - sizeof(ec_SoEt)+sizeof(ec_mbxheadert);
					totalsize += framedatasize;
					/* Does parameter fit in parameter buffer ? */
					if (totalsize <= *psize)
					{
						/* copy parameter data in parameter buffer */
						memcpy(bp, mp, framedatasize);
						/* increment buffer pointer */
						bp += framedatasize;
					}
					else
					{
						framedatasize -= totalsize - *psize;
						totalsize = *psize;
						/* copy parameter data in parameter buffer */
						if (framedatasize > 0) memcpy(bp, mp, framedatasize);
					}

					if (!aSoEp->incomplete)
					{
						NotLast = FALSE;
						*psize = totalsize;
					}
				}
				/* other slave response */
				else
				{
					NotLast = FALSE;
					if (((aSoEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_SOE) &&
						(aSoEp->opCode == ECT_SOE_READRES) &&
						(aSoEp->error == 1))
					{
						mp = (uint8 *)&MbxIn + (etohs(aSoEp->MbxHeader.length) + sizeof(ec_mbxheadert)-sizeof(uint16));
						errorcode = (uint16 *)mp;
						SoEerror(slave, idn, *errorcode);
					}
					else
					{
						packeterror(slave, idn, 0, 1); /* Unexpected frame returned */
					}
					wkc = 0;
				}
			}
			else
			{
				NotLast = FALSE;
				packeterror(slave, idn, 0, 4); /* no response */
			}
		}
	}
	return wkc;
}

/** SoE write, blocking.
*
* The IDN object of the selected slave and DriveNo is written. If a response
* is larger than the mailbox size then the response is segmented.
*
* @param[in]  context        = context struct
* @param[in]  slave         = Slave number
* @param[in]  driveNo       = Drive number in slave
* @param[in]  elementflags  = Flags to select what properties of IDN are to be transfered.
* @param[in]  idn           = IDN.
* @param[in]  psize         = Size in bytes of parameter buffer.
* @param[out] p             = Pointer to parameter buffer
* @param[in]  timeout       = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
*/
int ecx_contextt::SoEwrite(uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int psize, void *p, int timeout)
{
	ec_SoEt *SoEp, *aSoEp;
	uint16 framedatasize, maxdata;
	int wkc;
	uint8 *mp;
	uint8 *hp;
	uint16 *errorcode;
	ec_mbxbuft MbxIn, MbxOut;
	uint8 cnt;
	boolean NotLast;

	ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timeout set to 0 */
	wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
	ec_clearmbx(&MbxOut);
	aSoEp = (ec_SoEt *)&MbxIn;
	SoEp = (ec_SoEt *)&MbxOut;
	SoEp->MbxHeader.address = htoes(0x0000);
	SoEp->MbxHeader.priority = 0x00;
	SoEp->opCode = ECT_SOE_WRITEREQ;
	SoEp->error = 0;
	SoEp->driveNo = driveNo;
	SoEp->elementflags = elementflags;
	hp = static_cast<uint8_t*>(p);
	mp = (uint8 *)&MbxOut + sizeof(ec_SoEt);
	maxdata = slavelist[slave].mbx_l - sizeof(ec_SoEt);
	NotLast = TRUE;
	while (NotLast)
	{
		framedatasize = psize;
		NotLast = FALSE;
		SoEp->idn = htoes(idn);
		SoEp->incomplete = 0;
		if (framedatasize > maxdata)
		{
			framedatasize = maxdata;  /*  segmented transfer needed  */
			NotLast = TRUE;
			SoEp->incomplete = 1;
			SoEp->fragmentsleft = psize / maxdata;
		}
		SoEp->MbxHeader.length = htoes(sizeof(ec_SoEt)-sizeof(ec_mbxheadert)+framedatasize);
		/* get new mailbox counter, used for session handle */
		cnt = ec_nextmbxcnt(slavelist[slave].mbx_cnt);
		slavelist[slave].mbx_cnt = cnt;
		SoEp->MbxHeader.mbxtype = ECT_MBXT_SOE + (cnt << 4); /* SoE */
		/* copy parameter data to mailbox */
		memcpy(mp, hp, framedatasize);
		hp += framedatasize;
		psize -= framedatasize;
		/* send SoE request to slave */
		wkc = mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
		if (wkc > 0) /* succeeded to place mailbox in slave ? */
		{
			if (!NotLast || !mbxempty(slave, timeout))
			{
				/* clean mailboxbuffer */
				ec_clearmbx(&MbxIn);
				/* read slave response */
				wkc = mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
				if (wkc > 0) /* succeeded to read slave response ? */
				{
					NotLast = FALSE;
					/* slave response should be SoE, WriteRes */
					if (((aSoEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_SOE) &&
						(aSoEp->opCode == ECT_SOE_WRITERES) &&
						(aSoEp->error == 0) &&
						(aSoEp->driveNo == driveNo) &&
						(aSoEp->elementflags == elementflags))
					{
						/* SoE write succeeded */
					}
					/* other slave response */
					else
					{
						if (((aSoEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_SOE) &&
							(aSoEp->opCode == ECT_SOE_READRES) &&
							(aSoEp->error == 1))
						{
							mp = (uint8 *)&MbxIn + (etohs(aSoEp->MbxHeader.length) + sizeof(ec_mbxheadert)-sizeof(uint16));
							errorcode = (uint16 *)mp;
							SoEerror(slave, idn, *errorcode);
						}
						else
						{
							packeterror(slave, idn, 0, 1); /* Unexpected frame returned */
						}
						wkc = 0;
					}
				}
				else
				{
					packeterror(slave, idn, 0, 4); /* no response */
				}
			}
		}
	}
	return wkc;
}


#define EC_MAXERRORNAME 127

/** SDO error list type definition */
typedef struct
{
	/** Error code returned from SDO */
	uint32        errorcode;
	/** Readable error description */
	char          errordescription[EC_MAXERRORNAME + 1];
} ec_sdoerrorlist_t;

/** AL status code list type definition */
typedef struct
{
	/** AL status code */
	uint16        ALstatuscode;
	/** Readable description */
	char          ALstatuscodedescription[EC_MAXERRORNAME + 1];
} ec_ALstatuscodelist_t;

/** SoE error list type definition */
typedef struct
{
	/** SoE error code */
	uint16        errorcode;
	/** Readable description */
	char          errordescription[EC_MAXERRORNAME + 1];
} ec_soeerrorlist_t;

/** MBX error list type definition */
typedef struct
{
	/** MBX error code */
	uint16              errorcode;
	/** Readable description */
	char                errordescription[EC_MAXERRORNAME + 1];
} ec_mbxerrorlist_t;

char estring[EC_MAXERRORNAME];

/** SDO error list definition */
const ec_sdoerrorlist_t ec_sdoerrorlist[] = {
	{ 0x00000000, "No error" },
	{ 0x05030000, "Toggle bit not changed" },
	{ 0x05040000, "SDO protocol timeout" },
	{ 0x05040001, "Client/Server command specifier not valid or unknown" },
	{ 0x05040005, "Out of memory" },
	{ 0x06010000, "Unsupported access to an object" },
	{ 0x06010001, "Attempt to read to a write only object" },
	{ 0x06010002, "Attempt to write to a read only object" },
	{ 0x06010003, "Subindex can not be written, SI0 must be 0 for write access" },
	{ 0x06010004, "SDO Complete access not supported for variable length objects" },
	{ 0x06010005, "Object length exceeds mailbox size" },
	{ 0x06010006, "Object mapped to RxPDO, SDO download blocked" },
	{ 0x06020000, "The object does not exist in the object directory" },
	{ 0x06040041, "The object can not be mapped into the PDO" },
	{ 0x06040042, "The number and length of the objects to be mapped would exceed the PDO length" },
	{ 0x06040043, "General parameter incompatibility reason" },
	{ 0x06040047, "General internal incompatibility in the device" },
	{ 0x06060000, "Access failed due to a hardware error" },
	{ 0x06070010, "Data type does not match, length of service parameter does not match" },
	{ 0x06070012, "Data type does not match, length of service parameter too high" },
	{ 0x06070013, "Data type does not match, length of service parameter too low" },
	{ 0x06090011, "Subindex does not exist" },
	{ 0x06090030, "Value range of parameter exceeded (only for write access)" },
	{ 0x06090031, "Value of parameter written too high" },
	{ 0x06090032, "Value of parameter written too low" },
	{ 0x06090036, "Maximum value is less than minimum value" },
	{ 0x08000000, "General error" },
	{ 0x08000020, "Data cannot be transferred or stored to the application" },
	{ 0x08000021, "Data cannot be transferred or stored to the application because of local control" },
	{ 0x08000022, "Data cannot be transferred or stored to the application because of the present device state" },
	{ 0x08000023, "Object dictionary dynamic generation fails or no object dictionary is present" },
	{ 0xffffffff, "Unknown" }
};

/** AL status code list definition */
const ec_ALstatuscodelist_t ec_ALstatuscodelist[] = {
	{ 0x0000, "No error" },
	{ 0x0001, "Unspecified error" },
	{ 0x0002, "No memory" },
	{ 0x0011, "Invalid requested state change" },
	{ 0x0012, "Unknown requested state" },
	{ 0x0013, "Bootstrap not supported" },
	{ 0x0014, "No valid firmware" },
	{ 0x0015, "Invalid mailbox configuration" },
	{ 0x0016, "Invalid mailbox configuration" },
	{ 0x0017, "Invalid sync manager configuration" },
	{ 0x0018, "No valid inputs available" },
	{ 0x0019, "No valid outputs" },
	{ 0x001A, "Synchronization error" },
	{ 0x001B, "Sync manager watchdog" },
	{ 0x001C, "Invalid sync Manager types" },
	{ 0x001D, "Invalid output configuration" },
	{ 0x001E, "Invalid input configuration" },
	{ 0x001F, "Invalid watchdog configuration" },
	{ 0x0020, "Slave needs cold start" },
	{ 0x0021, "Slave needs INIT" },
	{ 0x0022, "Slave needs PREOP" },
	{ 0x0023, "Slave needs SAFEOP" },
	{ 0x0024, "Invalid input mapping" },
	{ 0x0025, "Invalid output mapping" },
	{ 0x0026, "Inconsistent settings" },
	{ 0x0027, "Freerun not supported" },
	{ 0x0028, "Synchronisation not supported" },
	{ 0x0029, "Freerun needs 3buffer mode" },
	{ 0x002A, "Background watchdog" },
	{ 0x002B, "No valid Inputs and Outputs" },
	{ 0x002C, "Fatal sync error" },
	{ 0x002D, "No sync error" }, // was "Invalid Output FMMU Configuration"
	{ 0x002E, "Invalid input FMMU configuration" },
	{ 0x0030, "Invalid DC SYNC configuration" },
	{ 0x0031, "Invalid DC latch configuration" },
	{ 0x0032, "PLL error" },
	{ 0x0033, "DC sync IO error" },
	{ 0x0034, "DC sync timeout error" },
	{ 0x0035, "DC invalid sync cycle time" },
	{ 0x0036, "DC invalid sync0 cycle time" },
	{ 0x0037, "DC invalid sync1 cycle time" },
	{ 0x0041, "MBX_AOE" },
	{ 0x0042, "MBX_EOE" },
	{ 0x0043, "MBX_COE" },
	{ 0x0044, "MBX_FOE" },
	{ 0x0045, "MBX_SOE" },
	{ 0x004F, "MBX_VOE" },
	{ 0x0050, "EEPROM no access" },
	{ 0x0051, "EEPROM error" },
	{ 0x0060, "Slave restarted locally" },
	{ 0x0061, "Device identification value updated" },
	{ 0x00f0, "Application controller available" },
	{ 0xffff, "Unknown" }
};

/** SoE error list definition */
const ec_soeerrorlist_t ec_soeerrorlist[] = {
	{ 0x0000, "No error" },
	{ 0x1001, "No IDN" },
	{ 0x1009, "Invalid access to element 1" },
	{ 0x2001, "No Name" },
	{ 0x2002, "Name transmission too short" },
	{ 0x2003, "Name transmission too long" },
	{ 0x2004, "Name cannot be changed (read only)" },
	{ 0x2005, "Name is write-protected at this time" },
	{ 0x3002, "Attribute transmission too short" },
	{ 0x3003, "Attribute transmission too long" },
	{ 0x3004, "Attribute cannot be changed (read only)" },
	{ 0x3005, "Attribute is write-protected at this time" },
	{ 0x4001, "No units" },
	{ 0x4002, "Unit transmission too short" },
	{ 0x4003, "Unit transmission too long" },
	{ 0x4004, "Unit cannot be changed (read only)" },
	{ 0x4005, "Unit is write-protected at this time" },
	{ 0x5001, "No minimum input value" },
	{ 0x5002, "Minimum input value transmission too short" },
	{ 0x5003, "Minimum input value transmission too long" },
	{ 0x5004, "Minimum input value cannot be changed (read only)" },
	{ 0x5005, "Minimum input value is write-protected at this time" },
	{ 0x6001, "No maximum input value" },
	{ 0x6002, "Maximum input value transmission too short" },
	{ 0x6003, "Maximum input value transmission too long" },
	{ 0x6004, "Maximum input value cannot be changed (read only)" },
	{ 0x6005, "Maximum input value is write-protected at this time" },
	{ 0x7002, "Operation data transmission too short" },
	{ 0x7003, "Operation data transmission too long" },
	{ 0x7004, "Operation data cannot be changed (read only)" },
	{ 0x7005, "Operation data is write-protected at this time (state)" },
	{ 0x7006, "Operation data is smaller than the minimum input value" },
	{ 0x7007, "Operation data is smaller than the maximum input value" },
	{ 0x7008, "Invalid operation data:Configured IDN will not be supported" },
	{ 0x7009, "Operation data write protected by a password" },
	{ 0x700A, "Operation data is write protected, it is configured cyclically" },
	{ 0x700B, "Invalid indirect addressing: (e.g., data container, list handling)" },
	{ 0x700C, "Operation data is write protected, due to other settings" },
	{ 0x700D, "Reserved" },
	{ 0x7010, "Procedure command already active" },
	{ 0x7011, "Procedure command not interruptible" },
	{ 0x7012, "Procedure command at this time not executable (state)" },
	{ 0x7013, "Procedure command not executable (invalid or false parameters)" },
	{ 0x7014, "No data state" },
	{ 0x8001, "No default value" },
	{ 0x8002, "Default value transmission too long" },
	{ 0x8004, "Default value cannot be changed, read only" },
	{ 0x800A, "Invalid drive number" },
	{ 0x800A, "General error" },
	{ 0x800A, "No element addressed" },
	{ 0xffff, "Unknown" }
};

/** MBX error list definition */
const ec_mbxerrorlist_t ec_mbxerrorlist[] = {
	{ 0x0000, "No error" },
	{ 0x0001, "Syntax of 6 octet Mailbox Header is wrong" },
	{ 0x0002, "The mailbox protocol is not supported" },
	{ 0x0003, "Channel Field contains wrong value" },
	{ 0x0004, "The service is no supported" },
	{ 0x0005, "Invalid mailbox header" },
	{ 0x0006, "Length of received mailbox data is too short" },
	{ 0x0007, "No more memory in slave" },
	{ 0x0008, "The lenght of data is inconsistent" },
	{ 0xffff, "Unknown" }
};

/** Look up text string that belongs to SDO error code.
*
* @param[in] sdoerrorcode   = SDO error code as defined in EtherCAT protocol
* @return readable string
*/
const char* ec_sdoerror2string(uint32 sdoerrorcode)
{
	int i = 0;

	while ((ec_sdoerrorlist[i].errorcode != 0xffffffffUL) &&
		(ec_sdoerrorlist[i].errorcode != sdoerrorcode))
	{
		i++;
	}

	return ec_sdoerrorlist[i].errordescription;
}

/** Look up text string that belongs to AL status code.
*
* @param[in] ALstatuscode   = AL status code as defined in EtherCAT protocol
* @return readable string
*/
char* ec_ALstatuscode2string(uint16 ALstatuscode)
{
	int i = 0;

	while ((ec_ALstatuscodelist[i].ALstatuscode != 0xffff) &&
		(ec_ALstatuscodelist[i].ALstatuscode != ALstatuscode))
	{
		i++;
	}

	return (char *)ec_ALstatuscodelist[i].ALstatuscodedescription;
}

/** Look up text string that belongs to SoE error code.
*
* @param[in] errorcode   = SoE error code as defined in EtherCAT protocol
* @return readable string
*/
char* ec_soeerror2string(uint16 errorcode)
{
	int i = 0;

	while ((ec_soeerrorlist[i].errorcode != 0xffff) &&
		(ec_soeerrorlist[i].errorcode != errorcode))
	{
		i++;
	}

	return (char *)ec_soeerrorlist[i].errordescription;
}

/** Look up text string that belongs to MBX error code.
*
* @param[in] errorcode   = MBX error code as defined in EtherCAT protocol
* @return readable string
*/
char* ec_mbxerror2string(uint16 errorcode)
{
	int i = 0;

	while ((ec_mbxerrorlist[i].errorcode != 0xffff) &&
		(ec_mbxerrorlist[i].errorcode != errorcode))
	{
		i++;
	}

	return (char *)ec_mbxerrorlist[i].errordescription;
}



/** Look up error in ec_errorlist and convert to text string.
*
* @param[in]  context        = context struct
* @return readable string
*/
char* ecx_contextt::elist2string()
{
	ec_errort Ec;
	char timestr[20];

	if (poperror(&Ec))
	{
		sprintf(timestr, "Time:%12.3f", Ec.Time.sec + (Ec.Time.usec / 1000000.0));
		switch (Ec.Etype)
		{
		case EC_ERR_TYPE_SDO_ERROR:
		{
			sprintf(estring, "%s SDO slave:%d index:%4.4x.%2.2x error:%8.8x %s\n",
				timestr, Ec.Slave, Ec.Index, Ec.SubIdx, (unsigned)Ec.AbortCode, ec_sdoerror2string(Ec.AbortCode));
			break;
		}
		case EC_ERR_TYPE_EMERGENCY:
		{
			sprintf(estring, "%s EMERGENCY slave:%d error:%4.4x\n",
				timestr, Ec.Slave, Ec.ErrorCode);
			break;
		}
		case EC_ERR_TYPE_PACKET_ERROR:
		{
			sprintf(estring, "%s PACKET slave:%d index:%4.4x.%2.2x error:%d\n",
				timestr, Ec.Slave, Ec.Index, Ec.SubIdx, Ec.ErrorCode);
			break;
		}
		case EC_ERR_TYPE_SDOINFO_ERROR:
		{
			sprintf(estring, "%s SDO slave:%d index:%4.4x.%2.2x error:%8.8x %s\n",
				timestr, Ec.Slave, Ec.Index, Ec.SubIdx, (unsigned)Ec.AbortCode, ec_sdoerror2string(Ec.AbortCode));
			break;
		}
		case EC_ERR_TYPE_SOE_ERROR:
		{
			sprintf(estring, "%s SoE slave:%d IDN:%4.4x error:%4.4x %s\n",
				timestr, Ec.Slave, Ec.Index, (unsigned)Ec.AbortCode, ec_soeerror2string(Ec.ErrorCode));
			break;
		}
		case EC_ERR_TYPE_MBX_ERROR:
		{
			sprintf(estring, "%s MBX slave:%d error:%4.4x %s\n",
				timestr, Ec.Slave, Ec.ErrorCode, ec_mbxerror2string(Ec.ErrorCode));
			break;
		}
		default:
		{
			sprintf(estring, "%s error:%8.8x\n",
				timestr, (unsigned)Ec.AbortCode);
			break;
		}
		}
		return (char*)estring;
	}
	else
	{
		return "";
	}
}
#ifdef EC_VER1
char* ec_elist2string(void)
{
	return ecx_contextt::getInstance()->elist2string();
}
#endif

#ifdef EC_VER1
void ec_pusherror(const ec_errort *Ec)
{
	ecx_contextt::getInstance()->pusherror(Ec);
}

boolean ec_poperror(ec_errort *Ec)
{
	return ecx_contextt::getInstance()->poperror(Ec);
}

boolean ec_iserror(void)
{
	return ecx_contextt::getInstance()->iserror();
}

void ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
	ecx_contextt::getInstance()->packeterror(Slave, Index, SubIdx, ErrorCode);
}

/** Initialise lib in single NIC mode
 * @param[in] ifname   = Dev name, f.e. "eth0"
 * @return >0 if OK
 * @see ecx_init
 */
int ec_init(const char * ifname)
{
	return ecx_contextt::getInstance()->init(ifname);
}

/** Initialise lib in redundant NIC mode
 * @param[in]  ifname   = Primary Dev name, f.e. "eth0"
 * @param[in]  if2name  = Secondary Dev name, f.e. "eth1"
 * @return >0 if OK
 * @see ecx_init_redundant
 */
int ec_init_redundant(const char *ifname, char *if2name)
{
	return ecx_contextt::getInstance()->init_redundant(ecx_redportt::getInstance(), ifname, if2name);
}

/** Close lib.
 * @see ecx_close
 */
void ec_close(void)
{
	ecx_contextt::getInstance()->close();
};

/** Read one byte from slave EEPROM via cache.
 *  If the cache location is empty then a read request is made to the slave.
 *  Depending on the slave capabillities the request is 4 or 8 bytes.
 *  @param[in] slave   = slave number
 *  @param[in] address = eeprom address in bytes (slave uses words)
 *  @return requested byte, if not available then 0xff
 * @see ecx_siigetbyte
 */
uint8 ec_siigetbyte(uint16 slave, uint16 address)
{
	return ecx_contextt::getInstance()->siigetbyte(slave, address);
}

/** Find SII section header in slave EEPROM.
 *  @param[in] slave   = slave number
 *  @param[in] cat     = section category
 *  @return byte address of section at section length entry, if not available then 0
 *  @see ecx_siifind
 */
int16 ec_siifind(uint16 slave, uint16 cat)
{
	return ecx_contextt::getInstance()->siifind(slave, cat);
}

/** Get string from SII string section in slave EEPROM.
 *  @param[out] str    = requested string, 0x00 if not found
 *  @param[in]  slave  = slave number
 *  @param[in]  Sn     = string number
 *  @see ecx_siistring
 */
void ec_siistring(char *str, uint16 slave, uint16 Sn)
{
	ecx_contextt::getInstance()->siistring(str, slave, Sn);
}

/** Get FMMU data from SII FMMU section in slave EEPROM.
 *  @param[in]  slave  = slave number
 *  @param[out] FMMU   = FMMU struct from SII, max. 4 FMMU's
 *  @return number of FMMU's defined in section
 *  @see ecx_siiFMMU
 */
uint16 ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU)
{
   return ecx_contextt::getInstance()->siiFMMU(slave, FMMU);
}

/** Get SM data from SII SM section in slave EEPROM.
 *  @param[in]  slave   = slave number
 *  @param[out] SM      = first SM struct from SII
 *  @return number of SM's defined in section
 *  @see ecx_siiSM
 */
uint16 ec_siiSM(uint16 slave, ec_eepromSMt* SM)
{
   return ecx_contextt::getInstance()->siiSM(slave, SM);
}

/** Get next SM data from SII SM section in slave EEPROM.
 *  @param[in]  slave  = slave number
 *  @param[out] SM     = first SM struct from SII
 *  @param[in]  n      = SM number
 *  @return >0 if OK
 *  @see ecx_siiSMnext
 */
uint16 ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n)
{
   return ecx_contextt::getInstance()->siiSMnext(slave, SM, n);
}

/** Get PDO data from SII PDO section in slave EEPROM.
 *  @param[in]  slave  = slave number
 *  @param[out] PDO    = PDO struct from SII
 *  @param[in]  t      = 0=RXPDO 1=TXPDO
 *  @return mapping size in bits of PDO
 *  @see ecx_siiPDO
 */
int ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
   return ecx_contextt::getInstance()->siiPDO(slave, PDO, t);
}

/** Read all slave states in ec_slave.
 * @return lowest state found
 * @see ecx_readstate
 */
int ec_readstate(void)
{
   return ecx_contextt::getInstance()->readstate();
}

/** Write slave state, if slave = 0 then write to all slaves.
 * The function does not check if the actual state is changed.
 * @param[in] slave = Slave number, 0 = master
 * @return 0
 * @see ecx_writestate
 */
int ec_writestate(uint16 slave)
{
   return ecx_contextt::getInstance()->writestate(slave);
}

/** Check actual slave state.
 * This is a blocking function.
 * @param[in] slave       = Slave number, 0 = all slaves
 * @param[in] reqstate    = Requested state
 * @param[in] timeout     = Timout value in us
 * @return Requested state, or found state after timeout.
 * @see ecx_statecheck
 */
uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout)
{
   return ecx_contextt::getInstance()->statecheck(slave, reqstate, timeout);
}

/** Check if IN mailbox of slave is empty.
 * @param[in] slave    = Slave number
 * @param[in] timeout  = Timeout in us
 * @return >0 is success
 * @see ecx_mbxempty
 */
int ec_mbxempty(uint16 slave, int timeout)
{
   return ecx_contextt::getInstance()->mbxempty(slave, timeout);
}

/** Write IN mailbox to slave.
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 * @see ecx_mbxsend
 */
int ec_mbxsend(uint16 slave,ec_mbxbuft *mbx, int timeout)
{
   return ecx_contextt::getInstance()->mbxsend(slave, mbx, timeout);
}

/** Read OUT mailbox from slave.
 * Supports Mailbox Link Layer with repeat requests.
 * @param[in]  slave      = Slave number
 * @param[out] mbx        = Mailbox data
 * @param[in]  timeout    = Timeout in us
 * @return Work counter (>0 is success)
 * @see ecx_mbxreceive
 */
int ec_mbxreceive(uint16 slave, ec_mbxbuft *mbx, int timeout)
{
   return ecx_contextt::getInstance()->mbxreceive(slave, mbx, timeout);
}

/** Dump complete EEPROM data from slave in buffer.
 * @param[in]  slave    = Slave number
 * @param[out] esibuf   = EEPROM data buffer, make sure it is big enough.
 * @see ecx_esidump
 */
void ec_esidump(uint16 slave, uint8 *esibuf)
{
   ecx_contextt::getInstance()->esidump(slave, esibuf);
}

/** Read EEPROM from slave bypassing cache.
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout   = Timeout in us.
 * @return EEPROM data 32bit
 * @see ecx_readeeprom
 */
uint32 ec_readeeprom(uint16 slave, uint16 eeproma, int timeout)
{
   return ecx_contextt::getInstance()->readeeprom(slave, eeproma, timeout);
}

/** Write EEPROM to slave bypassing cache.
 * @param[in] slave     = Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 * @see ecx_writeeeprom
 */
int ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_contextt::getInstance()->writeeeprom(slave, eeproma, data, timeout);
}

/** Set eeprom control to master. Only if set to PDI.
 * @param[in] slave = Slave number
 * @return >0 if OK
 * @see ecx_eeprom2master
 */
int ec_eeprom2master(uint16 slave)
{
   return ecx_contextt::getInstance()->eeprom2master(slave);
}

int ec_eeprom2pdi(uint16 slave)
{
   return ecx_contextt::getInstance()->eeprom2pdi(slave);
}

uint16 ec_eeprom_waitnotbusyAP(uint16 aiadr,uint16 *estat, int timeout)
{
   return ecx_contextt::getInstance()->eeprom_waitnotbusyAP(aiadr, estat, timeout);
}

/** Read EEPROM from slave bypassing cache. APRD method.
 * @param[in] aiadr       = auto increment address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout)
{
   return ecx_contextt::getInstance()->readeepromAP(aiadr, eeproma, timeout);
}

/** Write EEPROM to slave bypassing cache. APWR method.
 * @param[in] aiadr     = configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data      = 16bit data
 * @param[in] timeout   = Timeout in us.
 * @return >0 if OK
 * @see ecx_writeeepromAP
 */
int ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_contextt::getInstance()->writeeepromAP(aiadr, eeproma, data, timeout);
}

uint16 ec_eeprom_waitnotbusyFP(uint16 configadr,uint16 *estat, int timeout)
{
   return ecx_contextt::getInstance()->eeprom_waitnotbusyFP(configadr, estat, timeout);
}

/** Read EEPROM from slave bypassing cache. FPRD method.
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 64bit or 32bit
 * @see ecx_readeepromFP
 */
uint64 ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout)
{
   return ecx_contextt::getInstance()->readeepromFP(configadr, eeproma, timeout);
}

/** Write EEPROM to slave bypassing cache. FPWR method.
 * @param[in] configadr   = configured address of slave
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @param[in] data        = 16bit data
 * @param[in] timeout     = Timeout in us.
 * @return >0 if OK
 * @see ecx_writeeepromFP
 */
int ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout)
{
   return ecx_contextt::getInstance()->writeeepromFP(configadr, eeproma, data, timeout);
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 1, make request to slave.
 * @param[in] slave       = Slave number
 * @param[in] eeproma     = (WORD) Address in the EEPROM
 * @see ecx_readeeprom1
 */
void ec_readeeprom1(uint16 slave, uint16 eeproma)
{
   ecx_contextt::getInstance()->readeeprom1(slave, eeproma);
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 2, actual read from slave.
 * @param[in] slave       = Slave number
 * @param[in] timeout     = Timeout in us.
 * @return EEPROM data 32bit
 * @see ecx_readeeprom2
 */
uint32 ec_readeeprom2(uint16 slave, int timeout)
{
   return ecx_contextt::getInstance()->readeeprom2(slave, timeout);
}

/** Transmit processdata to slaves.
 * Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
 * Both the input and output processdata are transmitted.
 * The outputs with the actual data, the inputs have a placeholder.
 * The inputs are gathered with the receive processdata function.
 * In contrast to the base LRW function this function is non-blocking.
 * If the processdata does not fit in one datagram, multiple are used.
 * In order to recombine the slave response, a stack is used.
 * @param[in]  group          = group number
 * @return >0 if processdata is transmitted.
 * @see ecx_send_processdata_group
 */
int ec_send_processdata_group(uint8 group)
{
   return ecx_contextt::getInstance()->send_processdata_group(group);
}

/** Receive processdata from slaves.
 * Second part from ec_send_processdata().
 * Received datagrams are recombined with the processdata with help from the stack.
 * If a datagram contains input processdata it copies it to the processdata structure.
 * @param[in]  group          = group number
 * @param[in]  timeout        = Timeout in us.
 * @return Work counter.
 * @see ecx_receive_processdata_group
 */
int ec_receive_processdata_group(uint8 group, int timeout)
{
   return ecx_contextt::getInstance()->receive_processdata_group(group, timeout);
}

int ec_send_processdata(void)
{
   return ec_send_processdata_group(0);
}

int ec_receive_processdata(int timeout)
{
   return ec_receive_processdata_group(0, timeout);
}

int ec_SoEread(uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int *psize, void *p, int timeout)
{
	return ecx_contextt::getInstance()->SoEread(slave, driveNo, elementflags, idn, psize, p, timeout);
}

int ec_SoEwrite(uint16 slave, uint8 driveNo, uint8 elementflags, uint16 idn, int psize, void *p, int timeout)
{
	return ecx_contextt::getInstance()->SoEwrite(slave, driveNo, elementflags, idn, psize, p, timeout);
}

int ec_readIDNmap(uint16 slave, int *Osize, int *Isize)
{
	return ecx_contextt::getInstance()->readIDNmap(slave, Osize, Isize);
}

int ec_FOEdefinehook(void *hook)
{
	return ecx_contextt::getInstance()->FOEdefinehook(hook);
}

int ec_FOEread(uint16 slave, char *filename, uint32 password, int *psize, void *p, int timeout)
{
	return ecx_contextt::getInstance()->FOEread(slave, filename, password, psize, p, timeout);
}

int ec_FOEwrite(uint16 slave, char *filename, uint32 password, int psize, void *p, int timeout)
{
	return ecx_contextt::getInstance()->FOEwrite(slave, filename, password, psize, p, timeout);
}
void ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
	ecx_contextt::getInstance()->dcsync0(slave, act, CyclTime, CyclShift);
}

void ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
	ecx_contextt::getInstance()->dcsync01(slave, act, CyclTime0, CyclTime1, CyclShift);
}

boolean ec_configdc(void)
{
	return ecx_contextt::getInstance()->configdc();
}
/** Report SDO error.
*
* @param[in]  Slave      = Slave number
* @param[in]  Index      = Index that generated error
* @param[in]  SubIdx     = Subindex that generated error
* @param[in]  AbortCode  = Abortcode, see EtherCAT documentation for list
* @see ecx_SDOerror
*/
void ec_SDOerror(uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode)
{
	ecx_contextt::getInstance()->SDOerror(Slave, Index, SubIdx, AbortCode);
}

/** CoE SDO read, blocking. Single subindex or Complete Access.
*
* Only a "normal" upload request is issued. If the requested parameter is <= 4bytes
* then a "expedited" response is returned, otherwise a "normal" response. If a "normal"
* response is larger than the mailbox size then the response is segmented. The function
* will combine all segments and copy them to the parameter buffer.
*
* @param[in]  slave      = Slave number
* @param[in]  index      = Index to read
* @param[in]  subindex   = Subindex to read, must be 0 or 1 if CA is used.
* @param[in]  CA         = FALSE = single subindex. TRUE = Complete Access, all subindexes read.
* @param[in,out] psize   = Size in bytes of parameter buffer, returns bytes read from SDO.
* @param[out] p          = Pointer to parameter buffer
* @param[in]  timeout    = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
* @see ecx_SDOread
*/
int ec_SDOread(uint16 slave, uint16 index, uint8 subindex,
	boolean CA, int *psize, void *p, int timeout)
{
	return ecx_contextt::getInstance()->SDOread(slave, index, subindex, CA, psize, p, timeout);
}

/** CoE SDO write, blocking. Single subindex or Complete Access.
*
* A "normal" download request is issued, unless we have
* small data, then a "expedited" transfer is used. If the parameter is larger than
* the mailbox size then the download is segmented. The function will split the
* parameter data in segments and send them to the slave one by one.
*
* @param[in]  Slave      = Slave number
* @param[in]  Index      = Index to write
* @param[in]  SubIndex   = Subindex to write, must be 0 or 1 if CA is used.
* @param[in]  CA         = FALSE = single subindex. TRUE = Complete Access, all subindexes written.
* @param[in]  psize      = Size in bytes of parameter buffer.
* @param[out] p          = Pointer to parameter buffer
* @param[in]  Timeout    = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
* @see ecx_SDOwrite
*/
int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex,
	boolean CA, int psize, void *p, int Timeout)
{
	return ecx_contextt::getInstance()->SDOwrite(Slave, Index, SubIndex, CA, psize, p, Timeout);
}

/** CoE RxPDO write, blocking.
*
* A RxPDO download request is issued.
*
* @param[in]  Slave         = Slave number
* @param[in]  RxPDOnumber   = Related RxPDO number
* @param[in]  psize         = Size in bytes of PDO buffer.
* @param[out] p             = Pointer to PDO buffer
* @return Workcounter from last slave response
* @see ecx_RxPDO
*/
int ec_RxPDO(uint16 Slave, uint16 RxPDOnumber, int psize, void *p)
{
	return ecx_contextt::getInstance()->RxPDO(Slave, RxPDOnumber, psize, p);
}

/** CoE TxPDO read remote request, blocking.
*
* A RxPDO download request is issued.
*
* @param[in]  slave         = Slave number
* @param[in]  TxPDOnumber   = Related TxPDO number
* @param[in,out] psize      = Size in bytes of PDO buffer, returns bytes read from PDO.
* @param[out] p             = Pointer to PDO buffer
* @param[in]  timeout       = Timeout in us, standard is EC_TIMEOUTRXM
* @return Workcounter from last slave response
* @see ecx_TxPDO
*/
int ec_TxPDO(uint16 slave, uint16 TxPDOnumber, int *psize, void *p, int timeout)
{
	return ecx_contextt::getInstance()->TxPDO(slave, TxPDOnumber, psize, p, timeout);
}

/** Read PDO assign structure
* @param[in]  Slave         = Slave number
* @param[in]  PDOassign     = PDO assign object
* @return total bitlength of PDO assign
*/
int ec_readPDOassign(uint16 Slave, uint16 PDOassign)
{
	return ecx_contextt::getInstance()->readPDOassign(Slave, PDOassign);
}

/** Read PDO assign structure in Complete Access mode
* @param[in]  Slave         = Slave number
* @param[in]  PDOassign     = PDO assign object
* @return total bitlength of PDO assign
* @see ecx_readPDOmap
*/
int ec_readPDOassignCA(uint16 Slave, uint16 PDOassign)
{
	return ecx_contextt::getInstance()->readPDOassignCA(Slave, PDOassign);
}

/** CoE read PDO mapping.
*
* CANopen has standard indexes defined for PDO mapping. This function
* tries to read them and collect a full input and output mapping size
* of designated slave.
*
* For details, see #ecx_readPDOmap
*
* @param[in] Slave    = Slave number
* @param[out] Osize   = Size in bits of output mapping (rxPDO) found
* @param[out] Isize   = Size in bits of input mapping (txPDO) found
* @return >0 if mapping succesful.
*/
int ec_readPDOmap(uint16 Slave, int *Osize, int *Isize)
{
	return ecx_contextt::getInstance()->readPDOmap(Slave, Osize, Isize);
}

/** CoE read PDO mapping in Complete Access mode (CA).
*
* CANopen has standard indexes defined for PDO mapping. This function
* tries to read them and collect a full input and output mapping size
* of designated slave. Slave has to support CA, otherwise use ec_readPDOmap().
*
* @param[in] Slave    = Slave number
* @param[out] Osize   = Size in bits of output mapping (rxPDO) found
* @param[out] Isize   = Size in bits of input mapping (txPDO) found
* @return >0 if mapping succesful.
* @see ecx_readPDOmap ec_readPDOmapCA
*/
int ec_readPDOmapCA(uint16 Slave, int *Osize, int *Isize)
{
	return ecx_contextt::getInstance()->readPDOmapCA(Slave, Osize, Isize);
}

/** CoE read Object Description List.
*
* @param[in] Slave      = Slave number.
* @param[out] pODlist  = resulting Object Description list.
* @return Workcounter of slave response.
* @see ecx_readODlist
*/
int ec_readODlist(uint16 Slave, ec_ODlistt *pODlist)
{
	return ecx_contextt::getInstance()->readODlist(Slave, pODlist);
}

/** CoE read Object Description. Adds textual description to object indexes.
*
* @param[in] Item           = Item number in ODlist.
* @param[in,out] pODlist    = referencing Object Description list.
* @return Workcounter of slave response.
* @see ecx_readODdescription
*/
int ec_readODdescription(uint16 Item, ec_ODlistt *pODlist)
{
	return ecx_contextt::getInstance()->readODdescription(Item, pODlist);
}

int ec_readOEsingle(uint16 Item, uint8 SubI, ec_ODlistt *pODlist, ec_OElistt *pOElist)
{
	return ecx_contextt::getInstance()->readOEsingle(Item, SubI, pODlist, pOElist);
}

/** CoE read SDO service object entry.
*
* @param[in] Item           = Item in ODlist.
* @param[in] pODlist        = Object description list for reference.
* @param[out] pOElist       = resulting object entry structure.
* @return Workcounter of slave response.
* @see ecx_readOE
*/
int ec_readOE(uint16 Item, ec_ODlistt *pODlist, ec_OElistt *pOElist)
{
	return ecx_contextt::getInstance()->readOE(Item, pODlist, pOElist);
}
#endif
