/*
 * Simple Open EtherCAT Master Library
 *
 * File    : ethercatconfig.c
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
 * Configuration module for EtherCAT master.
 *
 * After successful initialisation with ec_init() or ec_init_redundant()
 * the slaves can be auto configured with this module.
 */

#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatsoe.h"
#include "ethercatconfig.h"

// define if debug printf is needed
//#define EC_DEBUG





struct ecx_mapt_t
{
public:
	static OSAL_THREAD_FUNC ecx_mapper_thread(void *param)
	{
		ecx_mapt_t *maptp = static_cast<ecx_mapt_t*>(param);
		maptp->map_coe_soe();
		maptp->running_ = 0;
	}
	int isRunning()const
	{
		return running_;
	}
	void initRunning()
	{
		running_ = 0;
	}
	void set(ecx_contextt *context, uint16 slave)
	{
		context_ = context;
		slave_ = slave;
		running_ = 1;
	}
private:
	void map_coe_soe()
	{
		context_->map_coe_soe(slave_);
	}
	int running_;
	ecx_contextt *context_;
	uint16 slave_;
} ;



#ifdef EC_VER1
/** Slave configuration structure */
typedef const struct
{
   /** Manufacturer code of slave */
   uint32           man;
   /** ID of slave */
   uint32           id;
   /** Readable name */
   char             name[EC_MAXNAME + 1];
   /** Data type */
   uint8            Dtype;
   /** Input bits */
   uint16            Ibits;
   /** Output bits */
   uint16           Obits;
   /** SyncManager 2 address */
   uint16           SM2a;
   /** SyncManager 2 flags */
   uint32           SM2f;
   /** SyncManager 3 address */
   uint16           SM3a;
   /** SyncManager 3 flags */
   uint32           SM3f;
   /** FMMU 0 activation */
   uint8            FM0ac;
   /** FMMU 1 activation */
   uint8            FM1ac;
} ec_configlist_t;



#include "ethercatconfiglist.h"
#endif

/** standard SM0 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM0  0x00010026
/** standard SM1 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM1  0x00010022
/** standard SM0 flags configuration for digital output slaves */
#define EC_DEFAULTDOSM0   0x00010044


ecx_mapt_t* EtherCATConfig::ecx_mapt_;
OSAL_THREAD_HANDLE* EtherCATConfig::ecx_threadh_;

struct Initializer
{
	Initializer()
	{
		EtherCATConfig::init();
	}

}init = Initializer();


#ifdef EC_VER1
/** Find slave in standard configuration list ec_configlist[]
 *
 * @param[in] man      = manufacturer
 * @param[in] id       = ID
 * @return index in ec_configlist[] when found, otherwise 0
 */
void EtherCATConfig::init()
{
	ecx_mapt_ = new ecx_mapt_t[MAX_MAPT];
	ecx_threadh_ = new OSAL_THREAD_HANDLE[MAX_MAPT];
}
int EtherCATConfig::ec_findconfig(uint32 man, uint32 id)
{
   int i = 0;

   do
   {
      i++;
   } while ( (ec_configlist[i].man != EC_CONFIGEND) &&
           ((ec_configlist[i].man != man) || (ec_configlist[i].id != id)) );
   if (ec_configlist[i].man == EC_CONFIGEND)
   {
      i = 0;
   }
   return i;
}
int EtherCATConfig::find(ec_slavet *csl, uint32 man, uint32 id)
{
	int cindex = ec_findconfig(man, id);
	csl->configindex = cindex;
	/* slave found in configuration table ? */
	if (cindex)
	{
		csl->Dtype = ec_configlist[cindex].Dtype;
		strcpy(csl->name, ec_configlist[cindex].name);
		csl->Ibits = ec_configlist[cindex].Ibits;
		csl->Obits = ec_configlist[cindex].Obits;
		if (csl->Obits)
		{
			csl->FMMU0func = 1;
		}
		if (csl->Ibits)
		{
			csl->FMMU1func = 2;
		}
		csl->FMMU[0].FMMUactive = ec_configlist[cindex].FM0ac;
		csl->FMMU[1].FMMUactive = ec_configlist[cindex].FM1ac;
		csl->SM[2].StartAddr = htoes(ec_configlist[cindex].SM2a);
		csl->SM[2].SMflags = htoel(ec_configlist[cindex].SM2f);
		/* simple (no mailbox) output slave found ? */
		if (csl->Obits && !csl->SM[2].StartAddr)
		{
			csl->SM[0].StartAddr = htoes(0x0f00);
			csl->SM[0].SMlength = htoes((csl->Obits + 7) / 8);
			csl->SM[0].SMflags = htoel(EC_DEFAULTDOSM0);
			csl->FMMU[0].FMMUactive = 1;
			csl->FMMU[0].FMMUtype = 2;
			csl->SMtype[0] = 3;
		}
		/* complex output slave */
		else
		{
			csl->SM[2].SMlength = htoes((csl->Obits + 7) / 8);
			csl->SMtype[2] = 3;
		}
		csl->SM[3].StartAddr = htoes(ec_configlist[cindex].SM3a);
		csl->SM[3].SMflags = htoel(ec_configlist[cindex].SM3f);
		/* simple (no mailbox) input slave found ? */
		if (csl->Ibits && !csl->SM[3].StartAddr)
		{
			csl->SM[1].StartAddr = htoes(0x1000);
			csl->SM[1].SMlength = htoes((csl->Ibits + 7) / 8);
			csl->SM[1].SMflags = htoel(0x00000000);
			csl->FMMU[1].FMMUactive = 1;
			csl->FMMU[1].FMMUtype = 1;
			csl->SMtype[1] = 4;
		}
		/* complex input slave */
		else
		{
			csl->SM[3].SMlength = htoes((csl->Ibits + 7) / 8);
			csl->SMtype[3] = 4;
		}
	}
	return cindex;
}
#endif
uint32 EtherCATConfig::getDefaultMBXSM0()
{
	return htoel(EC_DEFAULTMBXSM0);
}
uint32 EtherCATConfig::getDefaultMBXSM1()
{
	return htoel(EC_DEFAULTMBXSM1);
}
void EtherCATConfig::init_runnning()
{
	for (int thrn = 0; thrn < MAX_MAPT; thrn++)
	{
		ecx_mapt_[thrn].initRunning();
	}
}
int EtherCATConfig::ecx_find_mapt()
{
	int p;
	p = 0;
	while ((p < MAX_MAPT) && ecx_mapt_[p].isRunning())
	{
		p++;
	}
	if (p < MAX_MAPT)
	{
		return p;
	}
	else
	{
		return -1;
	}
}
void EtherCATConfig::findCoEandSoEmappingOfSlavesInMultipleThreads(ecx_contextt* context, uint16 slave)
{
	if (MAX_MAPT <= 1)
	{
		/* serialised version */
		context->map_coe_soe(slave);
	}
	else
	{
		int thrn;
		/* multi-threaded version */
		while ((thrn = ecx_find_mapt()) < 0)
		{
			osal_usleep(1000);
		}
		ecx_mapt_[thrn].set(context, slave);
		osal_thread_create(&(ecx_threadh_[thrn]), 128000,
			&ecx_mapt_t::ecx_mapper_thread, &(ecx_mapt_[thrn]));
	}
}

int EtherCATConfig::ecx_get_threadcount()
{
   int thrc, thrn;
   thrc = 0;
   for(thrn = 0 ; thrn < MAX_MAPT ; thrn++)
   {
      thrc += ecx_mapt_[thrn].isRunning();
   }
   return thrc;
}

/** Recover slave.
 *
 * @param[in] context = context struct
 * @param[in] slave   = slave to recover
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return >0 if successful
 */
int ecx_contextt::recover_slave(uint16 slave, int timeout)
{
   int rval;
   int wkc;
   uint16 ADPh, configadr, readadr;

   rval = 0;
   configadr = slavelist[slave].configadr;
   ADPh = (uint16)(1 - slave);
   /* check if we found another slave than the requested */
   readadr = 0xfffe;
   wkc = port->APRD(ADPh, ECT_REG_STADR, sizeof(readadr), &readadr, timeout);
   /* correct slave found, finished */
   if(readadr == configadr)
   {
       return 1;
   }
   /* only try if no config address*/
   if( (wkc > 0) && (readadr == 0))
   {
      /* clear possible slaves at EC_TEMPNODE */
      port->FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(0) , 0);
      /* set temporary node address of slave */
      if(port->APWRw(ADPh, ECT_REG_STADR, htoes(EC_TEMPNODE) , timeout) <= 0)
      {
         port->FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(0) , 0);
         return 0; /* slave fails to respond */
      }

      slavelist[slave].configadr = EC_TEMPNODE; /* temporary config address */
      eeprom2master(slave); /* set Eeprom control to master */

      /* check if slave is the same as configured before */
      if ((port->FPRDw(EC_TEMPNODE, ECT_REG_ALIAS, timeout) ==
             slavelist[slave].aliasadr) &&
          (readeeprom(slave, ECT_SII_ID, EC_TIMEOUTEEP) ==
             slavelist[slave].eep_id) &&
          (readeeprom(slave, ECT_SII_MANUF, EC_TIMEOUTEEP) ==
             slavelist[slave].eep_man) &&
          (readeeprom(slave, ECT_SII_REV, EC_TIMEOUTEEP) ==
             slavelist[slave].eep_rev))
      {
         rval = port->FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(configadr) , timeout);
         slavelist[slave].configadr = configadr;
      }
      else
      {
         /* slave is not the expected one, remove config address*/
         port->FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(0) , timeout);
         slavelist[slave].configadr = configadr;
      }
   }

   return rval;
}

#ifdef EC_VER1
/** Enumerate and init all slaves.
 *
 * @param[in] usetable     = TRUE when using configtable to init slaves, FALSE otherwise
 * @return Workcounter of slave discover datagram = number of slaves found
 * @see ecx_config_init
 */
int ec_config_init(uint8 usetable)
{
   return ecx_contextt::getInstance()->config_init(usetable);
}

/** Map all PDOs in one group of slaves to IOmap.
 *
 * @param[out] pIOmap     = pointer to IOmap
 * @param[in]  group      = group to map, 0 = all groups
 * @return IOmap size
 * @see ecx_config_map_group
 */
int ec_config_map_group(void *pIOmap, uint8 group)
{
   return ecx_contextt::getInstance()->config_map_group(pIOmap, group);
}

/** Map all PDOs from slaves to IOmap.
 *
 * @param[out] pIOmap     = pointer to IOmap
 * @return IOmap size
 */
int ec_config_map(void *pIOmap)
{
   return ec_config_map_group(pIOmap, 0);
}

/** Enumerate / map and init all slaves.
 *
 * @param[in] usetable    = TRUE when using configtable to init slaves, FALSE otherwise
 * @param[out] pIOmap     = pointer to IOmap
 * @return Workcounter of slave discover datagram = number of slaves found
 */
int ec_config(uint8 usetable, void *pIOmap)
{
   int wkc;
   wkc = ec_config_init(usetable);
   if (wkc)
   {
      ec_config_map(pIOmap);
   }
   return wkc;
}

/** Recover slave.
 *
 * @param[in] slave   = slave to recover
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return >0 if successful
 * @see ecx_recover_slave
 */
int ec_recover_slave(uint16 slave, int timeout)
{
   return ecx_contextt::getInstance()->recover_slave(slave, timeout);
}

/** Reconfigure slave.
 *
 * @param[in] slave   = slave to reconfigure
 * @param[in] timeout = local timeout f.e. EC_TIMEOUTRET3
 * @return Slave state
 * @see ecx_reconfig_slave
 */
int ec_reconfig_slave(uint16 slave, int timeout)
{
   return ecx_contextt::getInstance()->reconfig_slave(slave, timeout);
}
#endif
