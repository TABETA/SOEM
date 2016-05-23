/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
//#include <Mmsystem.h>

#include "osal.h"
#include "ethercat.h"

#define EC_TIMEOUTMON 500

class PDOTester
{
private:
	void hookPO2SO(ec_slavet& slave, int pos)
	{
		// beckhoff EL7031, using ec_slave[].name is not very reliable
		if ((slave.eep_man == 0x00000002) && (slave.eep_id == 0x1b773052))
		{
			printf("Found %s at position %d\n", slave.name, pos);
			// link slave specific setup to preop->safeop hook
			slave.PO2SOconfig = &EL7031setup;
		}
		// Copley Controls EAP, using ec_slave[].name is not very reliable
		if ((slave.eep_man == 0x000000ab) && (slave.eep_id == 0x00000380))
		{
			printf("Found %s at position %d\n", slave.name, pos);
			// link slave specific setup to preop->safeop hook
			slave.PO2SOconfig = &AEPsetup;
		}
	}
	//DWORD WINAPI ecatcheck( LPVOID lpParam )
	static OSAL_THREAD_FUNC ecatcheck(void *lpParam)
	{
		while (1)
		{
			if (inOP_ && ((wkc_ < expectedWKC_) || ec_group[currentgroup_].docheckstate))
			{
				if (needlf_)
				{
					needlf_ = FALSE;
					printf("\n");
				}
				/* one ore more slaves are not responding */
				ec_group[currentgroup_].docheckstate = FALSE;
				ec_readstate();
				for (int slave = 1; slave <= context_->getSlaveCount(); slave++)
				{
					if ((context_->ec_slave(slave).group == currentgroup_) && (context_->ec_slave(slave).state != EC_STATE_OPERATIONAL))
					{
						ec_group[currentgroup_].docheckstate = TRUE;
						if (context_->ec_slave(slave).state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
						{
							printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
							context_->ec_slave(slave).state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
							ec_writestate(slave);
						}
						else if (context_->ec_slave(slave).state == EC_STATE_SAFE_OP)
						{
							printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
							context_->ec_slave(slave).state = EC_STATE_OPERATIONAL;
							ec_writestate(slave);
						}
						else if (context_->ec_slave(slave).state > 0)
						{
							if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
							{
								context_->ec_slave(slave).islost = FALSE;
								printf("MESSAGE : slave %d reconfigured\n", slave);
							}
						}
						else if (!context_->ec_slave(slave).islost)
						{
							/* re-check state */
							ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
							if (!context_->ec_slave(slave).state)
							{
								context_->ec_slave(slave).islost = TRUE;
								printf("ERROR : slave %d lost\n", slave);
							}
						}
					}
					if (context_->ec_slave(slave).islost)
					{
						if (!context_->ec_slave(slave).state)
						{
							if (ec_recover_slave(slave, EC_TIMEOUTMON))
							{
								context_->ec_slave(slave).islost = FALSE;
								printf("MESSAGE : slave %d recovered\n", slave);
							}
						}
						else
						{
							context_->ec_slave(slave).islost = FALSE;
							printf("MESSAGE : slave %d found\n", slave);
						}
					}
				}
				if (!ec_group[currentgroup_].docheckstate)
					printf("OK : all slaves resumed OPERATIONAL.\n");
			}
			osal_usleep(10000);
		}

	}
	static int EL7031setup(uint16 slave)
	{
		int retval;
		uint16 u16val;

		// map velocity
		uint16 map_1c12[4] = { 0x0003, 0x1601, 0x1602, 0x1604 };
		uint16 map_1c13[3] = { 0x0002, 0x1a01, 0x1a03 };

		retval = 0;

		// Set PDO mapping using Complete Access
		// Strange, writing CA works, reading CA doesn't
		// This is a protocol error of the slave.
		retval += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

		// bug in EL7031 old firmware, CompleteAccess for reading is not supported even if the slave says it is.
		ecx_contextt::getInstance()->ec_slave(slave).CoEdetails &= ~ECT_COEDET_SDOCA;

		// set some motor parameters, just as example
		u16val = 1200; // max motor current in mA
		//    retval += ec_SDOwrite(slave, 0x8010, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);
		u16val = 150; // motor coil resistance in 0.01ohm
		//    retval += ec_SDOwrite(slave, 0x8010, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);

		// set other nescessary parameters as needed
		// .....

		while (EcatError) printf("%s", ec_elist2string());

		printf("EL7031 slave %d set, retval = %d\n", slave, retval);
		return 1;
	}
	static int AEPsetup(uint16 slave)
	{
		int retval;
		uint8 u8val;
		uint16 u16val;

		retval = 0;

		u8val = 0;
		retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		u16val = 0x1603;
		retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
		u8val = 1;
		retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

		u8val = 0;
		retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		u16val = 0x1a03;
		retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
		u8val = 1;
		retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

		u8val = 8;
		retval += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

		// set some motor parameters, just as example
		u16val = 1200; // max motor current in mA
		//    retval += ec_SDOwrite(slave, 0x8010, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);
		u16val = 150; // motor coil resistance in 0.01ohm
		//    retval += ec_SDOwrite(slave, 0x8010, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);

		// set other nescessary parameters as needed
		// .....

		while (EcatError) printf("%s", ec_elist2string());

		printf("AEP slave %d set, retval = %d\n", slave, retval);
		return 1;
	}
	/* most basic RT thread for process data, just does IO transfer */
	static void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
	{
		IOmap_[0]++;

		ec_send_processdata();
		wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
		rtcnt_++;
		/* do RT control stuff here */
	}
	void executeInOP()
	{
		printf("Operational state reached for all slaves.\n");
		int wkc_count = 0;
		inOP_ = TRUE;
		int oloop = context_->ec_slave(0).Obytes;
		if ((oloop == 0) && (context_->ec_slave(0).Obits > 0)) oloop = 1;
		oloop = min(oloop, 8);
		int iloop = context_->ec_slave(0).Ibytes;
		if ((iloop == 0) && (context_->ec_slave(0).Ibits > 0)) iloop = 1;
		iloop = min(iloop, 8);
		/* cyclic loop, reads data from RT thread */
		for (int i = 1; i <= 500; i++)
		{
			if (wkc_ >= expectedWKC_)
			{
				printf("Processdata cycle %4d, WKC %d , O:", rtcnt_, wkc_);

				for (int j = 0; j < oloop; j++)
				{
					printf(" %2.2x", *(context_->ec_slave(0).outputs + j));
				}

				printf(" I:");
				for (int j = 0; j < iloop; j++)
				{
					printf(" %2.2x", *(context_->ec_slave(0).inputs + j));
				}
				printf(" T:%lld\r", ec_DCtime);
				needlf_ = TRUE;
			}
			osal_usleep(50000);

		}
		inOP_ = FALSE;
	}
	void checkSlavesNotOP()
	{
		printf("Not all slaves reached operational state.\n");
		ec_readstate();
		for (int i = 1; i <= context_->getSlaveCount(); i++)
		{
			if (context_->ec_slave(i).state != EC_STATE_OPERATIONAL)
			{
				printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
					i, context_->ec_slave(i).state, context_->ec_slave(i).ALstatuscode, ec_ALstatuscode2string(context_->ec_slave(i).ALstatuscode));
			}
		}
	}
public:
	void createThread()const
	{
		osal_thread_create(&thread1_, 128000, &ecatcheck, (void*)&ctime);
	}
	void execute(char *ifname)
	{
		needlf_ = FALSE;
		inOP_ = FALSE;

		printf("Starting simple test\n");
		/* initialise SOEM, bind socket to ifname */
		if (context_->init(ifname))
		{
			printf("ec_init on %s succeeded.\n", ifname);
			/* find and auto-config slaves */

			if (context_->config_init(FALSE) > 0)
			{
				printf("%d slaves found and configured.\n", context_->getSlaveCount());

				if ((context_->getSlaveCount() > 1))
				{
					for (int slc = 1; slc <= context_->getSlaveCount(); slc++)
					{
						hookPO2SO(context_->ec_slave(slc), slc);
					}
				}
				context_->config_map_group(&IOmap_, 0);
				context_->configdc();

				printf("Slaves mapped, state to SAFE_OP.\n");
				/* wait for all slaves to reach SAFE_OP state */
				context_->statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

				//‚±‚±‚Å‚â‚é•K—v‚ ‚é‚Ì‚©H
				//int oloop = ec_slave[0].Obytes;
				//if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
				//oloop = min(oloop, 8);
				//int iloop = ec_slave[0].Ibytes;
				//if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
				//iloop = min(iloop, 8);

				printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

				printf("Request operational state for all slaves\n");
				expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
				printf("Calculated workcounter %d\n", expectedWKC_);
				context_->ec_slave(0).state = EC_STATE_OPERATIONAL;
				/* send one valid process data to make outputs in slaves happy*/
				context_->send_processdata_group(0);
				context_->receive_processdata_group(0, EC_TIMEOUTRET);

				/* start RT thread as periodic MM timer */
				UINT mmResult = timeSetEvent(1, 0, RTthread, 0, TIME_PERIODIC);

				/* request OP state for all slaves */
				context_->writestate(0);
				int chk = 40;
				/* wait for all slaves to reach OP state */
				do
				{
					ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
				} while (chk-- && (context_->ec_slave(0).state != EC_STATE_OPERATIONAL));
				if (context_->ec_slave(0).state == EC_STATE_OPERATIONAL)
				{
					executeInOP();
				}
				else
				{
					checkSlavesNotOP();
				}

				/* stop RT thread */
				timeKillEvent(mmResult);

				printf("\nRequest init state for all slaves\n");
				context_->ec_slave(0).state = EC_STATE_INIT;
				/* request INIT state for all slaves */
				ec_writestate(0);
			}
			else
			{
				printf("No slaves found!\n");
			}
			printf("End simple test, close socket\n");
			/* stop SOEM, close socket */
			ec_close();
		}
		else
		{
			printf("No socket connection on %s\nExcecute as root\n", ifname);
		}
	}

private:
	static ecx_contextt* context_;
	static char IOmap_[4096];
	static OSAL_THREAD_HANDLE thread1_;
	static int expectedWKC_;
	static boolean needlf_;
	static volatile int wkc_;
	static volatile int rtcnt_;
	static boolean inOP_;
	static uint8 currentgroup_;

};
ecx_contextt* PDOTester::context_ = ecx_contextt::getInstance();
char PDOTester::IOmap_[4096];
OSAL_THREAD_HANDLE PDOTester::thread1_;
int PDOTester::expectedWKC_;
boolean PDOTester::needlf_;
volatile int PDOTester::wkc_;
volatile int PDOTester::rtcnt_;
boolean PDOTester::inOP_;
uint8 PDOTester::currentgroup_;


int main(int argc, char *argv[])
{
	ec_adaptert * adapter = NULL;
	printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
	PDOTester tester;
	if (argc > 1)
	{
		/* create thread to handle slave error handling in OP */
		tester.createThread();

		char ifbuf[1024];
		strcpy(ifbuf, argv[1]);
		/* start cyclic part */
		tester.execute(ifbuf);
	}
	else
	{
		printf("Usage: simple_test ifname1\n");
		/* Print the list */
		printf ("Available adapters\n");
		adapter = ec_find_adapters ();
		while (adapter != NULL)
		{
			printf ("Description : %s, Device to use for wpcap: %s\n", adapter->desc,adapter->name);
			adapter = adapter->next;
		}
	}

	printf("End program\n");
	return (0);
}
