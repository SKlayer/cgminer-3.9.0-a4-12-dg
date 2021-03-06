/*
 * Copyright 2013 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 * .
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#ifdef HAVE_CURSES
#include <curses.h>
#endif

#include "bcm2835.h"
#include "logging.h"
#include "miner.h"

#include "spi-context.h"

//#define USE_ST_MCU

#ifdef USE_ST_MCU

//features
#define ST_MCU_EN //using ST MCU to control ASIC
#define A1_PLL_CLOCK_EN //config A1's clip PLL clock

#define API_STATE_EN //support show extra ASIC information
#define A1_SPI_SPEED_EN //support change Master SPI speed
#define A1_DIFFICULTY_EN
#define HW_RESET_EN  // hardware reset to STMCU
#define TEST_PIN_EN  // using test pin to test somethings

#define A1_CHK_CMD_TM //check command timeout

#define A1_TEST_MODE_EN 


#define A1_TEMP_EN //support read temperature from ASIC

#define A1_DELAY_RESET 3000 // 3000 ms
//#define A1_MAX_SPEED 18.6
#define A4_NORMAL_SPEED 2.388

#ifdef HW_RESET_EN
#define ASIC_RESET_PIN	RPI_V2_GPIO_P1_05
#endif

#endif //USE_ST_MCU

#define MAX_POLL_NUM   2 // 2

//#define TEST_DELAY
#define A1_ALL_DELAY 2 // 2 sec
#define MAX_ASIC_BOARD   6

#define MAX_JOB  4
//#define MAX_CORE_NUMS 32
//#define MAX_CORE_NUMS 54	// modified for a2
#define MAX_CORE_NUMS 100	// modified for a4
#define MAX_ASIC_NUMS 26 	// Max ASIC chip in a chain



#define TEST_PIN  	RPI_V2_GPIO_P1_16

extern unsigned int asic_vol_set;

#ifdef ST_MCU_EN
#define ST_MCU_NULL   0
#define ST_MCU_PRE_HEADER 1
static uint8_t stMcu=ST_MCU_NULL;
static int32_t stTempCut=90;
static int32_t stTempRun=35;
static int testMode=-1;
static bool testFirstTime=true;
static unsigned int work_s = 0;
static  struct work *work_old[3]={NULL,NULL,NULL};
static struct work *work_get;
#endif
#define PRE_HEADER  0xb5 // send Pre-header first

//struct work *work_cur[120]={NULL};
unsigned int chain_num = 0;
unsigned char work_get_flag[6];
unsigned long poll_data[6][20]={0};
unsigned char poll_data_start[6][20]={0};
unsigned int hw_reset_count[6]={0};
unsigned int clear_log[6]={0};

uint32_t hw_time[6]={0};
uint32_t hw_count_temp[6]={0};
uint32_t chip_no_work_count[6]={0};
unsigned int bc_reset_count[6]={0};
unsigned long spi_error_count = 0;
unsigned int asic_no_result[6][20]={0};

/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};


static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	if (we == NULL)
		return false;
	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}

/********** chip and chain context structures */
#ifdef A1_TEST_MODE_EN
#define ASIC_BOARD_OK 			0x00000000
//#define NO_ASIC_BOARD 			0x00000010
//#define ASIC_BOARD_TESTING		0x00000001
#define ERROR_CHIP	 			0x00000001
#define ERROR_CORE 				0x00000002
#define ERROR_TEMP_SENSOR	 	0x00000004
#define ERROR_BOARD			 	0x00000008
#define ERROR_OVERHEAD			 	0x00000010


//#define ERROR_OVERHEAT		 	0x00000008

#endif

struct A1_chip {
	int num_cores;
	int last_queued_id;
	struct work *work[MAX_JOB];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;
	int reset_errors;
	//uint8_t reg[6]; //latest update from ASIC
	uint8_t reg[12]; //latest update from ASIC	// modified for a2
#ifdef A1_TEMP_EN	
	int temp; //temperature
#endif	

};
#if 0
struct timeval {
	__kernel_time_t		tv_sec;		/* seconds */
	__kernel_suseconds_t	tv_usec;	/* microseconds */
};
#endif
struct A1_chain {
	struct cgpu_info *cgpu;
	int num_chips;
	int num_cores;
	uint8_t spi_tx[128];
	uint8_t spi_rx[128];
#ifdef A1_PLL_CLOCK_EN 	
	uint32_t regPll;
#endif

	struct spi_ctx *spi_ctx;
	struct A1_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;
#ifdef A1_CHK_CMD_TM
	struct timeval tvStart;
	struct timeval tvEnd;
	struct timeval tvDiff;
#endif
	struct timeval tvScryptLast;
	struct timeval tvScryptCurr;
	struct timeval tvScryptDiff;
	int reset_bc;
	int hashes_times;
	int reinit;
#ifdef A1_TEMP_EN	
	int cutTemp; //cutoff temperature 
	int runTemp; 
	int overheat;
	int shutdown;
#endif
#ifdef A1_TEST_MODE_EN
	uint32_t status;
#endif

	struct work *work_cur[20];
	uint16_t work_start_delay;
	uint8_t get_reg_num[10];
	uint8_t pool_reset_flag;
	uint16_t st_reset_delay;
	
};

enum A1_command {
	A1_BIST_START		= 0x01,
	A1_BIST_COLLECT		= 0x0b,		// modified for a2
	A1_BIST_FIX		= 0x03,
	A1_RESET		= 0x04,
	A1_RESETBC		= 0x05,		// modified for a2
	A1_WRITE_JOB		= 0x07,
	A1_READ_RESULT		= 0x08,
	A1_WRITE_REG		= 0x09,
	A1_READ_REG		= 0x0a,
	A1_READ_REG_RESP	= 0x1a,
	A1_POWER_ON		= 0x02,
	A1_POWER_OFF		= 0x06,
	A1_POWER_RESET 	        = 0x0c,
};

int A1_ConfigA1PLLClock(int optPll);
void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx);
static bool cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg);
static void A1_reinit_device(struct cgpu_info *cgpu);
void A1_reinit(void);	
void A1_flush_work_unlock(struct cgpu_info *cgpu);

/********** temporary helper for hexdumping SPI traffic */
#define DEBUG_HEXDUMP 0
static void hexdump(char *prefix, uint8_t *buff, int len)
{
#if DEBUG_HEXDUMP
	static char line[2048];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0)
			pos += sprintf(pos, "\n\t");
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(LOG_NOTICE, "%s", line);
	//printf("%s\n\t", line);
#endif
}

#ifdef A1_PLL_CLOCK_EN

#define A1_PLL_POSTDIV_MASK		0b11
#define A1_PLL_PREDIV_MASK		0b11111
#define A1_PLL_FBDIV_H_MASK		0b111111111

// Register masks for A1
//Reg32 Upper
#define A1_PLL_POSTDIV							(46-32)	// (2) pll post divider
#define A1_PLL_PREDIV							(41-32)	// (5) pll pre divider
#define A1_PLL_FBDIV_H							(32-32)	// (9) pll fb divider
//Reg32 Lower
#define A1_PLL_PD								25	// (1) PLL Reset
#define A1_PLL_LOCK								24	// (1) PLL Lock status (1: PLL locked , 0: PLL not locked )
#define A1_INTERNAL_SPI_FREQUENCY_CONFIG		18 	// (1) 1:  Internal  SPI  frequency  is System  clock  frequency  divide 64 
													//      0:  Internal  SPI  frequency  is System  clock  frequency  divide 128

#define A1_BACKUP_QUEUE_FULL_FLAG				17	// (1) 1:Backup queue is full , 0: Backup queue empty 
#define A1_ACTIVE_QUEUE_BUSY_FLAG				16	// (1) 1:Current active queue busy, 0:  Current  active  queue  not busy 
#define A1_BACKUP_QUEUE_JOB_ID					12	// (4) Job  id  corresponding  with  the backup queue  
#define A1_ACTIVE_QUEUE_JOB_ID					8	// (4) Job  id  corresponding  with  the current active queue 
#define A1_GOOD_CORE_NO       					0 	// (8) Good cores inside the chip, 32 maximum

typedef enum
{
    	A4_PLL_CLOCK_1600MHz = 0,
    	A4_PLL_CLOCK_1500MHz,
    	A4_PLL_CLOCK_1400MHz,
    	A4_PLL_CLOCK_1300MHz,
	 A4_PLL_CLOCK_1272MHz,
	A4_PLL_CLOCK_1248MHz,
	 A4_PLL_CLOCK_1224MHz,
    	A4_PLL_CLOCK_1200MHz,
	A4_PLL_CLOCK_1176MHz,
	 A4_PLL_CLOCK_1128MHz,
    	A4_PLL_CLOCK_1100MHz,
	 A4_PLL_CLOCK_1074MHz,
	A4_PLL_CLOCK_1050MHz,
	 A4_PLL_CLOCK_1026MHz,
    	A4_PLL_CLOCK_1000MHz,
	 A4_PLL_CLOCK_978MHz,
	A4_PLL_CLOCK_948MHz,
	 A4_PLL_CLOCK_924MHz,
    	A4_PLL_CLOCK_900MHz,
	 A4_PLL_CLOCK_876MHz,
    	A4_PLL_CLOCK_860MHz,
	 A4_PLL_CLOCK_852MHz,
	 A4_PLL_CLOCK_828MHz,
    	A4_PLL_CLOCK_800MHz,
	 A4_PLL_CLOCK_774MHz,
	A4_PLL_CLOCK_750MHz,
	 A4_PLL_CLOCK_726MHz,
    	A4_PLL_CLOCK_700MHz,
	 A4_PLL_CLOCK_675MHz,
	A4_PLL_CLOCK_650MHz,
	A4_PLL_CLOCK_624MHz,
    	A4_PLL_CLOCK_600MHz,
	A4_PLL_CLOCK_576MHz,
	A4_PLL_CLOCK_550MHz,
	A4_PLL_CLOCK_525MHz,
    	A4_PLL_CLOCK_500MHz,
	A4_PLL_CLOCK_450MHz,
	A4_PLL_CLOCK_400MHz,
	A4_PLL_CLOCK_350MHz,
	A4_PLL_CLOCK_300MHz,
	A4_PLL_CLOCK_200MHz,
	A4_PLL_CLOCK_MAX,
} A4_PLL_CLOCK;

static uint8_t A1Pll1=A4_PLL_CLOCK_860MHz;
static uint8_t A1Pll2=A4_PLL_CLOCK_860MHz;
static uint8_t A1Pll3=A4_PLL_CLOCK_860MHz;
static uint8_t A1Pll4=A4_PLL_CLOCK_860MHz;
static uint8_t A1Pll5=A4_PLL_CLOCK_860MHz;
static uint8_t A1Pll6=A4_PLL_CLOCK_860MHz;

struct PLL_Clock {
	uint32_t speedMHz;  	// unit MHz
	uint32_t hastRate;  	// divider 1000
	uint32_t pll_reg;
};

#define A4_PLL(prediv,fbdiv,postdiv) ((prediv<<(89-64))|fbdiv<<(80-64)|0b010<<(77-64)|postdiv<<(70-64))
const struct PLL_Clock PLL_Clk_12Mhz[A4_PLL_CLOCK_MAX]={
	{1600, 1600, A4_PLL(0b00011,0b110010000,0b00)},
	{1500, 1500, A4_PLL(0b00000,0b001111101,0b00)},
	{1400, 1400, A4_PLL(0b00011,0b101011110,0b00)},
	{1300, 1300, A4_PLL(0b00011,0b101000101,0b00)},
	{1272, 1272, A4_PLL(0b00001,0b001101010,0b00)},
	{1248, 1248, A4_PLL(0b00001,0b001101000,0b00)},
	{1224, 1224, A4_PLL(0b00001,0b001100110,0b00)},
//	{1200, 1200, A4_PLL(0b00011,0b100101100,0b00)},
	{1200, 1200, A4_PLL(0b00001,0b001100100,0b00)},
	{1176, 1176, A4_PLL(0b00001,0b001100010,0b00)},
	{1128, 1128, A4_PLL(0b00001,0b001011110,0b00)},
	{1100, 1100, A4_PLL(0b00011,0b100010011,0b00)},
	{1074, 1074, A4_PLL(0b00010,0b010110011,0b00)},
	{1050, 1050, A4_PLL(0b00010,0b010101111,0b00)},
	{1026, 1026, A4_PLL(0b00010,0b010101011,0b00)},
	{1000, 1000, A4_PLL(0b00011,0b011111010,0b00)},
	{978,   978, A4_PLL(0b00010,0b010100011,0b00)},
	{948,   948, A4_PLL(0b00010,0b010011110,0b00)},
	{924,   924, A4_PLL(0b00010,0b010011010,0b00)},
	{900,   900, A4_PLL(0b00011,0b011100001,0b00)},
	{876,   876, A4_PLL(0b00001,0b001001001,0b00)},
	{860,   860, A4_PLL(0b00011,0b011010111,0b00)},
	{852,   852, A4_PLL(0b00001,0b001000111,0b00)},
	{828,   828, A4_PLL(0b00001,0b001000101,0b00)},
	{800,   800, A4_PLL(0b00011,0b011001000,0b00)},
	{774,   774, A4_PLL(0b00010,0b010000001,0b00)},
	{750,   750, A4_PLL(0b00010,0b001111101,0b00)},
	{726,   726, A4_PLL(0b00010,0b001111001,0b00)},
	{700,   700, A4_PLL(0b00011,0b010101111,0b00)},
	{675,   675, A4_PLL(0b00010,0b011100001,0b01)},
	{650,   650, A4_PLL(0b00011,0b101000101,0b01)},
	{624,   624, A4_PLL(0b00001,0b001101000,0b01)},
	{600,   600, A4_PLL(0b00011,0b100101100,0b01)},
	{576,   576, A4_PLL(0b00001,0b001100000,0b01)},
	{550,   550, A4_PLL(0b00011,0b100010011,0b01)},
	{525,   525, A4_PLL(0b00010,0b010101111,0b01)},
	{500,   500, A4_PLL(0b00011,0b011111010,0b01)},
	{450,   450, A4_PLL(0b00001,0b001001011,0b01)},
	{400,   400, A4_PLL(0b00011,0b011001000,0b01)},
	{350,   350, A4_PLL(0b00011,0b010101111,0b01)},
	{300,   300, A4_PLL(0b00001,0b001100100,0b10)},
	{200,   200, A4_PLL(0b00011,0b011001000,0b10)},
};

	
int A1_ConfigA1PLLClock(int optPll)
{
	int i;
	int A1Pll;
	if(optPll>0)
	{
		A1Pll=A4_PLL_CLOCK_860MHz; 
		if(optPll<=PLL_Clk_12Mhz[A4_PLL_CLOCK_200MHz].speedMHz)
		{
			A1Pll=A4_PLL_CLOCK_200MHz; //found
		}
		else
		{
			for(i=1;i<A4_PLL_CLOCK_MAX;i++){
				if((optPll>PLL_Clk_12Mhz[i].speedMHz)&&(optPll<=PLL_Clk_12Mhz[i-1].speedMHz))
				{	
					A1Pll=i-1; //found
					break;
				}
			}
		}

		applog(LOG_NOTICE, "A1 = %d,%d",optPll,A1Pll);
		applog(LOG_NOTICE, "A1 PLL Clock = %dMHz",PLL_Clk_12Mhz[A1Pll].speedMHz);
		}

	return A1Pll;	
}


void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx)
{
	uint8_t i;
	struct A1_chip *chip;
	
	if(pllClkIdx<0||pllClkIdx>A4_PLL_CLOCK_MAX) //out of range
		return;

	a1->regPll=PLL_Clk_12Mhz[pllClkIdx].pll_reg;
 
	if(a1->chips == NULL)
		return;
	
	applog(LOG_NOTICE, "SPI(cs%d) A1_SetA1PLLClock, 0x%08x",a1->spi_ctx->config.gpio_cs, a1->regPll);
	
	chip = &a1->chips[6];
	memcpy(chip->reg,     (uint8_t*)&a1->regPll + 3 ,1);
	memcpy(chip->reg + 1, (uint8_t*)&a1->regPll + 2 ,1);
	memcpy(chip->reg + 2, (uint8_t*)&a1->regPll + 1 ,1);
	memcpy(chip->reg + 3, (uint8_t*)&a1->regPll + 0 ,1);
	chip->reg[8]=pllClkIdx;chip->reg[9]=pllClkIdx;
	printf("pllClkIdx is %d %d\n", chip->reg[8],chip->reg[9]);
	chip->reg[7]=asic_vol_set&0x00ff;chip->reg[6]=(asic_vol_set&0xff00)>>8;
	cmd_WRITE_REG(a1, 0, chip->reg);
	//while(1){}
}
#endif //A1_PLL_CLOCK_EN



/********** upper layer SPI functions */

static bool spi_send_command(struct A1_chain *a1, uint8_t cmd, uint8_t addr,
			     uint8_t *buff, int len)
{
	int tx_len;

if(stMcu==ST_MCU_PRE_HEADER)
{

	memset(a1->spi_tx + 2, 0, len);
	a1->spi_tx[0] = PRE_HEADER;
	a1->spi_tx[1] = PRE_HEADER;

	a1->spi_tx[2] = cmd;
	a1->spi_tx[3] = addr;
	if (len > 0 && buff != NULL)
		memcpy(a1->spi_tx + 4, buff, len);
	tx_len = (4 + len + 1) & ~1;
}
else
{

	memset(a1->spi_tx + 2, 0, len);
	a1->spi_tx[0] = cmd;
	a1->spi_tx[1] = addr;
	if (len > 0 && buff != NULL)
		memcpy(a1->spi_tx + 2, buff, len);
	tx_len = (2 + len + 1) & ~1;

}

	applog(LOG_DEBUG, "Processing command 0x%02x%02x", cmd, addr);
	bool retval = spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len);
	hexdump("TX", a1->spi_tx, tx_len);
	hexdump("RX", a1->spi_rx, tx_len);

	if(retval==false)
		applog(LOG_ERR, "SPI Err:spi_send_command 0x%2x",cmd);

	return retval;
}

static bool spi_poll_result(struct A1_chain *a1, uint8_t cmd,
			    uint8_t chip_id, int len)
{
	int i;
	int pollLen;
	cgsleep_us(500);
	pollLen=MAX_POLL_NUM*a1->num_chips;
	if(pollLen<=0)
		pollLen=MAX_POLL_NUM;
		
	for(i = 0; i < pollLen; i++) {
		bool s = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
		hexdump("TX", a1->spi_tx, 2);
		hexdump("RX", a1->spi_rx, 2);
		if (!s)
		{
			applog(LOG_ERR, "SPI(cs%d) Err:spi_poll_result 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
			return false;
		}
		if (a1->spi_rx[0] == cmd && a1->spi_rx[1] == chip_id) {
			applog(LOG_DEBUG, "Cmd 0x%02x ACK'd", cmd);
			if (len == 0)
				return true;
			s = spi_transfer(a1->spi_ctx, NULL,
					 a1->spi_rx + 2, len);
			hexdump("RX", a1->spi_rx + 2, len);
			if (!s)
			{
				applog(LOG_ERR, "SPI(cs%d) Err:spi_poll_result 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
				return false;
			}
			hexdump("poll_result", a1->spi_rx, len + 2);
			return true;
		}
		cgsleep_us(20);

	}
	applog(LOG_WARNING, "Failure(cs%d)(%d): missing ACK for cmd 0x%02x", a1->spi_ctx->config.gpio_cs,pollLen,cmd);
	return false;
}

static bool spi_poll_result_fast(struct A1_chain *a1, uint8_t cmd,
			    uint8_t chip_id, int len,uint32_t timeout)
{
	int i;
	int pollLen;
	bool firstTime=true;
	pollLen=MAX_POLL_NUM*a1->num_chips;
	if(pollLen<=0)
		pollLen=MAX_POLL_NUM;
		
	for(i = 0; i < pollLen; i++) {
		bool s = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
		hexdump("TX", a1->spi_tx, 2);
		hexdump("RX", a1->spi_rx, 2);
		if (!s)
		{
			applog(LOG_ERR, "SPI(cs%d) Err:spi_poll_result_fast 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
			return false;
		}

		if (a1->spi_rx[0] == 0xff&& a1->spi_rx[1] == 0xff)
			{
				applog(LOG_ERR, "SPI(cs%d) no device",a1->spi_ctx->config.gpio_cs);
				return false;
			}

		if (a1->spi_rx[0] == cmd && a1->spi_rx[1] == chip_id) {
			applog(LOG_DEBUG, "Cmd 0x%02x ACK'd", cmd);
			if (len == 0)
				return true;
			s = spi_transfer(a1->spi_ctx, NULL,
					 a1->spi_rx + 2, len);
			hexdump("RX", a1->spi_rx + 2, len);
			if (!s)
			{
				applog(LOG_ERR, "SPI(cs%d) Err2:spi_poll_result_fast 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
				return false;
			}
			hexdump("poll_result", a1->spi_rx, len + 2);
			return true;
		}
		if(firstTime)
		{
			cgsleep_ms(timeout);
			firstTime=false;
		}
//		cgsleep_ms(10);
//		cgsleep_us(500);
//	cgsleep_us(10);

	}
//	applog(LOG_WARNING, "Failure: missing ACK for cmd 0x%02x", cmd);
	applog(LOG_WARNING, "Failure(cs%d)(%d): missing ACK for cmd 0x%02x", a1->spi_ctx->config.gpio_cs,pollLen,cmd);

	return false;
}

/********** A1 SPI commands */
#ifdef ST_MCU_EN
static char set_flag = 0;
static bool cmd_BIST_START(struct A1_chain *a1)
{
	bool ret;
	unsigned char vol_set[2];
	vol_set[0] = asic_vol_set >> 8;
	vol_set[1] = asic_vol_set&0x00ff;
#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif
	if(set_flag == 0)
	{
		ret=spi_send_command(a1, A1_BIST_START, 0x00, NULL, 0);	
	}
	else 
	{
		set_flag = 1;
		//ret=spi_send_command(a1, A1_BIST_START, 0x00, vol_set, 2);
	}
//	ret=spi_poll_result_fast(a1, A1_BIST_START, 0x00, 2,A1_DELAY_RESET);
	cgsleep_us(50);
	ret=spi_poll_result(a1, A1_BIST_START, 0x00, 2);
	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_BIST_START-%d.%04ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);
	}
	return ret;
}

static bool cmd_BIST_COLLECT_BCAST(struct A1_chain *a1)		// modified for a2
{
	bool ret;

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif


	ret=spi_send_command(a1, A1_BIST_COLLECT, 0x00, NULL, 0);
//	ret=spi_poll_result_fast(a1, A1_BIST_COLLECT, 0x00, 0,A1_DELAY_RESET);
	cgsleep_us(50);
	ret=spi_poll_result(a1, A1_BIST_COLLECT, 0x00, 0);
	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(%d) timeout:cmd_BIST_COLLECT_BCAST-%d.%04ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);
	}

	return ret;
}

static bool cmd_BIST_FIX_BCAST(struct A1_chain *a1)
{
	bool ret;

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif


	ret=spi_send_command(a1, A1_BIST_FIX, 0x00, NULL, 0);
//	ret=spi_poll_result_fast(a1, A1_BIST_FIX, 0x00, 0,A1_DELAY_RESET);
	cgsleep_us(50);
	ret=spi_poll_result(a1, A1_BIST_FIX, 0x00, 0);
	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(%d) timeout:cmd_BIST_FIX_BCAST-%d.%04ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);
	}

	return ret;
}

static bool cmd_RESET_BCAST(struct A1_chain *a1)
{
//	struct timeval tv_start, tv_end,tv_diff;

	bool ret;
#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_RESET, 0x00, NULL, 0);
#ifdef ST_MCU_EN
	if(ret==false)
	{
		return ret;
	}	

#if 0	
	cgtime(&tv_start);

	cgsleep_ms(A1_DELAY_RESET); 
	cgtime(&tv_end);

	timersub(&tv_end, &tv_start, &tv_diff);

	applog(LOG_WARNING, "Reset time %d\n",tv_diff.tv_sec);
#endif	

#endif

	ret=spi_poll_result_fast(a1, A1_RESET, 0x00, 0,A1_DELAY_RESET);
//	ret=spi_poll_result(a1, A1_RESET, 0x00, 0);

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_RESET_BCAST - %.2f ms",a1->spi_ctx->config.gpio_cs, a1->tvDiff.tv_usec/1000.0);		
	}


//	if(ret==false) 	applog(LOG_WARNING, "ACK timeout:cmd_RESET_BCAST\n");
	return ret;
}

static bool cmd_RESETBC_BCAST(struct A1_chain *a1)
{
//	struct timeval tv_start, tv_end,tv_diff;

	bool ret;
#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_RESETBC, 0x00, NULL, 0);
#ifdef ST_MCU_EN
	if(ret==false)
	{
		return ret;
	}	

#if 0	
	cgtime(&tv_start);

	cgtime(&tv_end);

	timersub(&tv_end, &tv_start, &tv_diff);

	applog(LOG_WARNING, "Reset Bc time %d\n",tv_diff.tv_sec);
#endif	

#endif

	cgsleep_us(50);

//	ret=spi_poll_result_fast(a1, A1_RESETBC, 0x00, 0,A1_DELAY_RESET);
	ret=spi_poll_result(a1, A1_RESETBC, 0x00, 0);

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_RESETBC_BCAST-%d.%04ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);		
	}


//	if(ret==false) 	applog(LOG_WARNING, "ACK timeout:cmd_RESETBC_BCAST\n");
	return ret;
}

static bool cmd_POWER_ON_BCAST(struct A1_chain *a1)
{
//	struct timeval tv_start, tv_end,tv_diff;

	bool ret;
#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_POWER_ON, 0x00, NULL, 0);
#ifdef ST_MCU_EN
	if(ret==false)
	{
		return ret;
	}	

#if 0	
	cgtime(&tv_start);

	cgtime(&tv_end);

	timersub(&tv_end, &tv_start, &tv_diff);

	applog(LOG_WARNING, "Power on time %d\n",tv_diff.tv_sec);
#endif	

#endif

	cgsleep_us(50);

//	ret=spi_poll_result_fast(a1, A1_POWER_ON, 0x00, 0,A1_DELAY_RESET);
	ret=spi_poll_result(a1, A1_POWER_ON, 0x00, 0);

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_POWER_ON_BCAST-%d.%04ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);		
	}


//	if(ret==false) 	applog(LOG_WARNING, "ACK timeout:cmd_POWER_ON_BCAST\n");
	return ret;
}

static bool cmd_POWER_OFF_BCAST(struct A1_chain *a1)
{
//	struct timeval tv_start, tv_end,tv_diff;

	bool ret;
#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_POWER_OFF, 0x00, NULL, 0);
#ifdef ST_MCU_EN
	if(ret==false)
	{
		return ret;
	}	

#if 0	
	cgtime(&tv_start);

	cgtime(&tv_end);

	timersub(&tv_end, &tv_start, &tv_diff);

	applog(LOG_WARNING, "Power off time %d\n",tv_diff.tv_sec);
#endif	

#endif

	cgsleep_us(50);

//	ret=spi_poll_result_fast(a1, A1_POWER_OFF, 0x00, 0,A1_DELAY_RESET);
	ret=spi_poll_result(a1, A1_POWER_OFF, 0x00, 0);

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_POWER_OFF_BCAST-%d.%04ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);		
	}


//	if(ret==false) 	applog(LOG_WARNING, "ACK timeout:cmd_POWER_OFF_BCAST\n");
	return ret;
}



static bool cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg)
{
	bool ret;

#if 0 //patch A
	return	spi_send_command(a1, A1_WRITE_REG, chip, reg, 6)&&
		spi_poll_result(a1, A1_WRITE_REG, chip, 6);
#else
	//ret=spi_send_command(a1, A1_WRITE_REG, chip, reg, 6);
	ret=spi_send_command(a1, A1_WRITE_REG, chip, reg, 12);	// modified for a2
	cgsleep_us(50);
	spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
        hexdump("RX", a1->spi_rx, 2);
#endif
	return ret;
}

static bool cmd_READ_REG(struct A1_chain *a1, uint8_t chip)
{
	bool ret;
#ifdef TEST_PIN_EN
	bcm2835_gpio_fsel(TEST_PIN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(TEST_PIN, HIGH);
#endif

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_READ_REG, chip, NULL, 0);
	cgsleep_us(50);
#ifdef A1_TEMP_EN	
	//ret=spi_poll_result(a1, A1_READ_REG_RESP, chip, 8);
	ret=spi_poll_result(a1, A1_READ_REG_RESP, chip, 14);	// modified for a2
#else
	//ret=spi_poll_result(a1, A1_READ_REG_RESP, chip, 6);
	ret=spi_poll_result(a1, A1_READ_REG_RESP, chip, 12);	// modified for a2
#endif	

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_READ_REG-%d.%ds",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_sec,a1->tvDiff.tv_usec);
	}
#ifdef TEST_PIN_EN
	bcm2835_gpio_fsel(TEST_PIN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(TEST_PIN, LOW);
#endif


	return ret;
}

#else

static bool cmd_BIST_START(struct A1_chain *a1)
{
	return	spi_send_command(a1, A1_BIST_START, 0x00, NULL, 0) &&
		spi_poll_result(a1, A1_BIST_START, 0x00, 2);
}

static bool cmd_BIST_FIX_BCAST(struct A1_chain *a1)
{
	return	spi_send_command(a1, A1_BIST_FIX, 0x00, NULL, 0) &&
		spi_poll_result(a1, A1_BIST_FIX, 0x00, 0);
}

static bool cmd_RESET_BCAST(struct A1_chain *a1)
{
	return	spi_send_command(a1, A1_RESET, 0x00, NULL, 0) &&
		spi_poll_result(a1, A1_RESET, 0x00, 0);
}

static bool cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg)
{
	return	spi_send_command(a1, A1_WRITE_REG, chip, reg, 6);
}

static bool cmd_READ_REG(struct A1_chain *a1, uint8_t chip)
{
	return	spi_send_command(a1, A1_READ_REG, chip, NULL, 0) &&
		spi_poll_result(a1, A1_READ_REG_RESP, chip, 6);
}
#endif //GPIO_CS_EN


/********** A1 low level functions */
static void A1_hw_reset(void)
{

#ifdef HW_RESET_EN

		applog(LOG_NOTICE, "ST MCU hardware reset start");

		bcm2835_gpio_fsel(ASIC_RESET_PIN, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_write(ASIC_RESET_PIN, HIGH);
	
		cgsleep_ms(2000);
	
		bcm2835_gpio_fsel(ASIC_RESET_PIN, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_write(ASIC_RESET_PIN, LOW);
	
		cgsleep_ms(2000);
#else
	/* TODO: issue cold reset */
//	usleep(100000);
#endif
}

static void rev(unsigned char *s, size_t l)
{
	size_t i, j;
	unsigned char t;

	for (i = 0, j = l - 1; i < j; i++, j--) {
		t = s[i];
		s[i] = s[j];
		s[j] = t;
	}
}

/********** job creation and result evaluation */
#ifdef A1_DIFFICULTY_EN

static const uint8_t difficult_Tbl[24][4] = {
	{0x1e, 0xff, 0xff, 0xff},	// 1
	{0x1e, 0x7f, 0xff, 0xff},	// 2
	{0x1e, 0x3f, 0xff, 0xff},	// 4
	{0x1e, 0x1f, 0xff, 0xff},	// 8
	{0x1e, 0x0f, 0xff, 0xff},	// 16
	{0x1e, 0x07, 0xff, 0xff},	// 32
	{0x1e, 0x03, 0xff, 0xff},	// 64 
	{0x1e, 0x01, 0xff, 0xff},	// 128
	{0x1e, 0x00, 0xff, 0xff},	// 256
	{0x1e, 0x00, 0x7f, 0xff},	// 512
	{0x1e, 0x00, 0x3f, 0xff},	// 1024
	{0x1e, 0x00, 0x1f, 0xff},	// 2046
	{0x1e, 0x00, 0x0f, 0xff},	// 4096
	{0x1e, 0x00, 0x07, 0xff},	// 8192
	{0x1e, 0x00, 0x03, 0xff},	// 16384
	{0x1e, 0x00, 0x01, 0xff},	// 32768
	{0x1e, 0x00, 0x00, 0xff}	// 65536
};

#endif

//#define work_data_per 0xccccb
#define work_data_per 0x2fffffff 

static uint8_t *create_job(uint8_t chip_id, uint8_t job_id,
			   const char *wdata, const double sdiff)
{
	//static uint8_t job[58] = {
	static uint8_t job[90] = {			// modified for a2
		/* command */
		0x00, 0x00,
		/* wdata 63 to 0 */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* wdata 75 to 64 */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* difficulty */
		0x00, 0x00, 0x00, 0x00,
		/* end nonce */
		0x00, 0x00, 0x00, 0x00
	};
	
	uint8_t diffIdx;
	
	uint8_t data63to0[64];
	uint8_t startnonce[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t data75to64[12];
	uint8_t diff[4]       = {0x1e, 0x03, 0xff, 0xff};
	//uint8_t endnonce[4] = {0x0, 0x01, 0x00, 0x00};	//
	uint8_t endnonce[4] = {0x00, 0x40, 0x00, 0x00};	// 10s
	 // uint8_t endnonce[4] = {0x07, 0xff, 0xff, 0xff};	// 80s
	//uint8_t endnonce[4] = {0x1f, 0xff, 0xff, 0xff};	// 320s
	//uint8_t endnonce[4] = {0xff, 0xff, 0xff, 0xff};	// 46m

	memcpy(data63to0, wdata, 64);
	memcpy(data75to64, wdata+64, 12);
	
	//printf("sdiff = %f\n", sdiff);

#ifdef A1_DIFFICULTY_EN
	if(sdiff > 65535.0)
		memcpy(diff, difficult_Tbl[16], 4);
	else if(sdiff > 32767.0)
	        memcpy(diff, difficult_Tbl[15], 4);
	else if(sdiff > 16383.0)
	        memcpy(diff, difficult_Tbl[14], 4);
	else if(sdiff > 8191.0)
	        memcpy(diff, difficult_Tbl[13], 4);
	else if(sdiff > 4095.0)
                memcpy(diff, difficult_Tbl[12], 4);
	else if(sdiff > 2047.0)
		memcpy(diff, difficult_Tbl[11], 4);
	else if(sdiff > 1023.0)
		memcpy(diff, difficult_Tbl[10], 4);
	else if(sdiff > 511.0)
		memcpy(diff, difficult_Tbl[9], 4);
	else if(sdiff > 255.0)
		memcpy(diff, difficult_Tbl[8], 4);
	else {
		if(opt_diff>=1&&opt_diff<=8)
		{
			diffIdx=opt_diff-1;
			memcpy(diff, difficult_Tbl[diffIdx], 4);
		}
		else
			memcpy(diff, difficult_Tbl[7], 4);
	}
#endif
	
	//if(poll_data_start[chain_num][chip_id-1]==1)
	if(0)
	{

			//printf("diff =0x%.2x 0x%.2x 0x%.2x 0x%.2x \n", diff[0], diff[1], diff[2], diff[3]);	
			startnonce[0]=(poll_data[chain_num][chip_id-1]&(0xff000000))>>24;
			startnonce[1]=(poll_data[chain_num][chip_id-1]&(0x00ff0000))>>16;
			startnonce[2]=(poll_data[chain_num][chip_id-1]&(0x0000ff00))>>8;
		    startnonce[3]=poll_data[chain_num][chip_id-1]&(0x000000ff);
			
			//poll_data+=0x3fffff; 2.5s
			//poll_data+=0xccccb;  0.5s
			//poll_data+=0x1fffff; 1.25s
			poll_data[chain_num][chip_id-1] +=work_data_per;	

			endnonce[0]=(poll_data[chain_num][chip_id-1]&(0xff000000))>>24;
		    endnonce[1]=(poll_data[chain_num][chip_id-1]&(0x00ff0000))>>16;
		    endnonce[2]=(poll_data[chain_num][chip_id-1]&(0x0000ff00))>>8;
		    endnonce[3]=poll_data[chain_num][chip_id-1]&(0x000000ff);
			poll_data[chain_num][chip_id-1]+=1;
			//poll_data = 0;
			//printf("get new data chip%d\n",chip_id);
			if(poll_data[chain_num][chip_id-1] >= (0xffffffff-(work_data_per*2)))
			{
				//printf("work_data is over %8x\n",poll_data[chain_num][chip_id-1]);
				poll_data[chain_num][chip_id-1] = 0;
				
			}	

	}else{

			startnonce[0]=0x00;startnonce[1]=0x00;startnonce[2]=0x00;startnonce[3]=0x00;
			//endnonce[0]=0x01;endnonce[1]=0x00;endnonce[2]=0x00;endnonce[3]=0x00;//10s
			//endnonce[0]=0x00;endnonce[1]=0x80;endnonce[2]=0x00;endnonce[3]=0x00;//5s
			//endnonce[0]=0x00;endnonce[1]=0x20;endnonce[2]=0x00;endnonce[3]=0x00;//1.2s
			endnonce[0]=0xff;endnonce[1]=0xff;endnonce[2]=0xff;endnonce[3]=0xff;//20s
			//poll_data_start[chain_num][chip_id-1]++;
		        //printf("work_data is start %8x\n",poll_data[chain_num][chip_id-1]);

	}


	rev(data63to0, 64);
	rev(startnonce, 4);
	rev(data75to64, 12);
	rev(diff, 4);
	rev(endnonce, 4);

	job[0] = (job_id << 4) | A1_WRITE_JOB;
	job[1] = chip_id;
	memcpy(job+2, 	        data63to0,  64);
	memcpy(job+2+64,        startnonce, 4);
	memcpy(job+2+64+4,      data75to64, 12);
	memcpy(job+2+64+4+12,   diff,       4);
	memcpy(job+2+64+4+12+4, endnonce,   4);	

	return job;
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work)
{
	//unsigned char *midstate = work->midstate;
	//unsigned char *wdata = work->data + 64;
	unsigned char *wdata = work->data;		// modified for a2	
	double sdiff = work->sdiff;

	if(a1->chips == NULL)
		return false;

	struct A1_chip *chip = &a1->chips[chip_id - 1];
	bool retval = false;

	chip->last_queued_id++;
	chip->last_queued_id &= 3;
#if 1
	int n,m,k;
	for(m=0;m<4;m++)
	{
		if (chip->work[m] != NULL) {
			if(chip->work[m] != work)
			{
				work_completed(a1->cgpu, chip->work[m]);
				//n=(chip_id<=5?0:5);
				//for(k=n;k<(n+5);k++)
				//{
					//chip = &a1->chips[k];
					chip->work[0] = NULL;
					chip->work[1] = NULL;
					chip->work[2] = NULL;
					chip->work[3] = NULL;
					
				//}
				//printf("work_completed is ok %d\n",chip_id);
				break;
			}
		}
	}
#endif
	uint8_t *jobdata = create_job(chip_id, chip->last_queued_id + 1,
				      wdata, sdiff);
	
	//printf("chip and chip->last_queued_id is %d %d\n",chip_id, chip->last_queued_id);
if(stMcu==ST_MCU_PRE_HEADER)
{
	//hexdump("JOB", jobdata, 58);
	hexdump("JOB", jobdata, 90);			// modified for a2
	a1->spi_tx[0]=PRE_HEADER;
	a1->spi_tx[1]=PRE_HEADER;
	//memcpy(&a1->spi_tx[2], jobdata, 58);
	memcpy(&a1->spi_tx[2], jobdata, 90);	// modified for a2
	//if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 60)) {
	if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 92)) {	// modified for a2
		/* give back work */
		//work_completed(a1->cgpu, work);
		printf("Failed to set work for chip(cs%d) %d.%d\n",a1->spi_ctx->config.gpio_cs,chip_id, chip->last_queued_id + 1);
		 chip->work[chip->last_queued_id] = work;
		applog(LOG_ERR, "Failed to set work for chip(cs%d) %d.%d",
		       a1->spi_ctx->config.gpio_cs,chip_id, chip->last_queued_id + 1);
		// TODO: what else?
	} else {
		chip->work[chip->last_queued_id] = work;
	}
}
else
{
	//hexdump("JOB", jobdata, 58);
	hexdump("JOB", jobdata, 90);			// modified for a2
	//memcpy(a1->spi_tx, jobdata, 58);
	memcpy(a1->spi_tx, jobdata, 90);		// modified for a2
	//if (!spi_transfer(a1->spi_ctx, jobdata, a1->spi_rx, 58)) {
	if (!spi_transfer(a1->spi_ctx, jobdata, a1->spi_rx, 90)) {		// modified for a2

		/* give back work */
		//work_completed(a1->cgpu, work);
		printf("Failed to set work for chip(cs%d) %d.%d\n",a1->spi_ctx->config.gpio_cs,chip_id, chip->last_queued_id + 1);
		chip->work[chip->last_queued_id] = work;
		applog(LOG_ERR, "Failed to set work for chip(cs%d) %d.%d",
		       a1->spi_ctx->config.gpio_cs,chip_id, chip->last_queued_id + 1);
		// TODO: what else?
	} else {
		chip->work[chip->last_queued_id] = work;
	}

}
		
	return retval;
}

/* check for pending results in a chain, returns false if output queue empty */
static bool get_nonce(struct A1_chain *a1, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	int i;

	int pollLen=0;
	pollLen=MAX_POLL_NUM*a1->num_chips;
	if(pollLen<=0)
		pollLen=MAX_POLL_NUM;

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif
	
	if (!spi_send_command(a1, A1_READ_RESULT, 0x00, NULL, 0))
	{
		return false;
	}

	cgsleep_us(100);

	for(i = 0; i < pollLen; i++) {
		memset(a1->spi_tx, 0, 2);
		if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 2))
			{
				applog(LOG_ERR, "SPI Err(cs%d):get_nonce result",a1->spi_ctx->config.gpio_cs);
			return false;
			}
		hexdump("RX", a1->spi_rx, 2);
		if (a1->spi_rx[0] == A1_READ_RESULT && a1->spi_rx[1] == 0x00) {
			applog(LOG_DEBUG, "Output queue empty");
			return false;
		}
		if ((a1->spi_rx[0] & 0x0f) == A1_READ_RESULT &&
#if 1 //Patch C
		    a1->spi_rx[1] != 0) {
#else		    
		    a1->spi_rx[0] != 0) {
#endif		    
			*job_id = a1->spi_rx[0] >> 4;
			*chip = a1->spi_rx[1];

			if (!spi_transfer(a1->spi_ctx, NULL, nonce, 4))
			{
					applog(LOG_ERR, "SPI Err(cs%d):get_nonce",a1->spi_ctx->config.gpio_cs);
				return false;
			}


			applog(LOG_DEBUG, "Got nonce for chip %d / job_id %d",
			       *chip, *job_id);
			hexdump("nonce", nonce, 4);

			return true;
		}
//nouse		cgsleep_us(50);
	}

	cgtime(&a1->tvEnd);
	timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
	//applog(LOG_WARNING, "ACK(cs%d) timeout:get_nonce - %.2f ms                    ",a1->spi_ctx->config.gpio_cs,a1->tvDiff.tv_usec/1000.0);
	return false;
}

/* reset input work queues in chip chain */
static bool abort_work(struct A1_chain *a1)
{
	/*
	 * for now, the proposed input queue reset does not seem to work
	 * TODO: implement reliable abort method
	 * NOTE: until then, we are completing queued work => stales
	 */

	a1->reset_bc=1;	

	applog(LOG_DEBUG, "abort_work...");
	return true;
}

/********** driver interface */
void exit_A1_chain(struct A1_chain *a1)
{
	if (a1 == NULL)
		return;
	
	if(a1->chips)
	free(a1->chips);
	a1->chips = NULL;
	spi_exit(a1->spi_ctx);
	a1->spi_ctx = NULL;
	free(a1);
}
#ifdef A1_TEST_MODE_EN //assume ST_MCU_EN enable
struct A1_chain *init_Test_A1_chain(struct spi_ctx *ctx)
{
	int i;//,num_chips =0;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	applog(LOG_DEBUG, "A1 init chain");
	if (a1 == NULL) {
		applog(LOG_ERR, "A1_chain allocation error");
		goto failure;
	}
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->num_chips = 0;
	a1->num_cores = 0;	
	a1->chips = NULL;
	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

	return a1;

failure:
	if(a1->chips)
	free(a1->chips);
	a1->chips = NULL;
	spi_exit(a1->spi_ctx);
	a1->spi_ctx = NULL;

	return a1;
}

#endif // A1_TEST_MODE_EN //assume ST_MCU_EN enable

struct A1_chain *init_A1_chain(struct spi_ctx *ctx)
{
	int i,num_chips =0;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	applog(LOG_DEBUG, "A1 init chain");
	if (a1 == NULL) {
		applog(LOG_ERR, "A1_chain allocation error");
		goto failure;
	}
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->status=ASIC_BOARD_OK;
	a1->shutdown=0;
	a1->overheat=0;
	a1->reset_bc=1;	// clear old jobs
	a1->reinit=0;

	//cmd_POWER_ON_BCAST(a1);
#ifdef ST_MCU_EN

	if (!cmd_RESET_BCAST(a1))
		goto failure;

	if (!cmd_BIST_START(a1))
		goto failure;
#else

	if (!cmd_RESET_BCAST(a1) || !cmd_BIST_START(a1))
		goto failure;
#endif
	a1->num_chips = a1->spi_rx[3];
	applog(LOG_WARNING, "spidev%d.%d(cs%d): Found %d A2 chips",
	       a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
	       a1->spi_ctx->config.gpio_cs,a1->num_chips);

	a1->num_cores=0;

	if (a1->num_chips == 0)
		goto failure;

	a1->chips = calloc(a1->num_chips, sizeof(struct A1_chip));
	if (a1->chips == NULL) {
		applog(LOG_ERR, "A1_chips allocation error");
		goto failure;
	}

	if (!cmd_BIST_COLLECT_BCAST(a1))	// modified for a2
		goto failure;

	if (!cmd_BIST_FIX_BCAST(a1))
		goto failure;

	if(a1->num_chips<MAX_ASIC_NUMS)
		a1->status|=ERROR_CHIP;

	for (i = 0; i < a1->num_chips; i++) {
		int chip_id = i + 1;
		if (!cmd_READ_REG(a1, chip_id)) {
			applog(LOG_WARNING, "Failed to read register for "
			       "chip %d -> disabling", chip_id);
			a1->chips[i].num_cores = 0;
			continue;
		}
		num_chips++;
		//a1->chips[i].num_cores = a1->spi_rx[7];
		a1->chips[i].num_cores = a1->spi_rx[13];		// modified for a2
		//printf("spi_rx[13] is %d\n",a1->spi_rx[13]);
		//if(a1->spi_rx[7]>MAX_CORE_NUMS)
		if(a1->chips[i].num_cores>MAX_CORE_NUMS)		// modified for a2
			a1->chips[i].num_cores = MAX_CORE_NUMS;

		if(a1->chips[i].num_cores<MAX_CORE_NUMS)
			a1->status=ERROR_CORE;

		//memcpy(a1->chips[i].reg, a1->spi_rx + 2, 6);	//keep ASIC register value
		memcpy(a1->chips[i].reg, a1->spi_rx + 2, 12);	//keep ASIC register value	// modified for a2
#ifdef A1_TEMP_EN	
		//a1->chips[i].temp= (a1->spi_rx[8]<<8)|a1->spi_rx[9];
		if(i < 4) {
			a1->chips[i].temp= (a1->spi_rx[14]<<8)|a1->spi_rx[15];	// modified for a2
		}
#endif

		a1->num_cores += a1->chips[i].num_cores;
		applog(LOG_WARNING, "Found chip %2d with %2d active cores",
		       chip_id, a1->chips[i].num_cores);

		a1->chips[i].reset_errors = 0;
	}
	applog(LOG_WARNING, "Found %d chips with total %d active cores",
	       a1->num_chips, a1->num_cores);

	if(a1->num_cores==0) // The A1 board  haven't any cores
		goto failure;		

	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

	return a1;

failure:

	a1->status=ERROR_BOARD;

	exit_A1_chain(a1);
	return NULL;
}

static bool A1_detect_one_chain(struct spi_config *cfg)
{
	struct cgpu_info *cgpu;
	struct spi_ctx *ctx = spi_init(cfg);
	struct A1_chain *a1;
	int i;

	if (ctx == NULL)
		return false;
#ifdef A1_TEST_MODE_EN
	if(testMode>0)
		a1 = init_Test_A1_chain(ctx);
	else
#endif
		a1 = init_A1_chain(ctx);

	if (a1 == NULL)
		return false;

	cgpu = malloc(sizeof(*cgpu));
	if (cgpu == NULL) {
		applog(LOG_ERR, "cgpu allocation error");
		exit_A1_chain(a1);
		return false;
	}
	memset(cgpu, 0, sizeof(*cgpu));
	cgpu->drv = &bitmineA1_drv;
	cgpu->name = "BitmineA1";
	cgpu->threads = 1;

	cgpu->device_data = a1;

#ifdef A1_TEST_MODE_EN
	if(testMode)
		cgpu->deven = DEV_DISABLED;
	else
#endif
	cgpu->deven = DEV_ENABLED;


	a1->cgpu = cgpu;
#ifdef USE_ST_MCU	
	cgpu->restart= 0;
#endif
	a1->cutTemp= stTempCut;
	a1->runTemp= stTempRun;

	add_cgpu(cgpu);
#ifdef A1_PLL_CLOCK_EN
	switch(cgpu->device_id){
		case 0:A1_SetA1PLLClock(a1, A1Pll1);break;
		case 1:A1_SetA1PLLClock(a1, A1Pll2);break;
		case 2:A1_SetA1PLLClock(a1, A1Pll3);break;
		case 3:A1_SetA1PLLClock(a1, A1Pll4);break;
		case 4:A1_SetA1PLLClock(a1, A1Pll5);break;
		case 5:A1_SetA1PLLClock(a1, A1Pll6);break;
		default:;
	}
#endif
	return true;
}

#define MAX_SPI_BUS	1
//#define MAX_SPI_CS	2
#define MAX_SPI_CS	1

/* Probe SPI channel and register chip chain */
void A1_detect(bool hotplug)
{
	int bus;
	int cs_line;
//	uint32_t speed=DEFAULT_SPI_SPEED; // 1 500 000;
	uint32_t speed=4000000;  // 4Mhz
//	uint32_t speed=8000000;  // 8Mhz
	uint8_t no_cgpu=0;

	if(!CheckForAnotherInstance())
	{
        pabort("Another cgminer is running");
		return;
	}
	
	/* no hotplug support for now */
	if (hotplug)
		return;
	applog(LOG_DEBUG, "A1 detect");

	gpio_CS_Init();
	
#ifdef A1_TEST_MODE_EN
	if(opt_test>0)
	{
		testMode=opt_test;
		applog(LOG_NOTICE, "Run Test Mode");

	}
#endif
	applog(LOG_NOTICE, "Run Reset=%d",opt_hwReset);

	if(opt_hwReset==true)
	A1_hw_reset();

#ifdef A1_SPI_SPEED_EN
	if(opt_spiSpeed>0)
		speed=opt_spiSpeed*100000; // x 1kHz
	applog(LOG_NOTICE, "SPI Speed %d kHz",speed/1000);

#endif

#ifdef ST_MCU_EN
	if(opt_stmcu<0)
	{
		stMcu=ST_MCU_NULL;
		applog(LOG_NOTICE, "ST MCU - Disable");		
	}
	else
	{
		stMcu=ST_MCU_PRE_HEADER;
		applog(LOG_NOTICE, "ST MCU - Enable (Pre-header)");		
	}
#endif

#ifdef A1_TEMP_EN	
	if(opt_tempCut>0)
	{
		stTempCut = opt_tempCut;
		applog(LOG_NOTICE, "Cutoff temperature %dC",stTempCut);		
	}

	if(opt_tempRun>0)
	{
		stTempRun = opt_tempRun;
		applog(LOG_NOTICE, "Run    temperature %dC",stTempRun);		
	}
#endif

#ifdef A1_PLL_CLOCK_EN
		A1Pll1 = A1_ConfigA1PLLClock(opt_A1Pll1);
		A1Pll2 = A1_ConfigA1PLLClock(opt_A1Pll2);
		A1Pll3 = A1_ConfigA1PLLClock(opt_A1Pll3);
		A1Pll4 = A1_ConfigA1PLLClock(opt_A1Pll4);
		A1Pll5 = A1_ConfigA1PLLClock(opt_A1Pll5);
		A1Pll6 = A1_ConfigA1PLLClock(opt_A1Pll6);
#endif

	//const int diff_table[16]={1,2,3,4,8,16,32,37,43,52,65,86,103,129,256,1024};

	//if(opt_diff>=1&&opt_diff<=16)
	//	applog(LOG_NOTICE, "ASIC Difficulty =  %d",diff_table[opt_diff-1]);		
	//else
	//	applog(LOG_NOTICE, "ASIC Difficulty =  %d",256); //default

#ifdef USE_GPIO_CS 
	uint8_t gpio_cs,minGpioCS,maxGpioCS;
	struct spi_config cfg = default_spi_config;


#ifdef A1_TEST_MODE_EN
	if(testMode>0)
		opt_gcs=G_CS_MAX; //force auto CS if test mode
#endif
	if(opt_gcs<G_CS_MAX)
	{
		if(opt_gcs<0)
		{
			minGpioCS=G_CS_0;
			maxGpioCS=G_CS_0+1;
			applog(LOG_NOTICE, "GPIO CS is OFF");
		}
		else
		{
			minGpioCS=opt_gcs;
			maxGpioCS=opt_gcs+1;
			applog(LOG_NOTICE, "GPIO CS is ON at CS%d",opt_gcs);		
		}
	}
	else //auto >=G_CS_MAX
	{
		minGpioCS=G_CS_0;
		maxGpioCS=MAX_ASIC_BOARD;
		applog(LOG_NOTICE, "AUTO GPIO CS");
	}

	for (gpio_cs = minGpioCS; gpio_cs < maxGpioCS; gpio_cs++) {
		for (bus = 0; bus < MAX_SPI_BUS; bus++) {
			for (cs_line = 0; cs_line < MAX_SPI_CS; cs_line++) {
//				if(gpio_cs ==G_CS_6) //hardware issue skip CS 6
//					continue;
				cfg = default_spi_config;
				cfg.mode = SPI_MODE_1;
				cfg.speed = speed;
				cfg.bus = bus;
				cfg.cs_line = cs_line;
				cfg.gpio_cs = gpio_cs;
				if(A1_detect_one_chain(&cfg))
					no_cgpu++;
			}
		}
	}

#else
	for (bus = 0; bus < MAX_SPI_BUS; bus++) {
		for (cs_line = 0; cs_line < MAX_SPI_CS; cs_line++) {
			struct spi_config cfg = default_spi_config;
			cfg.mode = SPI_MODE_1;
			cfg.speed = speed;
			cfg.bus = bus;
			cfg.cs_line = cs_line;
			if(A1_detect_one_chain(&cfg))
				no_cgpu++;
		}
	}

#endif //USE_GPIO_CS 


	if (no_cgpu==0) {
		applog(LOG_ERR, "No any A1 board");
		pabort("\nNo any A1 board\n");
	}
	else 
	{
		struct cgpu_info *cgpu;
		struct A1_chain *a1;
		uint32_t num_cores=0,eff;
		float speed;
		int i;
		for(i=0;i<no_cgpu;i++)
		{
			cgpu = get_devices(i);
			a1 = cgpu->device_data;
			num_cores+=a1->num_cores;
		}
		//eff=((num_cores*100)/(no_cgpu*MAX_CORE_NUMS*MAX_ASIC_NUMS));
		//speed=(no_cgpu*eff*A1_MAX_SPEED*PLL_Clk_12Mhz[A1Pll1].speedMHz/1000)/100.0;
		//applog(LOG_NOTICE, "A2 boards=%d, active cores=%d, Efficient=%d%%, speed=%.1fM",no_cgpu,num_cores,eff,speed);
		speed=no_cgpu*A4_NORMAL_SPEED*num_cores/100.0;
		applog(LOG_NOTICE, "A2 boards=%d, active cores=%d, speed=%.2fM",no_cgpu,num_cores,speed);
	}



}
static int scan_chip_count = 0;
pthread_mutex_t lock_all;
static unsigned int  core_reset_time_delay = 0;
static int64_t A1_scanwork(struct thr_info *thr)
{
	int i,j,p;
	int ak,no_result = 0;
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;
	int32_t nonce_cnt = 0;
	int32_t A1Pll = 1000;
	int32_t nonce_ranges_processed = 0;

//	applog(LOG_ERR, "A1_scanwork cs=%d",a1->spi_ctx->config.gpio_cs);

	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id,evtKey;
	bool work_updated = false,overheat=false;
	//static struct work *work_get;
	//printf("xxxxxcgpu id is %d\n",a1->chips->temp);
#if 0
	if(a1->st_reset_delay==0)
	{
		a1->st_reset_delay = 1000;
	}else if( a1->st_reset_delay >1 ){
                        if(a1->work_start_delay==1)
                        {
				 printf("st reset.......... \n"); 
				 a1->st_reset_delay--;	
				if(a1->st_reset_delay == 999)
				      cmd_POWER_ON_BCAST(a1);
                 		 if(a1->st_reset_delay == 50)
                  		{

                        	switch(cgpu->device_id){
                        	case 0:A1_SetA1PLLClock(a1, A1Pll1);break;
                        	case 1:A1_SetA1PLLClock(a1, A1Pll2);break;
                        	case 2:A1_SetA1PLLClock(a1, A1Pll3);break;
                        	case 3:A1_SetA1PLLClock(a1, A1Pll4);break;
                        	case 4:A1_SetA1PLLClock(a1, A1Pll5);break;
                        	case 5:A1_SetA1PLLClock(a1, A1Pll6);break;
                        	default:;
                        	}
                  		}		
                    		goto scan_delay;

					
                        }
        }
 
#endif
#if 0
	switch(KeyDetect())
	{
		case KEY_EVENT_RESET:
			A1_reinit();
			break;
	}
#endif

	mutex_lock(&a1->lock);
	//scan_chip_count++;
	//printf("%d lock is set try\n",a1->spi_ctx->config.gpio_cs);
	//if(scan_chip_count>=300)
	//{
	mutex_lock(&lock_all);
	  // printf("%d lock is set ok\n",a1->spi_ctx->config.gpio_cs);

	//}
#ifdef USE_ST_MCU	
	if(cgpu->restart)
	{
		A1_reinit_device(cgpu);
		cgpu->restart=0;
		mutex_unlock(&a1->lock);
		mutex_unlock(&lock_all);
		return 0;
	}
#endif
	if(a1->chips == NULL || a1->shutdown) //not need post job as not chip avaiable
	{
		cgsleep_ms(100);/* in case of no progress, prevent busy looping */
		mutex_unlock(&a1->lock);
		mutex_unlock(&lock_all);
		return 0;
	}
#if 0
	if(a1->overheat) { //overheat
		cmd_POWER_OFF_BCAST(a1);
		a1->overheat = 0;

                for (i = 0; i < a1->num_chips; i++) {
                        if(a1->chips[i].num_cores==0) { //No any core can do work in this chip
				a1->chips[i].temp = 0;
                                continue;
			}

			if(i < 4) {  // temp sensor correct 
                        	if (!cmd_READ_REG(a1, i + 1)) {
                        	        applog(LOG_ERR, "Failed to read reg from chip(cs%d) %d            ",a1->spi_ctx->config.gpio_cs,  i);
                        	        continue;
                        	}
				
				a1->chips[i].temp = (a1->spi_rx[14]<<8)|a1->spi_rx[15];

                        	if(a1->chips[i].temp > a1->runTemp) {
                        	        a1->overheat = 1;
				}
			}
		}

		cgsleep_ms(1000);/* in case of no progress, prevent busy looping */
		mutex_unlock(&a1->lock);
		return 0;
	}
	else {
		//cmd_POWER_ON_BCAST(a1);
	}

        if(a1->reset_bc == 0) {
		if(a1->hashes_times > 300) {
                        applog(LOG_DEBUG, "Job is working over 60s, flushing work, chip(cs%d)          ",a1->spi_ctx->config.gpio_cs);
                        a1->hashes_times = 0;
			A1_flush_work_unlock(cgpu);
			mutex_unlock(&a1->lock);
			return 0;
                }
                else {
                        a1->hashes_times++;
                }
	}

        if(a1->reset_bc == 0) {
		a1->num_cores = 0;
		
		for (i = 0; i < a1->num_chips; i++) {
			if(a1->chips[i].num_cores==0) { //No any core can do work in this chip
				a1->chips[i].temp = 0;
				continue;
			}

			if (!cmd_READ_REG(a1, i + 1)) {
				applog(LOG_ERR, "Failed to read reg from chip(cs%d) %d            ",a1->spi_ctx->config.gpio_cs,  i);
				continue;
			}

			if(i < 4) {
				a1->chips[i].temp = (a1->spi_rx[14]<<8)|a1->spi_rx[15];
				if(a1->cutTemp > 0) {
				 	if(a1->chips[i].temp >= a1->cutTemp) {	
						a1->reset_bc = 1;
						a1->reinit = 1;	
						a1->overheat = 1;
						applog(LOG_ERR, "Overheat from chip(cs%d) %d            ",a1->spi_ctx->config.gpio_cs,  i);
						continue;
					}
				}
			}
//wutuhua

                	if(a1->spi_rx[13] == 0){ // a1->chips[i].num_cores
                	        a1->reset_bc = 1;
				a1->chips[i].reset_errors++;
                	        applog(LOG_ERR, "Failed to read num_cores from chip(cs%d) %d (check job)",a1->spi_ctx->config.gpio_cs,  i);
                	}
			else if ((a1->spi_rx[11] & 0x01) != 0x01) { // not working 
				a1->reset_bc = 1;
				a1->chips[i].reset_errors++;
				
				//printf("xxxxx%d %d %d %d %d %d\n",a1->spi_rx[0],a1->spi_rx[1],a1->spi_rx[2],a1->spi_rx[3],a1->spi_rx[4],a1->spi_rx[5]);
				//printf("xxxxx%d %d %d %d %d %d\n",a1->spi_rx[6],a1->spi_rx[7],a1->spi_rx[8],a1->spi_rx[9],a1->spi_rx[10],a1->spi_rx[11]);
				//printf("xxxxxssssss %d %d %d %d\n",a1->spi_rx[12],a1->spi_rx[13],a1->spi_rx[14],a1->spi_rx[15]);

		
				applog(LOG_ERR, "Job is not working from chip(cs%d) %d          ",a1->spi_ctx->config.gpio_cs,  i);
			}
			else
			{
				a1->chips[i].reset_errors = 0;
			}

			 //a1->chips[i].reset_errors = 0;//wutuhua


			if (a1->chips[i].reset_errors > 2){
				a1->chips[i].num_cores = 0;
				a1->chips[i].reset_errors = 0;
				a1->reinit = 1;
				applog(LOG_ERR, "Job it not working for many times chip(cs%d) %d  ",a1->spi_ctx->config.gpio_cs,  i);
			}	
		

			//a1->num_cores += a1->chips[i].num_cores;
		}
	}
		
        if(a1->reset_bc == 1)
        {
		a1->reset_bc = 0;
		a1->hashes_times = 0;
		cgtime(&a1->tvScryptLast);
		cgtime(&a1->tvScryptCurr);

		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);
		cmd_RESETBC_BCAST(a1);

		while (get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)) {
			nonce_cnt++;
                	if (nonce_cnt > 8){
				a1->reinit = 1;
                        	applog(LOG_ERR, "(cs%d) reset bc get nonce deadlock              ",a1->spi_ctx->config.gpio_cs);
				break;
                	}
		}

		for (i = 0; i < a1->num_chips; i++) {
			if(a1->chips[i].num_cores==0) { //No any core can do work in this chip
				a1->chips[i].temp = 0;
				continue;
			}

			if (!cmd_READ_REG(a1, i + 1)) {
				applog(LOG_ERR, "Failed to read reg from chip(cs%d) %d           ",a1->spi_ctx->config.gpio_cs,  i);
				continue;
			}

			if(i < 4) {
				a1->chips[i].temp = (a1->spi_rx[14]<<8)|a1->spi_rx[15];
			}

			if(a1->spi_rx[13] == 0){ // a1->chips[i].num_cores
				a1->reinit = 1;
                                applog(LOG_ERR, "Failed to read num_cores from chip(cs%d) %d (reset bc)",a1->spi_ctx->config.gpio_cs, i);
                        }
			else if ((a1->spi_rx[11] & 0x01) == 0x01) { // reset bc fail ?
				a1->reinit = 1;
				applog(LOG_ERR, "Failed to reset bc from chip(cs%d) %d           ",a1->spi_ctx->config.gpio_cs,  i);
			}
		}
        }

	if(a1->reinit == 1)
	{
		A1_flush_work_unlock(cgpu);
		A1_reinit_device(cgpu);
		a1->reinit = 0;
		cgsleep_ms(1000);
		mutex_unlock(&a1->lock);
		return 0;
	}
#endif


	/* poll queued results */
	while (get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)) {
		//nonce = bswap_32(nonce);		// modified for a2
		work_updated = true;
		nonce_cnt++;
		//printf("get_nonce chip_id and job_id is %d %d\n",chip_id,job_id);
		asic_no_result[a1->spi_ctx->config.gpio_cs][chip_id-1] = 0;
		if(nonce_cnt == 1)
		{
			//scan_chip_count = -1;
		}
#if 1
		if(chip_id == 0 ||chip_id > a1->num_chips)
			continue;
		if(job_id == 0 ||job_id>4 )
			continue;
#else
		assert(chip_id > 0 && chip_id <= a1->num_chips);
		assert(job_id > 0 && job_id <= 4);
#endif		

		struct A1_chip *chip = &a1->chips[chip_id - 1];
		struct work *work = chip->work[job_id - 1];
		if (work == NULL) {
			/* already been flushed => stale */
			//a1->reset_bc = 1;
			//applog(LOG_ERR, "chip(cs%d) %d: stale nonce 0x%08x                                       ",
			//       a1->spi_ctx->config.gpio_cs,chip_id, nonce);
			chip->stales++;
			//cgsleep_ms(100);
			//mutex_unlock(&a1->lock);
			break;
			//return 0;
		}
		 //chip->hw_errors++;
		 //cgpu->hw_errors++;
                // printf("hw is error...\n");

		if (!submit_nonce(thr, work, nonce)) {
			if(opt_realquiet)
				{;}//applog(LOG_ERR, "chip(cs%d) %d: invalid nonce 0x%08x                                 ", 
			       //a1->spi_ctx->config.gpio_cs,chip_id, nonce);
			else				
				{;}//applog(LOG_ERR, "chip(cs%d) %d: invalid nonce 0x%08x                                 ", 
			       //a1->spi_ctx->config.gpio_cs,chip_id, nonce);
			chip->hw_errors=0;
			hw_reset_count[a1->spi_ctx->config.gpio_cs]++;
			hw_count_temp[a1->spi_ctx->config.gpio_cs]++;
			//cgpu->hw_errors=chip->hw_errors/2;
			//printf("hw is error...\n");
			//quit(1, "hw reset........\n");
			/* add a penalty of a full nonce range on HW errors */
			nonce_ranges_processed--;
			//scan_chip_count=-1;

			continue;
		}
		//applog(LOG_DEBUG, "YEAH: chip(cs%d) %d: job %d nonce 0x%08x",
		 //      a1->spi_ctx->config.gpio_cs,chip_id,job_id,nonce);
		chip->nonces_found++;
	}

	//applog(LOG_DEBUG, "chip(cs%d) nonce_cnt = %d", a1->spi_ctx->config.gpio_cs, nonce_cnt);
	hw_time[a1->spi_ctx->config.gpio_cs]++;
	//printf(" a1->hw_time is %d \n", hw_time[a1->spi_ctx->config.gpio_cs]);
	if(spi_error_count >= 3000)//3000
	{
			system("sudo ./run.sh");
                        quit(1, "hw reset........\n");
	
	}
	if(hw_reset_count[a1->spi_ctx->config.gpio_cs]>=200)
	{
				hw_reset_count[a1->spi_ctx->config.gpio_cs]=0;
				if(a1->work_start_delay == 1)
                                   a1->work_start_delay = 3;
                                spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                cgsleep_us(50);
                                spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
				printf("acc is ok..........\n");

	}
	clear_log[a1->spi_ctx->config.gpio_cs]++;
	if(clear_log[a1->spi_ctx->config.gpio_cs]>=1000)
	{
		clear_log[a1->spi_ctx->config.gpio_cs]=0;
		system("echo "" ./cgminer.log");

	}
	for(ak=0;ak<20;ak++)
	{
		asic_no_result[a1->spi_ctx->config.gpio_cs][ak]++;
		//printf("%d asic_no_result  is %d \n",a1->spi_ctx->config.gpio_cs, asic_no_result[a1->spi_ctx->config.gpio_cs][ak]);
		if(asic_no_result[a1->spi_ctx->config.gpio_cs][ak]>12000)
			asic_no_result[a1->spi_ctx->config.gpio_cs][ak] = 12000;
	}
	for(ak=0;ak<20;ak++)
	{
		if(asic_no_result[a1->spi_ctx->config.gpio_cs][ak]>=12000)
			no_result++;
			
	}
	if(no_result >= 4)
	{
				printf("asic_no_result reset A1_POWER_RESET is try..........\n");
		 		if(a1->work_start_delay == 1)
                                   a1->work_start_delay = 6;
                                spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                cgsleep_us(50);
                                spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
				for(ak=0;ak<20;ak++)
					 asic_no_result[a1->spi_ctx->config.gpio_cs][ak] = 0;

	}
	
	//if(core_reset_time_delay)core_reset_time_delay--;
	//static unsigned int  core_reset_time_delay = 0;
	if(hw_time[a1->spi_ctx->config.gpio_cs]>=50)
	{
		hw_time[a1->spi_ctx->config.gpio_cs] = 0;
				
		int core_i=0,core_j=0,core_all_count = 0;
                //static unsigned int  core_reset_time_delay = 0;
                for(core_i=0;core_i<20;core_i++)
                {
                         core_all_count += a1->chips[core_i].num_cores;
                         if(a1->chips[core_i].num_cores == 0)
                                core_j++;

                }
                if((core_j>=9)&&(core_reset_time_delay == 0))
                {
                                printf(" cs(%d)  core error is reset .....\n",a1->spi_ctx->config.gpio_cs);
                                if(a1->work_start_delay == 1)
                                {
                                        a1->work_start_delay = 6;
				}
				core_reset_time_delay = 50;
                                spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                cgsleep_us(50);
                                spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
                                
                }

                if(core_all_count<1850)
                         printf("cs(%d)  cores is %d .....\n",a1->spi_ctx->config.gpio_cs,core_all_count);

		if(a1->chips[0].temp>90)
			 printf(" cs(%d)  temp is %d .....\n",a1->spi_ctx->config.gpio_cs,a1->chips[0].temp);


		if(hw_count_temp[a1->spi_ctx->config.gpio_cs]>=20)
		{
			system("sudo ./run.sh");
 			quit(1, "hw reset........\n");
		}
		 //printf("hw_count_temp is %d ...........\n",hw_count_temp[a1->spi_ctx->config.gpio_cs]);
		hw_count_temp[a1->spi_ctx->config.gpio_cs] = 0;
//	 	 printf("hw_count_temp is %d ...........\n",hw_count_temp[a1->spi_ctx->config.gpio_cs]);	
		if(chip_no_work_count[a1->spi_ctx->config.gpio_cs]>=50)//25
		{
			//system("sudo ./run.sh");
                        //quit(1, "hw reset........\n");
			cmd_RESETBC_BCAST(a1);
			bc_reset_count[a1->spi_ctx->config.gpio_cs]++;
			if(bc_reset_count[a1->spi_ctx->config.gpio_cs]>=8)//4
			{
				bc_reset_count[a1->spi_ctx->config.gpio_cs]=0;
				if(a1->work_start_delay == 1)
		           	   a1->work_start_delay = 6;
			        spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
			        cgsleep_us(50);
			        spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
				printf("chip_no_work A1_POWER_RESET is try ..... \n");
									
			}

			#if 0
				
				if(a1->spi_ctx->config.gpio_cs==1)
				{
					 if(a1->work_start_delay == 1)
                                   		a1->work_start_delay = 6;
					
					spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                	cgsleep_us(50);
                                	spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);

				}
			
			#endif 
			
		}else  bc_reset_count[a1->spi_ctx->config.gpio_cs]=0;
		//printf("chip_no_work_count is %d ...........\n",chip_no_work_count[a1->spi_ctx->config.gpio_cs]);
		chip_no_work_count[a1->spi_ctx->config.gpio_cs] = 0;
		#if 0

                                if(a1->spi_ctx->config.gpio_cs==1)
                                {
                                         if(a1->work_start_delay == 1)
                                                a1->work_start_delay = 6;

                                        spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                        cgsleep_us(50);
                                        spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);

                                }

                #endif

	}	
		#if 0
		int core_i=0,core_j=0,core_all_count = 0;
		static char core_reset_flag = 0;
		for(core_i=0;core_i<20;core_i++)
                {	
			 core_all_count += a1->chips[core_i].num_cores; 
                         if(a1->chips[core_i].num_cores == 0)
                                core_j++;

                }
                if(core_j>=9)
                {
                                printf(" %d  core error is reset .....\n",a1->spi_ctx->config.gpio_cs);
                                if(a1->work_start_delay == 1)
				{
                                   	a1->work_start_delay = 30;
                                	spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                	cgsleep_us(50);
                                	spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
				}

                }
		if(core_all_count<1850)
			 printf(" %d  cores is %d .....\n",a1->spi_ctx->config.gpio_cs,core_all_count);
		#endif

	if(core_reset_time_delay)core_reset_time_delay--;

	/* check for completed works */
	if(a1->work_start_delay>1)
	{
		a1->work_start_delay--;
		//printf("delay-----\n");
	}
	else
	{	
		
	    for (i = 0; i < a1->num_chips; i++) {
#if 0 //Patch D
		if(a1->chips[i].num_cores==0) //No any core can do work in this chip
			continue;
#endif

		if (!cmd_READ_REG(a1, i + 1)) {
			applog(LOG_ERR, "Failed to read reg from chip(cs%d) %d",a1->spi_ctx->config.gpio_cs,  i);
			// TODO: what to do now?
			spi_error_count++;
			continue;
		}
		if(1) {
			a1->chips[i].temp = (a1->spi_rx[14]<<8)|a1->spi_rx[15];
		}
		a1->chips[i].num_cores = a1->spi_rx[13];

		#if 0
		for(core_i=0;core_i<20;core_i++)
		{
			 if(a1->chips[core_i].num_cores == 0)
				core_j++;
	
		}
		if(core_j>=9)
		{
				printf(" %d  core error is reset .....\n",a1->spi_ctx->config.gpio_cs);
				if(a1->work_start_delay == 1)
                                   a1->work_start_delay = 6;
                                spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                cgsleep_us(50);
                                spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
				
	
		}
		#endif

#if 0
		if(a1->spi_rx[13] == 0){ // a1->chips[i].num_cores
			a1->reset_bc = 1;
			applog(LOG_ERR, "Failed to read num_cores from chip(cs%d) %d (write job)",a1->spi_ctx->config.gpio_cs,  i);

			cgsleep_ms(100);
			mutex_unlock(&a1->lock);
                	return 0;
		}
#endif
		hexdump("A1 RX", a1->spi_rx, 8);
#ifdef A1_TEMP_EN	
		//a1->chips[i].temp= (a1->spi_rx[8]<<8)|a1->spi_rx[9];
		//a1->chips[i].temp= (a1->spi_rx[14]<<8)|a1->spi_rx[15];	// modified for a2
#endif			
		#if 0
		if((a1->spi_rx[11] == 0x00)&&(a1->work_start_delay))
			printf("cs %d chip%d is %02x\n",a1->spi_ctx->config.gpio_cs,i,a1->spi_rx[11]);
		if((a1->spi_rx[11] == 0x20)&&(a1->work_start_delay))
                        printf("cs %d chip%d is %02x\n",a1->spi_ctx->config.gpio_cs,i,a1->spi_rx[11]);
		#endif
		
		if(((a1->spi_rx[11] & 0x01) != 0x01)&&(a1->work_start_delay))
		{
			//printf("cs %d chip%d is %02x\n",a1->spi_ctx->config.gpio_cs,i,a1->spi_rx[11]);
			if(work_get_flag[a1->spi_ctx->config.gpio_cs])
			  chip_no_work_count[a1->spi_ctx->config.gpio_cs]++;
		}	
		if(((a1->spi_rx[11] == 0x00)||(a1->spi_rx[11] == 0x20))&&(a1->work_start_delay))
		{
				//if(a1->work_start_delay == 1)
                                //   a1->work_start_delay = 6;
                                //spi_send_command(a1, A1_POWER_RESET, 0x00, NULL, 0);
                                //cgsleep_us(50);
                                //spi_poll_result(a1, A1_POWER_RESET, 0x00, 2);
                                //printf("asic_error A1_POWER_RESET is try ..... \n");
	
		}	
		//if ((a1->spi_rx[5] & 0x02) != 0x02) {
		//if(a1->get_reg_num[i]!=a1->spi_rx[12])
		//	a1->get_reg_num[i]=a1->spi_rx[12];
		//else continue;
		//if ((a1->spi_rx[11] & 0x02) == 0x00) {					// modified for a2
		if ((a1->spi_rx[11] & 0x01) != 0x01) {
			applog(LOG_DEBUG, "get_work...");
			work_updated = true;
			//static unsigned int work_s = 0;
			static unsigned int work_queue_count = 0;
			//static  struct work *work_old[3]={NULL,NULL,NULL};
			
			int n,m,k;
			struct A1_chip *chip1;			
			chain_num = a1->spi_ctx->config.gpio_cs;
			k=i/5;
			if(work_get_flag[chain_num]==0)
			{		
					//printf("cs%d chip%d try get work\n",chain_num,i);
					if(a1->work_cur[i]==NULL)
					{
						struct work *work_temp=wq_dequeue(&a1->active_wq);
						a1->work_cur[i]=work_temp;	
						//struct work *work_temp = wq_dequeue(&a1->active_wq);
						//printf("chip %d get work is ok\n",i);
					}
					for(n=0;n<a1->num_chips;n++)
					//for(n=0;n<2;n++)
					{
						if(a1->work_cur[n]==NULL)
							break;
					}
					if(n<a1->num_chips)continue;
					//if(n<2)continue;
					work_get_flag[chain_num]=1;
					// printf("get work is ok\n");
					i=0;
			}
			#if 0
			 if(a1->get_reg_num[i]!=a1->spi_rx[12])
                	        a1->get_reg_num[i]=a1->spi_rx[12];
		         else continue;
			#endif
#if 0			
                if(a1->st_reset_delay==0)
                {
                   a1->st_reset_delay = 1000;
                   printf("st reset.......... \n");
                   p = 100;
                   while(!cmd_POWER_ON_BCAST(a1))
                   {
                        p--;
                        if(p==0)
                        break;
                   }
		   goto scan_delay;
		}
   		else if( a1->st_reset_delay > 1)
		{
                  a1->st_reset_delay--;
                  if(a1->st_reset_delay == 50)
                  {

                        switch(cgpu->device_id){
                        case 0:A1_SetA1PLLClock(a1, A1Pll1);break;
                        case 1:A1_SetA1PLLClock(a1, A1Pll2);break;
                        case 2:A1_SetA1PLLClock(a1, A1Pll3);break;
                        case 3:A1_SetA1PLLClock(a1, A1Pll4);break;
                        case 4:A1_SetA1PLLClock(a1, A1Pll5);break;
                        case 5:A1_SetA1PLLClock(a1, A1Pll6);break;
                        default:;
                        }
                  }
		    goto scan_delay;
       	        }
#endif			
			
			if (!set_work(a1, i + 1, a1->work_cur[i]))	
			{
				
				if((i==19)&&(a1->pool_reset_flag))
				{
					a1->pool_reset_flag=0;
					a1->work_start_delay = 10;
					//printf("cg reset is try");
					//system("sudo ./run.sh");
					//printf("cg reset is try2");
					//quit(1, "hw reset........\n");
				}

				//while(1);
				if(a1->work_start_delay==0)
				{
					
					a1->work_start_delay = 10;
				}
				continue;
			}
			nonce_ranges_processed++;
			struct A1_chip *chip = &a1->chips[i];
			chip->nonce_ranges_done++;
#if 0 // not need			
			applog(LOG_NOTICE, "chip(cs%d) %d: job done => %d/%d/%d/%d",
			       a1->spi_ctx->config.gpio_cs,i + 1,
			       chip->nonce_ranges_done, chip->nonces_found,
			       chip->hw_errors, chip->stales);
#endif
		}
	}
	}
scan_delay:
	mutex_unlock(&a1->lock);
	mutex_unlock(&lock_all);

	if (nonce_ranges_processed < 0) {
		applog(LOG_INFO, "Negative nonce_processed");
		nonce_ranges_processed = 0;
	}

	if (nonce_ranges_processed != 0) {
		applog(LOG_INFO, "nonces processed %d",nonce_ranges_processed);
	}

	cgtime(&a1->tvScryptCurr);
	timersub(&a1->tvScryptCurr, &a1->tvScryptLast, &a1->tvScryptDiff);
	cgtime(&a1->tvScryptLast);

	/* in case of no progress, prevent busy looping */
	//if (!work_updated)
	 cgsleep_ms(100);

	//applog(LOG_NOTICE, "second %d, usecond %d",a1->tvScryptDiff.tv_sec,a1->tvScryptDiff.tv_usec);

	if(nonce_ranges_processed==-1) //prevent A1 board disable
		nonce_ranges_processed =0;

	switch(cgpu->device_id){
		case 0:A1Pll = PLL_Clk_12Mhz[A1Pll1].speedMHz;break;
		case 1:A1Pll = PLL_Clk_12Mhz[A1Pll2].speedMHz;break;
		case 2:A1Pll = PLL_Clk_12Mhz[A1Pll3].speedMHz;break;
		case 3:A1Pll = PLL_Clk_12Mhz[A1Pll4].speedMHz;break;
		case 4:A1Pll = PLL_Clk_12Mhz[A1Pll5].speedMHz;break;
		case 5:A1Pll = PLL_Clk_12Mhz[A1Pll6].speedMHz;break;
		default:;
	}
	return (int64_t)(1550832.85 * A1Pll/1000 * (a1->num_cores/54.0) * (a1->tvScryptDiff.tv_usec / 1000000.0));

}

/* queue one work per chip in chain */
static bool A1_queue_full(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int queue_full = false;
	struct work *work;

	mutex_lock(&a1->lock);
	applog(LOG_DEBUG, "A1 running queue_full: %d/%d",
	       a1->active_wq.num_elems, a1->num_chips);

	if (a1->active_wq.num_elems >= a1->num_chips) {
	//if (a1->active_wq.num_elems >= 20) {
		applog(LOG_DEBUG, "active_wq full");
		queue_full = true;
	} else {
		wq_enqueue(&a1->active_wq, get_queued(cgpu));
		//printf("A1_queue is fill\n");
	}
	mutex_unlock(&a1->lock);

	return queue_full;
}

void A1_flush_work(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	applog(LOG_DEBUG, "A1 running flushwork");
	unsigned char  work_completed_flag = 0;
	int i,p;
	//work_s = 0;
	if(a1->chips == NULL)
		return;

	mutex_lock(&a1->lock);
	/* stop chips hashing current work */
	if (!abort_work(a1)) {
		applog(LOG_ERR, "failed to abort work in chip chain!");
	}
	/* flush the work chips were currently hashing */
#if 0
	for (i = 0; i < a1->num_chips; i++) {
		int j;
		struct A1_chip *chip = &a1->chips[i];
		for (j = 0; j < 4; j++) {
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "flushing chip %d, work %d: 0x%p",
			       i, j + 1, work);
			if(work_completed_flag == 0)
			{
				work_completed_flag = 1;
				//printf("work_completed is ok\n");
				work_completed(cgpu, work);
			}
			chip->work[j] = NULL;
		}
	}
#endif
	work_get_flag[a1->spi_ctx->config.gpio_cs]=0;
	chip_no_work_count[a1->spi_ctx->config.gpio_cs] = 0;
	for(i=0;i<a1->num_chips; i++)
	{
		if(a1->work_cur[i]!=NULL)
		{
			a1->work_cur[i]=NULL;
			if(a1->work_start_delay)
			{
				a1->pool_reset_flag = 1;
				//p=100;
				//while(!cmd_RESETBC_BCAST(a1));
				//{
				//	p--;
				//	if(p==0)break;
				//}
			}
		}
	}
	if(a1->pool_reset_flag)
	{
		 	p=100;
                        while(!cmd_RESETBC_BCAST(a1))
                        {
                                        p--;
                                        if(p==0)break;
                        }

	}
	/* flush queued work */
	applog(LOG_DEBUG, "flushing queued work...");
#if 1
	while(a1->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&a1->active_wq);
		//assert(work != NULL);
		applog(LOG_DEBUG, "flushing 0x%p", work);
		if(work != NULL)work_completed(cgpu, work);
		//printf("flushing is ok\n");
	}
//	 printf("flushing is ok\n");
	
#endif
	mutex_unlock(&a1->lock);
}

void A1_flush_work_unlock(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	applog(LOG_DEBUG, "A1 running flushwork");

	int i;

	if(a1->chips == NULL)
		return;

	/* stop chips hashing current work */
	if (!abort_work(a1)) {
		applog(LOG_ERR, "failed to abort work in chip chain!");
	}
	/* flush the work chips were currently hashing */
#if 0
	for (i = 0; i < a1->num_chips; i++) {
		int j;
		struct A1_chip *chip = &a1->chips[i];
		for (j = 0; j < 4; j++) {
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "flushing chip %d, work %d: 0x%p",
			       i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}
	}
#endif 
	/* flush queued work */
	applog(LOG_DEBUG, "flushing queued work...");
#if 0
	while (a1->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&a1->active_wq);
		//assert(work != NULL);
		applog(LOG_DEBUG, "flushing 0x%p", work);
		work_completed(cgpu, work);
	}
#endif
}

#ifdef API_STATE_EN
static void A1_reinit_device(struct cgpu_info *cgpu)
{
	int k=100;
	struct A1_chain *a1 = cgpu->device_data;

	a1->reset_bc = 1;
	a1->status = ASIC_BOARD_OK;

	applog(LOG_NOTICE, "Reinit board (cs%d)", a1->spi_ctx->config.gpio_cs);
	int i,num_chips =0;
	
	//work_cur=malloc(sizeof(struct work *)*20*6);
	
	//cmd_POWER_ON_BCAST(a1);
	//printf("st reset wait\n");
	//while(k--)
	//sleep(1);
	//printf("st reset ok\n");

#ifdef ST_MCU_EN
	if (!cmd_RESET_BCAST(a1))
		goto failure;

	if (!cmd_BIST_START(a1))
		goto failure;
#else
	if (!cmd_RESET_BCAST(a1) || !cmd_BIST_START(a1))
		goto failure;
#endif
	a1->num_chips = a1->spi_rx[3];
	applog(LOG_WARNING, "spidev%d.%d(cs%d): Found %d A2 chips",
	       a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
	       a1->spi_ctx->config.gpio_cs,a1->num_chips);

	a1->num_cores=0;

	if (a1->num_chips == 0)
		goto failure;

	if (!cmd_BIST_COLLECT_BCAST(a1))	// modified for a2
		goto failure;

	if (!cmd_BIST_FIX_BCAST(a1))
		goto failure;

	if(a1->num_chips<MAX_ASIC_NUMS)
		a1->status|=ERROR_CHIP;

	for (i = 0; i < a1->num_chips; i++) {
		int chip_id = i + 1;
		if (!cmd_READ_REG(a1, chip_id)) {
			applog(LOG_WARNING, "Failed to read register for "
			       "chip %d -> disabling", chip_id);
			a1->chips[i].num_cores = 0;
			continue;
		}
		num_chips++;
		a1->chips[i].num_cores = a1->spi_rx[13];		// modified for a2
		if(a1->chips[i].num_cores>MAX_CORE_NUMS)		// modified for a2
			a1->chips[i].num_cores = MAX_CORE_NUMS;

		if(a1->chips[i].num_cores<MAX_CORE_NUMS)
			a1->status=ERROR_CORE;

		memcpy(a1->chips[i].reg, a1->spi_rx + 2, 12);	//keep ASIC register value	// modified for a2
#ifdef A1_TEMP_EN	
		if(i < 4) {
			a1->chips[i].temp= (a1->spi_rx[14]<<8)|a1->spi_rx[15];	// modified for a2
		}
#endif

		a1->num_cores += a1->chips[i].num_cores;
		//applog(LOG_WARNING, "Found chip %2d with %2d active cores",
		 //      chip_id, a1->chips[i].num_cores);

		a1->chips[i].reset_errors = 0;
	}
	applog(LOG_WARNING, "Found %d chips with total %d active cores",
	       a1->num_chips, a1->num_cores);

	if(a1->num_cores==0) // The A1 board  haven't any cores
		goto failure;		
	//printf("pll is %d\n",A1Pll1);
	switch(cgpu->device_id){
		case 0:A1_SetA1PLLClock(a1, A1Pll1);break;
		case 1:A1_SetA1PLLClock(a1, A1Pll2);break;
		case 2:A1_SetA1PLLClock(a1, A1Pll3);break;
		case 3:A1_SetA1PLLClock(a1, A1Pll4);break;
		case 4:A1_SetA1PLLClock(a1, A1Pll5);break;
		case 5:A1_SetA1PLLClock(a1, A1Pll6);break;
		default:;
	}

	return;

failure:
	applog(LOG_NOTICE, "Reinit board failure (cs%d)", a1->spi_ctx->config.gpio_cs);

	// Set zero cores before check num chips
	for (i = 0; i < a1->num_chips; i++)
		a1->chips[i].num_cores = 0;
	a1->num_cores = 0;

	return;
}

void A1_reinit(void)
{
	int i;
	struct A1_chain *a1;
	struct cgpu_info *cgpu;

	applog(LOG_WARNING, "A1_reinit");

	testFirstTime=false;


	rd_lock(&devices_lock);
	for (i = 0; i < total_devices; ++i)
	{
		cgpu=devices[i];
		a1=cgpu->device_data;

		applog(LOG_WARNING, "%d,%d,%d",cgpu->deven,cgpu->status,a1->status);
		cgpu->deven = DEV_RECOVER;
//		a1->reinit=1;
		cgpu->restart=1;

	}
	rd_unlock(&devices_lock);
	

}

static void A1_get_statline(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int i;
	char strT[100],*pStrT;
	float avgTemp=0,maxTemp=0;
	bool overheat=false;

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also


	if (a1->chips == NULL) 
	{
//		tailsprintf(buf, bufsiz, " | T:--C");
		return;
	}

	if(cgpu->deven != DEV_ENABLED)
		return;


	avgTemp=a1->chips[0].temp;
	pStrT=strT;
	sprintf(pStrT,"%02d",a1->chips[0].temp);
	pStrT+=2;


	for(i=1;i<a1->num_chips;i++)
	{
		if(maxTemp<a1->chips[i].temp)
			maxTemp=a1->chips[i].temp;

		if(a1->cutTemp<a1->chips[i].temp)
			overheat=true;
			
		avgTemp+=a1->chips[i].temp;
		sprintf(pStrT,"-%02d",a1->chips[i].temp);		
		pStrT+=3;

	}
	avgTemp/=a1->num_chips;

	if(a1->shutdown)
		tailsprintf(buf, bufsiz, " | T:%2.0fC(%s)(shutdown=%d)", maxTemp,strT,a1->shutdown);
	else if(overheat)
		tailsprintf(buf, bufsiz, " | T:%2.0fC*Hot*(%s)", maxTemp,strT);
	else	
		tailsprintf(buf, bufsiz, " | T:%2.0fC     (%s)", maxTemp,strT);


}

static void A1_statline_before(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	char strT[100],*pStrT;
	int i;
	
	if (a1->chips == NULL) 
		return;

	if(cgpu->deven != DEV_ENABLED)
		return;



#ifdef USE_GPIO_CS	
#if 1
	pStrT=strT;

	sprintf(pStrT,"cs:%d A:%d");

	if(a1->num_chips<MAX_ASIC_NUMS)
		sprintf(pStrT,"cs:%d A:%d*-C:%03d ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
	else if(a1->num_cores<MAX_CORE_NUMS*MAX_ASIC_NUMS)
		sprintf(pStrT,"cs:%d A:%d -C:%03d*", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
	else
		sprintf(pStrT,"cs:%d A:%d -C:%03d ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);

	pStrT+=15;
	if(a1->num_chips)
	{
		sprintf(pStrT," (%02d", a1->chips[i].num_cores);
		pStrT+=4;

		for(i=1;i<a1->num_chips;i++)
		{
				sprintf(pStrT,"-%02d", a1->chips[i].num_cores);
				pStrT+=3;
		}
		sprintf(pStrT,")");
		pStrT+=1;
	}
	sprintf(pStrT," | ");
	pStrT+=3;

	tailsprintf(buf, bufsiz, "%s", strT);

#else
		if(a1->num_chips<MAX_ASIC_NUMS)
			tailsprintf(buf, bufsiz, "cs:%d A:%dE-C:%03d | ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
		else if(a1->num_cores<MAX_CORE_NUMS)
			tailsprintf(buf, bufsiz, "cs:%d A:%d-C:%03dE | ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
		else
			tailsprintf(buf, bufsiz, "cs:%d A:%d-C:%03d | ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
#endif		
//	}

#else
#ifdef A1_TEST_MODE_EN
	if(testMode>0)
	{
		if(a1->num_chips<MAX_ASIC_NUMS)
			tailsprintf(buf, bufsiz, "A:%dE-C:%03d | ", a1->num_chips, a1->num_cores);
		else if(a1->num_cores<MAX_CORE_NUMS)
			tailsprintf(buf, bufsiz, "A:%d-C:%03dE | ", a1->num_chips, a1->num_cores);
		else
			tailsprintf(buf, bufsiz, "A:%d-C:%03d | ", a1->num_chips, a1->num_cores);
	}
	else
#endif
	{
		tailsprintf(buf, bufsiz, "A:%d-C:%03d | ", a1->num_chips, a1->num_cores);
	}
#endif
}

#ifdef API_STATE_EN
static struct api_data *A1_api_stats(struct cgpu_info *cgpu)
{
	struct api_data *root = NULL;
	struct A1_chain *a1 = cgpu->device_data;
	int i;
	char strT[100],*pStrT;
	float avgTemp=0;
	int num_chips=0;

	if (a1->chips == NULL) 
	{
		root = api_add_string(root, "Temperature", strT, false);
		return root ;
	}

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also
#ifdef USE_GPIO_CS	
	root = api_add_int(root, "CS", (int*)&(a1->spi_ctx->config.gpio_cs), false);
#endif
	root = api_add_int(root, "ASIC", &(a1->num_chips), false);
	//root = api_add_int(root, "CORES(TOTAL)", &(a1->num_cores), false);

	pStrT=strT;
        sprintf(pStrT,"%02d",a1->chips[0].num_cores);
        pStrT+=2;

        for(i=1;i<a1->num_chips;i++)
        {
                if(a1->chips[i].num_cores)
                {
                        //sprintf(pStrT,"-%02d",a1->chips[i].num_cores);
                }
                else
                        sprintf(pStrT,"- F");

                pStrT+=3;
        }

	//root = api_add_string(root, "CORES(SOLO)", strT, true);

	pStrT=strT;
	if(a1->chips[0].temp)
        {
		num_chips++;
         	avgTemp+=a1->chips[0].temp;
         	sprintf(pStrT,"%02d",a1->chips[0].temp);
    	}
        else
        	sprintf(pStrT,"%02d",0);

	pStrT+=2;

	//for(i=1;i<a1->num_chips;i++)
	for(i=1;i<1;i++)
	{
		if(a1->chips[i].temp)
		{
			num_chips++;
			avgTemp+=a1->chips[i].temp;
			sprintf(pStrT,"-%02d",a1->chips[i].temp);
		}
		else
			sprintf(pStrT,"-%02d",0);
		
		pStrT+=3;
	}
	avgTemp=a1->chips[0].temp;///=num_chips;
	root = api_add_temp(root, "TEMP(AVG)", &avgTemp, true);
	root = api_add_string(root, "TEMP(SOLO)", strT, true);

	return root;
}
#endif

#ifdef HAVE_CURSES

#define CURBUFSIZ 256
#define cg_mvwprintw(win, y, x, fmt, ...) do { \
	char tmp42[CURBUFSIZ]; \
	snprintf(tmp42, sizeof(tmp42), fmt, ##__VA_ARGS__); \
	mvwprintw(win, y, x, "%s", tmp42); \
} while (0)
#define cg_wprintw(win, fmt, ...) do { \
	char tmp42[CURBUFSIZ]; \
	snprintf(tmp42, sizeof(tmp42), fmt, ##__VA_ARGS__); \
	wprintw(win, "%s", tmp42); \
} while (0)


extern WINDOW *mainwin, *statuswin, *logwin;


void A1_curses_print_status(int y)
{

//	applog(LOG_WARNING, "A1_curses_print_status");
	struct A1_chain *a1;
	struct cgpu_info *cgpu;
	int i,testing=0,errHw=0,errBoard=0,errOverheat=0,errChip=0,errCore=0,errSenor=0,numChips=0;


	char strT[100],*pStrT;

	if(testFirstTime)
	{
		y+=2;
		cg_mvwprintw(statuswin, y, 1, "Press 'R' key for hardware testing");
		return;
	}


#if 1
	//Note: not allow write any data to  devices[i]
	rd_lock(&devices_lock);
	for (i = 0; i < total_devices; ++i)
	{
		cgpu=devices[i];
		a1=cgpu->device_data;

		if(cgpu->deven == DEV_RECOVER)
		{
			testing=1;
			break;
		}
		if(a1->status&(ERROR_BOARD|ERROR_CHIP|ERROR_CORE|ERROR_TEMP_SENSOR))
		{
			errHw=1;

			if(a1->status&ERROR_CHIP)
				errChip=1;
			if(a1->status&ERROR_CORE)
				errCore=1;
			if(a1->status&ERROR_TEMP_SENSOR)
				errSenor=1;
			if(a1->status&ERROR_BOARD)
				errBoard=1;
			if(a1->status&ERROR_OVERHEAD)
				errOverheat=1;
		}

		

		if(a1->status==ASIC_BOARD_OK)
		{
			numChips++;
		}

	}
	rd_unlock(&devices_lock);
#endif

	if(numChips<MAX_ASIC_BOARD)
	{
//		errHw=1;
	//	errBoard=1;
	}

	++y;
	mvwhline(statuswin, y, 0, '-', 80);

	if(total_devices)
	{
		if(testing)
			cg_mvwprintw(statuswin, ++y, 1, "Testing .....");
		else if(errHw)
		{
			pStrT=strT;
			sprintf(pStrT,"Hardware: ");
			pStrT+=strlen("Hardware: ");			
			if(errBoard)
			{
				sprintf(pStrT,"ASIC board error,");
				pStrT+=strlen("ASIC board error,");
			}
			if(errChip)
			{
				sprintf(pStrT,"Chip error,");
				pStrT+=strlen("Chip error,");
			}

			if(errCore)
			{
				sprintf(pStrT,"Core error,");
				pStrT+=strlen("Core error,");
			}

			if(errSenor)
			{
				sprintf(pStrT,"Senor error,");
				pStrT+=strlen("Senor error,");
			}

			if(errOverheat)
			{
				sprintf(pStrT,"Overheat");
				pStrT+=strlen("Overheat");
			}

			cg_mvwprintw(statuswin, ++y, 1, "%s",strT);
		}
		else
			cg_mvwprintw(statuswin, ++y, 1, "Hardware testing: Passed");	
	}
	else
		cg_mvwprintw(statuswin, ++y, 1, "Test finished: No any ASIC board");	

	wclrtoeol(statuswin);
	++y;
	mvwhline(statuswin, y, 0, '-', 80);
	//wattroff(statuswin, A_BOLD);
	wclrtoeol(statuswin);
}

#endif

void A1_stop_test(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
//    a1->status  = (a1->status & (~ASIC_BOARD_TESTING));
}
static bool A1_get_stats(struct cgpu_info *cgpu)
{

	struct A1_chain *a1 = cgpu->device_data;
	int i,overheatCnt;
	char strT[100],*pStrT;
	float avgTemp=0,maxTemp=0;
	bool overheat=false;

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also

	if (a1->chips == NULL) 
	{
//		cgpu->deven=DEV_DISABLED;
		//cgpu->status= LIFE_NOSTART;
//		a1->status=NO_ASIC_BOARD;
		return true;
	}


	if(a1->num_chips<MAX_ASIC_NUMS)
	{
//		cgpu->status = LIFE_DEAD;
		a1->status|=ERROR_CHIP;
	}
	else
	{
		if(a1->num_cores<(MAX_ASIC_NUMS*MAX_CORE_NUMS))
		{
//			cgpu->status = LIFE_SICK;
			a1->status|=ERROR_CORE;
		}

		for(i=0;i<a1->num_chips;i++)
		{
			if(a1->chips[i].temp==0)
			{
//				cgpu->status = LIFE_SICK;
				a1->status|=ERROR_TEMP_SENSOR;				
			}
		}
	}

	return true;
}
#endif

struct device_drv bitmineA1_drv = {
	.drv_id = DRIVER_bitmineA1,
	.dname = "BA2",
	.name = "BA2",
	.drv_detect = A1_detect,

	.hash_work = hash_queued_work,
	.scanwork = A1_scanwork,
	.queue_full = A1_queue_full,
	.flush_work = A1_flush_work,
	.reinit_device = A1_reinit_device,
#ifdef USE_ST_MCU
	.get_statline_before = A1_statline_before,	
	.get_statline= A1_get_statline,	
	.get_stats= A1_get_stats,
#ifdef API_STATE_EN
	.get_api_stats = A1_api_stats,
#endif	
#endif	
};
