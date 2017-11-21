/*
 * generic SPI functions
 *
 * Copyright 2013 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>


#include "logging.h"
#include "miner.h"

#include "spi-context.h"

//#define USE_ST_MCU

#ifdef USE_GPIO_CS
#define GPIO_SPI_CS_EN //using GPIO as chip select

#define ASIC_SPI_CS0  	RPI_V2_GPIO_P1_11
#define ASIC_SPI_CS1  	RPI_V2_GPIO_P1_12
#define ASIC_SPI_CS2  	RPI_V2_GPIO_P1_13
#define ASIC_SPI_EN		RPI_V2_GPIO_P1_22

static pthread_mutex_t gpio_lock;

#endif //USE_GPIO_CS


#ifdef TEST_KEY_EN

#define MAX_KEY	2

#define ASIC_TEST_KEY	RPI_V2_GPIO_P1_11
#define ASIC_RESET_KEY	RPI_V2_GPIO_P1_13

static pthread_mutex_t key_lock;

struct key_config {
	struct timeval tvStart;
	uint8_t pin;
	uint8_t event;
	uint8_t holdSec; // key hold time in sec
};
struct key_config keys[MAX_KEY]={
	{.pin=ASIC_TEST_KEY,.event=KEY_EVENT_NULL,.holdSec=2},
	{.pin=ASIC_RESET_KEY,.event=KEY_EVENT_NULL,.holdSec=2}
};

uint8_t KeyDetectInit(void);
	
#endif //TEST_KEY_EN


void pabort(const char *s)
{
    perror(s);
    abort();
}
#ifdef GPIO_SPI_CS_EN 
static void gpio_CS_Setoutput(void)
{
    // Function selects are 10 pins per 32 bit word, 3 bits per pin
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPFSEL0/4 + 1;
    volatile uint32_t* paddr2 = bcm2835_gpio + BCM2835_GPFSEL0/4 + 2;    //2 tango  add
    uint32_t  mask = BCM2835_GPIO_FSEL_MASK << ((ASIC_SPI_CS0 % 10) * 3)|\
	       	     BCM2835_GPIO_FSEL_MASK << ((ASIC_SPI_CS1 % 10) * 3);
		     //BCM2835_GPIO_FSEL_MASK << ((ASIC_SPI_CS2 % 10) * 3);
    uint32_t  value = (BCM2835_GPIO_FSEL_OUTP << ((ASIC_SPI_CS0 % 10) * 3))| \
		      (BCM2835_GPIO_FSEL_OUTP << ((ASIC_SPI_CS1 % 10) * 3));
		      //(BCM2835_GPIO_FSEL_OUTP << ((ASIC_SPI_CS2 % 10) * 3));
	uint32_t  mask2 = BCM2835_GPIO_FSEL_MASK << ((ASIC_SPI_CS2 % 10) * 3);
	uint32_t  value2 = (BCM2835_GPIO_FSEL_OUTP << ((ASIC_SPI_CS2 % 10) * 3));
    bcm2835_peri_set_bits(paddr, value, mask);
    bcm2835_peri_set_bits(paddr2, value2, mask2);
}

static int gpio_init=0;
int gpio_CS_Init(void)
{

//    if(opt_gcs<0)
  //      return;

	if(gpio_init)
		return 1;
	
	gpio_init=1;

	mutex_init(&gpio_lock);

    if (!bcm2835_init()) {
        pabort("gpio initial fail");
    }

    bcm2835_gpio_fsel(ASIC_SPI_EN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ASIC_SPI_EN, LOW);	

    bcm2835_gpio_fsel(ASIC_SPI_CS0, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ASIC_SPI_CS0, LOW);

    bcm2835_gpio_fsel(ASIC_SPI_CS1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ASIC_SPI_CS1, LOW);

    bcm2835_gpio_fsel(ASIC_SPI_CS2, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ASIC_SPI_CS2, LOW);

#ifdef TEST_KEY_EN
	KeyDetectInit();
#endif
    return 0;

}

// 0->2, 1_1, 2-0 ,3-5,4-6,5-4
#if 0
#define GPIO_CS_MASK 	(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS_CLR 	GPIO_CS_MASK

#define SET_CS0_BIT0 	GPIO_CS_CLR
#define SET_CS0_BIT1 	(0<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS0		SET_CS0_BIT1

#define SET_CS1_BIT0 	GPIO_CS_CLR
#define SET_CS1_BIT1 	(1<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS1		SET_CS1_BIT1

#define SET_CS2_BIT0 	GPIO_CS_CLR
#define SET_CS2_BIT1 	(0<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS2		SET_CS2_BIT1

#define SET_CS3_BIT0 	GPIO_CS_CLR
#define SET_CS3_BIT1 	(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS3		SET_CS3_BIT1

#define SET_CS4_BIT0 	GPIO_CS_CLR
#define SET_CS4_BIT1 	(0<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS4		SET_CS4_BIT1

#define SET_CS5_BIT0 	GPIO_CS_CLR
#define SET_CS5_BIT1 	(0<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS5		SET_CS5_BIT1


#define SET_CS6_BIT0 	GPIO_CS_CLR
#define SET_CS6_BIT1 	(1<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS6		SET_CS6_BIT1

#define SET_CS7_BIT0 	GPIO_CS_CLR
#define SET_CS7_BIT1 	(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS7		SET_CS7_BIT1

#else
#define GPIO_CS_MASK 	(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS_CLR 	GPIO_CS_MASK

#define SET_CS0_BIT0 	GPIO_CS_CLR
#define SET_CS0_BIT1 	(0<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS0		SET_CS0_BIT1

#define SET_CS1_BIT0 	GPIO_CS_CLR
#define SET_CS1_BIT1 	(1<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS1		SET_CS1_BIT1

#define SET_CS2_BIT0 	GPIO_CS_CLR
#define SET_CS2_BIT1 	(0<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS2		SET_CS2_BIT1

#define SET_CS3_BIT0 	GPIO_CS_CLR
#define SET_CS3_BIT1 	(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS3		SET_CS3_BIT1


#define SET_CS4_BIT0 	GPIO_CS_CLR
#define SET_CS4_BIT1 	(0<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS4		SET_CS4_BIT1


#define SET_CS5_BIT0 	GPIO_CS_CLR
#define SET_CS5_BIT1 	(1<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS5		SET_CS5_BIT1


#define SET_CS6_BIT0 	GPIO_CS_CLR
#define SET_CS6_BIT1 	(0<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS6		SET_CS6_BIT1

#define SET_CS7_BIT0 	GPIO_CS_CLR
#define SET_CS7_BIT1 	(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS7		SET_CS7_BIT1

#endif

struct cs_config {
	uint32_t clr;
	uint32_t set;
	uint32_t read; // key hold time in sec
};
struct cs_config CSs[G_CS_MAX]={
	{.clr=SET_CS0_BIT0,.set=SET_CS0_BIT1,.read=GPIO_CS0},
	{.clr=SET_CS1_BIT0,.set=SET_CS1_BIT1,.read=GPIO_CS1},
	{.clr=SET_CS2_BIT0,.set=SET_CS2_BIT1,.read=GPIO_CS2},
	{.clr=SET_CS3_BIT0,.set=SET_CS3_BIT1,.read=GPIO_CS3},
	{.clr=SET_CS4_BIT0,.set=SET_CS4_BIT1,.read=GPIO_CS4},
	{.clr=SET_CS5_BIT0,.set=SET_CS5_BIT1,.read=GPIO_CS5},
	{.clr=SET_CS6_BIT0,.set=SET_CS6_BIT1,.read=GPIO_CS6},
	{.clr=SET_CS7_BIT0,.set=SET_CS7_BIT1,.read=GPIO_CS7},
};

static void gpioChipSelectOn(G_CS_t cs)
{
    static G_CS_t preCs=G_CS_MAX;
    if(opt_gcs<0)
        return;

	uint32_t v;

	volatile uint32_t* paddrClr = bcm2835_gpio + BCM2835_GPCLR0/4;
	volatile uint32_t* paddrSet = bcm2835_gpio + BCM2835_GPSET0/4;
	volatile uint32_t* paddrRead = bcm2835_gpio + BCM2835_GPLEV0/4;
	uint8_t ret,loop=0;
	struct cs_config *csCfg;

    if (cs>=G_CS_MAX)
	    return; // out of range

    // make stable in chip select IC
    if (preCs==cs)
	    return; // not need change
	
	gpio_CS_Setoutput();

	csCfg=&CSs[cs];

	do{

		*paddrClr=csCfg->clr;
		*paddrClr=csCfg->clr;

		*paddrSet=csCfg->set;
		*paddrSet=csCfg->set;

		ret=((*paddrRead&GPIO_CS_MASK)!=csCfg->read);
		ret=((*paddrRead&GPIO_CS_MASK)!=csCfg->read);
		if(ret)
		{
			loop=1;
			applog(LOG_ERR, "Cannot set CS Pin(%d):0x%X\n", cs,*paddrRead&GPIO_CS_MASK);
		}

	}while(ret);

    // make stable in chip select IC
    if (preCs!=cs)
    {
        preCs=cs;
//        bcm2835_delayMicroseconds(1); // delay of about 80 microseconds due to system
//	cgsleep_ms(5); // 5 ms
//	cgsleep_ms(10); // OK
//	cgsleep_ms(7); // 5 ms 

    }
//	cgsleep_ms(10); // OK
	//cgsleep_ms(40); // OK	
	
//    bcm2835_gpio_write(ASIC_SPI_EN, LOW);		// 

    bcm2835_gpio_fsel(ASIC_SPI_EN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ASIC_SPI_EN, LOW);	

}

static void gpioChipSelectOff(void)
{
    if(opt_gcs<0)
        return;
//	volatile uint32_t* paddrSet = bcm2835_gpio + BCM2835_GPSET0/4;

//gpioChipSelectOn(G_CS_7);

// *paddrSet=(1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2);
// cgsleep_us(1);

 //   bcm2835_gpio_write(ASIC_SPI_EN, HIGH);	// 
}

#ifdef TEST_KEY_EN
uint8_t KeyDetectInit(void)
{
	int i;

	mutex_init(&key_lock);

	for(i=0;i<MAX_KEY;i++)
	{
		// Set RPI pin P1-15 to be an input
		bcm2835_gpio_fsel(keys[i].pin, BCM2835_GPIO_FSEL_INPT);
		//	with a pullup
		bcm2835_gpio_set_pud(keys[i].pin, BCM2835_GPIO_PUD_UP);
	}

}
uint8_t KeyDetect(void)
{
	struct timeval tvNow;
	struct timeval tvDiff;
	struct key_config *key;

	uint8_t i,ret=KEY_EVENT_NULL;

	if(pthread_mutex_unlock (&key_lock))
		return ret;
	
	mutex_lock(&key_lock);

	for(i=0;i<MAX_KEY;i++)
	{
		key=&keys[i];

		// Read some data
		if(bcm2835_gpio_lev(key->pin)) //no pressed
		{
			cgtime(&key->tvStart);
		}
		else
		{
			cgtime(&tvNow);

			timersub(&tvNow, &key->tvStart, &tvDiff);
			if(tvDiff.tv_sec>key->holdSec)
			{
				cgtime(&key->tvStart);
				applog(LOG_NOTICE, "Key(%d) pressed",key->pin);
				ret=key->event;
				break;
			}
		}

	}

	mutex_unlock(&key_lock);
	return ret;

}
#endif

#endif //GPIO_CS_EN

bool CheckForAnotherInstance()
{

#ifndef _MSC_VER // Linux only code

    int fd;
    struct flock fl;

    fd = open("/tmp/cgminer.lock", O_RDWR|O_CREAT);

    if (fd == -1)
        return false;

    fl.l_type   = F_WRLCK;  /* F_RDLCK, F_WRLCK, F_UNLCK    */
    fl.l_whence = SEEK_SET; /* SEEK_SET, SEEK_CUR, SEEK_END */
    fl.l_start  = 0;        /* Offset from l_whence         */
    fl.l_len    = 0;        /* length, 0 = to EOF           */
    fl.l_pid    = getpid(); /* our PID                      */

    // try to create a file lock
    if ( fcntl(fd, F_SETLK, &fl) == -1)  /* F_GETLK, F_SETLK, F_SETLKW */
    {
        // we failed to create a file lock, meaning it's already locked //

        if ( errno == EACCES || errno == EAGAIN)
            return false;

    }

#endif // Linux only code //

    return true;

}

struct spi_ctx *spi_init(struct spi_config *config)
{
	char dev_fname[PATH_MAX];
	struct spi_ctx *ctx;

	if (config == NULL)
		return NULL;

	sprintf(dev_fname, SPI_DEVICE_TEMPLATE, config->bus, config->cs_line);

	int fd = open(dev_fname, O_RDWR);
	if (fd < 0) {
		applog(LOG_ERR, "SPI: Can not open SPI device %s", dev_fname);
		return NULL;
	}

	if ((ioctl(fd, SPI_IOC_WR_MODE, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MODE, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config->speed) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config->speed) < 0)) {
		applog(LOG_ERR, "SPI: ioctl error on SPI device %s", dev_fname);
		close(fd);
		return NULL;
	}

	ctx = malloc(sizeof(*ctx));
	if (ctx == NULL) {
		applog(LOG_ERR, "SPI: can't allocate SPI context");
		close(fd);
		return NULL;
	}

	ctx->fd = fd;
	ctx->config = *config;
	applog(LOG_INFO, "SPI '%s': mode=%hhu, bits=%hhu, speed=%u", dev_fname,
	       ctx->config.mode, ctx->config.bits, ctx->config.speed);

	return ctx;
}

extern void spi_exit(struct spi_ctx *ctx)
{
	if (NULL == ctx)
		return;
	close(ctx->fd);
	free(ctx);
}

extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len)
{
#ifdef GPIO_SPI_CS_EN 
	mutex_lock(&gpio_lock);
	gpioChipSelectOn(ctx->config.gpio_cs);
#endif

	struct spi_ioc_transfer xfr;
	int ret;

	if (rxbuf != NULL)
		memset(rxbuf, 0xff, len);

	ret = len;

	xfr.tx_buf = (unsigned long)txbuf;
	xfr.rx_buf = (unsigned long)rxbuf;
	xfr.len = len;
	xfr.speed_hz = ctx->config.speed;
	xfr.delay_usecs = ctx->config.delay;
	xfr.bits_per_word = ctx->config.bits;
	xfr.cs_change = 0;
	xfr.pad = 0;

	ret = ioctl(ctx->fd, SPI_IOC_MESSAGE(1), &xfr);
	if (ret < 1)
		applog(LOG_ERR, "SPI: ioctl error on SPI device: %d", ret);

#ifdef GPIO_SPI_CS_EN 
		gpioChipSelectOff();
		mutex_unlock(&gpio_lock);
#endif

	return ret > 0;

}
