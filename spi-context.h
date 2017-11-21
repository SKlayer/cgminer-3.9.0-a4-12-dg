#ifndef SPI_CONTEXT_H
#define SPI_CONTEXT_H

#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "bcm2835.h"

#define SPI_DEVICE_TEMPLATE		"/dev/spidev%d.%d"
#define DEFAULT_SPI_BUS			0
#define DEFAULT_SPI_CS_LINE		0
#define DEFAULT_SPI_MODE		SPI_MODE_0
#define DEFAULT_SPI_BITS_PER_WORD	8
#define DEFAULT_SPI_SPEED		4000000 //4MHz
#define DEFAULT_SPI_DELAY_USECS		0


#define KEY_EVENT_NULL 0
#define KEY_EVENT_RESET 1

#ifdef USE_GPIO_CS

#define TEST_KEY_EN //Send Siginal to ASIC board if user press test key more than 2 sec


typedef enum
{
    G_CS_0=0,
    G_CS_1,
    G_CS_2,
    G_CS_3,
    G_CS_4,
    G_CS_5,
    G_CS_6,
    G_CS_7,
    G_CS_MAX,

} G_CS_t;
#endif


struct spi_config {
	int bus;
	int cs_line;
	uint8_t mode;
	uint32_t speed;
	uint8_t bits;
	uint16_t delay;
#ifdef USE_GPIO_CS
	uint8_t gpio_cs;
#endif
};

static const struct spi_config default_spi_config = {
	.bus		= DEFAULT_SPI_BUS,
	.cs_line	= DEFAULT_SPI_CS_LINE,
	.mode		= DEFAULT_SPI_MODE,
	.speed		= DEFAULT_SPI_SPEED,
	.bits		= DEFAULT_SPI_BITS_PER_WORD,
	.delay		= DEFAULT_SPI_DELAY_USECS,
};

struct spi_ctx {
	int fd;
	struct spi_config config;
};

/* create SPI context with given configuration, returns NULL on failure */
extern struct spi_ctx *spi_init(struct spi_config *config);
/* close descriptor and free resources */
extern void spi_exit(struct spi_ctx *ctx);
/* process RX/TX transfer, ensure buffers are long enough */
extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len);


#ifdef USE_GPIO_CS
extern int gpio_CS_Init(void);
extern void pabort(const char *s);
extern uint8_t KeyDetect(void);
#endif
extern bool CheckForAnotherInstance();


#endif /* SPI_CONTEXT_H */
