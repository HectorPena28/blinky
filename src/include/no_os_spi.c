/***************************************************************************//**
 *   @file   no_os_spi.c
 *   @brief  Implementation of the SPI Interface
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <inttypes.h>
#include "no_os_spi.h"
#include <stdlib.h>

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/spi.h>

#include "ad717x.h"

/* The devicetree node identifier for the "csadc" alias. */
#define cs_adc_pin DT_ALIAS(csadc)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec csadc = GPIO_DT_SPEC_GET(cs_adc_pin, gpios);

/* The devicetree node identifier for the "led0" alias. */
// #define LED0_NODE DT_ALIAS(led0)
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
			 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 4000000,
	.slave = 0,
};

struct device* spi_dev;

uint8_t spi_send(uint8_t tx_value)
{
	int err;
	static uint8_t tx_buffer[1];
	static uint8_t rx_buffer[1];

	tx_buffer[0] = tx_value;

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
	if (err < 0) {
		printk("SPI error: %d\n", err);
		return SPI_COMM_ERR+err;
	}
	// else {
	// 	/* Connect MISO to MOSI for loopback */
	// 	printk("TX sent: %x\n", tx_buffer[0]);
	// 	printk("RX recv: %x\n", rx_buffer[0]);
	// }

	return rx_buffer[0];
}


/**
 * @brief Initialize the SPI communication peripheral.
 * @return 0 in case of success, -1 otherwise.
 */
uint32_t spi_init(void)
{
	uint32_t ret = 0;

	//CONFIG CS pin of output Chip
	ret = gpio_pin_configure_dt(&csadc, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	const char* const spiName = "SPI_1";
	spi_dev = device_get_binding(spiName);

	if (spi_dev == NULL) {
		printk("Could not get %s device\n", spiName);
		ret = -1;
	}
	return ret;
}

/**
 * Write and read data to/from SPI.
 *
 * @param [in] desc         - The SPI descriptor.
 * @param [in/out] data     - The buffer with the transmitted/received data.
 * @param [in] bytes_number - Number of bytes to write/read.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t spi_write_and_read(uint8_t* data, uint8_t bytes_number)
{
	int32_t err = 0;

	static uint8_t totalData[24];

	// //Debugg print
	// printk("SPI Write: data: ");
	// for (int i = 0; i < bytes_number;i++) {
	// 	printk("%02x, ", data[i]);
	// }
	// printk("bytes: %d\n", bytes_number);
	// //Debugg print

	//cs pin active
	gpio_pin_set_dt(&csadc, 1);

	for (int i = 0; i < bytes_number;i++) {
		totalData[i] = spi_send(data[i]);
		*(data + i) = totalData[i];
		if (totalData[i] < 0) //Error Check
			return totalData[i];
	}

	//cs pin inactive
	gpio_pin_set_dt(&csadc, 0);

	// // Debugg print
	// printk("Data Received: ");
	// for (int i = 0; i < bytes_number;i++) {
	// 	printk("%x, ", *(data + i));
	// }
	// printk("\n");

	return err;
}
