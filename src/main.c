/*
* Copyright (c) 2012-2014 Wind River Systems, Inc.
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/spi.h>

#include "include/ad411x_regs.h"
#include "include/no_os_spi.h"

#include "include/ad717x.h"


/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define rs485_enable DT_ALIAS(rs485enable)


/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec rs485en = GPIO_DT_SPEC_GET(rs485_enable, gpios);


void main(void)
{
	printk("Program Init\n");

	/* Other variables */
	int timeout = 1000;
	int8_t ret;
	uint32_t sample;

	/* Create a new driver instance */
	ad717x_dev* ad411x_dev;
	ad717x_init_param ad411x_init;

	ad411x_init.active_device = ID_AD4111;
	ad411x_init.regs = ad4111_regs;
	ad411x_init.useCRC = AD717X_DISABLE;
	ad411x_init.num_regs = sizeof(ad4111_regs) / sizeof(ad4111_regs[0]);
	ad411x_init.num_setups = 1; //Num of ADC channel setups, Setup0 0-10V, Setup1 4-20mA
	ad411x_init.num_channels = 12; //Use 12 channels
	ad411x_init.mode = CONTINUOUS; //Continuos read mode

	// ADC Setup
	//Setup configuration for Voltage input 0-10V, Setup0
	ad411x_init.setups[0].bi_unipolar = false; //Set all channels to unipolar
	ad411x_init.setups[0].ref_buff = false; //Disable Reference Buffer
	ad411x_init.setups[0].input_buff = true; //Enable Input Buffer
	ad411x_init.setups[0].ref_source = EXTERNAL_REF; //Set channel Reference Source to EXTERNAL 2.5V REF
	ad411x_init.filter_configuration[0].odr = sps_503; //503 SPS Output data rate

	//Setup configuration for Voltage input 0-10V, Setup1
	ad411x_init.setups[1].bi_unipolar = false; //Set all channels to unipolar
	ad411x_init.setups[1].ref_buff = false; //Disable Reference Buffer
	ad411x_init.setups[1].input_buff = false; //Input Buffer disabled for 4-20mA inputs
	ad411x_init.setups[1].ref_source = EXTERNAL_REF; //Set channel Reference Source to EXTERNAL 2.5V REF
	ad411x_init.filter_configuration[1].odr = sps_503; //503 SPS Output data rate

	// TODO: unused 4-20mA gives overrange error, check
	//Disabled for now

	//Config LED
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	//Config RS485 Enable
	ret = gpio_pin_configure_dt(&rs485en, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	/* Initialize the driver instance */
	ret = AD717X_Init(&ad411x_dev, ad411x_init);
	if (ret < 0) {
		printk("bobop1\n");
	}

	while (1) {
		// /* Read data from the ADC */
		ret = AD717X_WaitForReady(ad411x_init, timeout);
		if (ret < 0)
			printk("bobop2\n");
		/*
			Data Output calc :
			Code = (2^N × VIN × 0.1)/VREF
			The output code for any input current is represented as
			Code = (2^N × IIN × 50 Ω)/VREF
			Our Side: 5.02V input = 5.2925944 in code, adjust offset
		*/
		//Voltage Input (V) = Data * 0.000001490116119384765625
		//TODO: Print all channels data
		AD717X_ReadData(ad411x_init, &sample);

		// printk("CRUDE CHN = %d, S Read: %d\n", ret, sample);

		int channel = sample & 0x0F;
		sample = sample >> 7;

		printk("CHN = %d, S Read: %d\n", channel, sample);

		// k_sleep(K_MSEC(5));
	}
}
