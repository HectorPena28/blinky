#include <device.h>
#include <toolchain.h>

/* 1 : /soc/clock@40000000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_clock_40000000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 2 : /soc/gpio@50000300:
 * Direct Dependencies:
 *   - (/soc)
 * Supported:
 *   - (/plcinputs/input_1)
 *   - (/plcinputs/input_2)
 *   - (/plcinputs/input_4)
 *   - (/plcinputs/input_5)
 *   - (/plcoutputchip/dis_pin)
 *   - (/spi_chipselect/cs_flashmemory)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_gpio_50000300[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 3 : /soc/gpio@50000000:
 * Direct Dependencies:
 *   - (/soc)
 * Supported:
 *   - (/leds/led_0)
 *   - (/leds/led_1)
 *   - (/plcinputs/input_0)
 *   - (/plcinputs/input_3)
 *   - (/plcinputs/input_6)
 *   - (/plcinputs/input_7)
 *   - (/plcoutputchip/cs_outputchip)
 *   - (/plcoutputchip/diag_pin)
 *   - (/spi_chipselect/CS_ADC)
 *   - (/spi_chipselect/cs_rtcc)
 *   - (/spi_chipselect/rs485_enable)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_gpio_50000000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 4 : /soc/uart@40028000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/pin-controller/uart1_default)
 *   - (/pin-controller/uart1_sleep)
 *   - (/soc/interrupt-controller@e000e100)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_uart_40028000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 5 : /soc/uart@40002000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/pin-controller/uart0_default)
 *   - (/pin-controller/uart0_sleep)
 *   - (/soc/interrupt-controller@e000e100)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_uart_40002000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 6 : /soc/spi@40004000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/pin-controller/spi1_default)
 *   - (/pin-controller/spi1_sleep)
 *   - (/soc/interrupt-controller@e000e100)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40004000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };
