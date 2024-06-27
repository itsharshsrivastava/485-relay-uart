#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

/* Get the UART device specified in the device tree */
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* Function to initialize UART with specific settings */
void uart_init(void)
{
	struct uart_config uart_cfg;

	/* Set UART configuration parameters */
	uart_cfg.baudrate = 9600;
	uart_cfg.parity = UART_CFG_PARITY_NONE;
	uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
	uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
	uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	/* Configure the UART device with the settings */
	int ret = uart_configure(uart_dev, &uart_cfg);
	if (ret != 0) {
		printk("Failed to configure UART: %d", ret);
	}
}

int main(void)
{
	if (!device_is_ready(uart_dev)) { 		// Check if the UART device is ready
		printk("UART device not found!");
		return 0;
	}

	uart_init(); 	// initialise UART with specified settings

	/* Define the on and off signals to be sent via UART 
	 	
		signal = {
  		 1:deviceId/slaveID, 
                 2:function ("Write Single Coil"-->0x05),
                 3,4: coil address/channel number,
                 5,6: coil state (0xFF00 --> ON and 0x0000 --> OFF),
                 7,8: Cyclic Redundancy Check for error detection
		}
  	*/
	
	uint8_t on_signal1[8] = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A};		// hex code to turn on channel 1

	uint8_t off_signal1[8] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA};		// hex code to turn off channel 1

	uint8_t on_signal2[8] = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA};		// hex code to turn on channel 2

	uint8_t off_signal2[8] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A};		// hex code to turn off channel 2

	while (1) {
		/* Send each byte of the on_signal1 array via UART */
		for (int i = 0; i < sizeof(on_signal1); i++) {
			uart_poll_out(uart_dev, on_signal1[i]);		//  turn on channel 1
			k_msleep(1);                            	// Small delay for each byte
		}
		k_msleep(1000);

		/* Send each byte of the on_signal2 array via UART */
		for (int i = 0; i < sizeof(off_signal1); i++) {
			uart_poll_out(uart_dev, off_signal1[i]); 	//  turn off channel 1
			k_msleep(1);                             	// Small delay for each byte
		}
		k_msleep(1000);

		/* Send each byte of the on_signal2 array via UART */
		for (int i = 0; i < sizeof(on_signal2); i++) {
			uart_poll_out(uart_dev, on_signal2[i]); 	//  turn on channel 2
			k_msleep(1);                            	// Small delay for each byte
		}
		k_msleep(1000);

		/* Send each byte of the on_signal2 array via UART */
		for (int i = 0; i < sizeof(off_signal2); i++) {
			uart_poll_out(uart_dev, off_signal2[i]); 	//  turn off channel 2
			k_msleep(1);                             	// Small delay for each byte
		}
		k_msleep(1000);
	}
	return 0;
}
