#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <zephyr/logging/log.h>

// #include "modbus_api_nRF52832.h"

LOG_MODULE_REGISTER(modbus,LOG_LEVEL_DBG);

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

/* Get the UART device specified in the device tree */
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* Function to initialize UART with specific settings */
void uart_init(void) {
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

uint16_t modbusRTU_CRC16(uint8_t buff[], int len) {
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)buff[i]; // XOR with the next byte

        for (int j = 0; j < 8; j++) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else { // Else LSB is not set
                crc >>= 1; // Just shift right
            }
        }
    }

    return crc; // Return the computed CRC-16
}

// Transmit request from master/client
void send_request(uint8_t signal[]) {
	// printk("Transmitting Request...");
	// k_msleep(1);
	for(int i = 0; i < 8; i++) {
		uart_poll_out(uart_dev, signal[i]);
		k_msleep(1);
	}
}

// Recieve the response from server/slave
// Reads the response and stores it into the holding registers
void read_response(uint16_t *const regs, int size) {
	int bytes_read = 0;

	while (bytes_read < size)
	{
		uint8_t data;
		uart_poll_in(uart_dev, &data);

		regs[bytes_read] = data;
		bytes_read += 1;
		k_msleep(1);
	}
	
}

// void modbus_read_coils(uint deviceID, uint16_t start_addr, uint8_t *const coil_tbl, const uint16_t coil_qty) {

// 	uint8_t FC = 0x01;
// 	uint8_t addr_low = start_addr & 0xFF;
// 	uint8_t addr_high = (start_addr >> 8) & 0xFF;

// }

// Read the holding register
void modbus_read_holding_regs(uint8_t deviceID, uint16_t start_addr, uint16_t *const reg_buf, uint16_t num_regs) {
	uint8_t FC = 0x03;
	uint8_t addr_low = start_addr & 0xFF;
	uint8_t addr_high = (start_addr >> 8) & 0xFF;

	uint8_t num_regs_low = num_regs & 0xFF;
	uint8_t num_regs_high = (num_regs >> 8) & 0xFF;

	uint8_t buff[6] = {deviceID, FC, addr_high, addr_low, num_regs_high, num_regs_low};

	uint16_t crc = modbusRTU_CRC16(buff, sizeof(buff));

	uint8_t signal[8] = {deviceID, FC, addr_high, addr_low,
				 		 num_regs_high, num_regs_low, 0x00, 0x00};

	signal[6] = crc & 0xFF; // CRC MSB
	signal[7] = (crc >> 8) & 0xFF; // CRC LSB					 

	send_request(signal);

	read_response(reg_buf, sizeof(reg_buf));

}

// writes the 'state' to the coil with address 'addr'
void modbus_write_coil(uint8_t deviceID, uint16_t coil_addr, bool state) {

	uint8_t FC = 0x05;
	uint8_t addr_low = coil_addr & 0xFF;
	uint8_t addr_high = (coil_addr >> 8) & 0xFF;
	uint8_t _state;

	if (state) {
		_state = 0xFF;
	} else {
		_state = 0x00;
	}

	uint8_t buff[6] = {deviceID, FC, addr_high, addr_low, _state, 0x00};

	uint16_t crc = modbusRTU_CRC16(buff, sizeof(buff));
	
	uint8_t signal[8] = {deviceID, FC, addr_high, addr_low,
				 		 _state, 0x00, 0x00, 0x00};

	signal[6] = crc & 0xFF; // CRC MSB
	signal[7] = (crc >> 8) & 0xFF; // CRC LSB

	send_request(signal);
}

// calculation of the lux data from the GY-485 light intensity sensor
// double GY_485(uint8_t regs[]) {
// 	double lux = (reg[3]<<24)‖(reg[4]<<16)‖(reg[5]<<8)‖ reg[6];
// 	return lux;
// }

// return double get_sensor_data(uint8_t regs[)


int main(void)
{
	if (!device_is_ready(uart_dev)) { // Check if the UART device is ready
		printk("UART device not found!");
		return 0;
	}

	uart_init(); // initialise UART with specified settings

	/* Define the on and off signals to be sent via UART

		signal = {
		 1:deviceId/slaveID,
		 2:function ("Write Single Coil"-->0x05),
		 3,4: coil address/channel number,
		 5,6: coil state (0xFF00 --> ON and 0x0000 --> OFF),
		 7,8: Cyclic Redundancy Check for error detection
		}
	*/

	// uint8_t on_signal1[8] = {0x01, 0x05, 0x00, 0x00,
	// 			 0xFF, 0x00, 0x8C, 0x3A}; // hex code to turn on channel 1

	// uint8_t off_signal1[8] = {0x01, 0x05, 0x00, 0x00,
	// 			  0x00, 0x00, 0xCD, 0xCA}; // hex code to turn off channel 1

	// uint8_t on_signal2[8] = {0x01, 0x05, 0x00, 0x01,
	// 			 0xFF, 0x00, 0xDD, 0xFA}; // hex code to turn on channel 2

	// uint8_t off_signal2[8] = {0x01, 0x05, 0x00, 0x01,
	// 			  0x00, 0x00, 0x9C, 0x0A}; // hex code to turn off channel 2

	uint8_t deviceID = 0x5A;
	uint16_t coil_addr = 0x00;
	uint16_t holding_regs_gy485[9] = {0};
	/*
	  (1) Byte0: 0x5A frame header flag
	  (2) Byte1: 0x5A frame header flag
      (3) Byte2: 0x09 device ID
      (4) Byte3: 0x04 data volume
      (5) Byte4: 0x00~0xFF high 8 bits before data
      (6) Byte5: 0x00~0xFF lower 8 bits before data
      (7) Byte6: 0x00~0xFF data after 8 bits high
      (8) Byte7: 0x00~0xFF data lower 8 bits
      (9) Byte8: 0x00~0xFF checksum (the previous data is added and accumulated, 
	  								 only 8 bits are left)*/
	uint8_t coil_tbl[2] = {0};

	// Sensor Outputs
	// double lux = 0.0;

	while (1) {

		// send_request(on_signal1);
		// modbus_write_coil(deviceID, coil_addr, true);

		// k_msleep(1000);

		// // send_request(off_signal1);
		// modbus_write_coil(deviceID, coil_addr, false);

		// k_msleep(1000);

		// read_response(holding_regs, 8);

		modbus_read_holding_regs(deviceID, 0, holding_regs_gy485, sizeof(holding_regs_gy485));
		k_msleep(2000);

		for(int i = 0; i < 8; i++) {
			printk ("0x%04x", holding_regs[i]);
		}

		

		// for(int i = 0; i < 8; i++) {
		// 	LOG_INF("0x%02x", holding_regs[i]);
		// }
		// k_msleep(1000);

	}
	return 0;
}
