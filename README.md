# UART Control Project with Zephyr

This project demonstrates how to use the Zephyr RTOS to control a 485 relay using a 485 to TTL converter. The project uses hex codes to send commands via UART to control two channels.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Files](#files)
- [Configuration](#configuration)
- [License](#license)

## Overview

The project initializes UART with specific settings and sends signals to control two channels. The signals are sent in a loop to turn the channels on and off with predefined hex commands. This is useful for applications requiring repetitive control over UART.

## Features

- Configures UART with a baud rate of 9600, no parity, 1 stop bit, and 8 data bits.
- Sends specific hex codes to turn on and off channels connected via UART.
- Includes small delays to ensure signal integrity.

## Requirements

- Zephyr RTOS
- UART device compatible with Zephyr
- Zephyr development environment setup

## Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/yourusername/zephyr-uart-control.git
    ```
2. **Navigate to the project directory**:
    ```bash
    cd zephyr-uart-control
    ```
3. **Set up your Zephyr environment** following the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

4. **Build the project**:
    ```bash
    west build -b <board-name> -s . -d build
    ```
   Replace `<board-name>` with your target board, such as `nrf52840dk_nrf52840`.

5. **Flash the firmware**:
    ```bash
    west flash
    ```

## Usage

1. **Connect** the device to a UART-compatible device or monitor.
2. **Power on** the board.
3. The project will automatically start sending hex signals via UART to control the connected channels.
4. Monitor the UART output to see the signals being sent.

### Signal Structure

The signals are defined as:

- **On Signal Channel 1**: `{0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A}`
- **Off Signal Channel 1**: `{0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA}`
- **On Signal Channel 2**: `{0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA}`
- **Off Signal Channel 2**: `{0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A}`

These signals represent the control commands to turn the channels on and off.

## Files

- `src/main.c`: Contains the main code for UART initialization and signal transmission.
- `prj.conf`: Zephyr project configuration file.
- `CMakeLists.txt`: CMake configuration file for building the project.

## Configuration

### UART Configuration

The UART is configured with the following parameters in `main.c`:
```c
struct uart_config uart_cfg;
uart_cfg.baudrate = 9600;
uart_cfg.parity = UART_CFG_PARITY_NONE;
uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
