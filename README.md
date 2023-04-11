# YACA - Yet Another CANBus Adapter

ESP32 + SN65HVD230 CANBus adapter firmware.

## Commands

| Command | Description | Example |
| ------- | ----------- | ------- |
| S#      | Set CAN speed | S0 |
| C | Close CAN channel||
| O | Open CAN channel||
| T# | Transmit 29bit CAN message | T10ABBA0031A0230 |
| t# | Transmit 11bit CAN message | t7E031A0230 |
| V | Get firmware version | V |
| N | Get serial number | N |

All commands are terminated with CR `\r` character.