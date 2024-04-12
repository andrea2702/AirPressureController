# Air Pressure Control System

This project implements a air pressure control system using STM microcontroller, NodeMCU, and an Arduino dashboard. The system regulates the air pressure inside a container simulating a tire. A pressure sensor, an air pump, and a solenoid valve are connected to the STM microcontroller. The NodeMCU communicates with the STM microcontroller via CAN communication. The NodeMCU connects to an Arduino dashboard, allowing the user to select the desired tire pressure for their plants. The selected pressure is sent to the STM microcontroller for pressure control. The NodeMCU updates the dashboard with the current tire pressure obtained from the STM microcontroller.

## Overview

The repository contains three folders:
- **IntarfaceSensorActuator**: Contains the code for the STM microcontroller.
- **ArduinoIoTCan**: Contains the code for the NodeMCU.
- **Dashboard_nov09a**: Contains the code for the Arduino dashboard.

## Demo

A video demonstration of the project is available


[![Demo del Proyecto](http://img.youtube.com/vi/0Lf6t5LeqJk/0.jpg)](https://youtu.be/0Lf6t5LeqJk).
