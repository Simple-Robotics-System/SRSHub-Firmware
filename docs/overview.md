---
title: Overview
description: The basics of the SRSHub
---

import { Card } from '@astrojs/starlight/components';

<Card title="Note" icon="information">
  This section describes hardware functionality and setup. If you're looking for driver setup or programming information, see [programming](/hub/programming) and [reference](/hub/reference).
</Card>

## Introduction
The SRS Hub is a sensor reading board that aims to solve several issues with currently available FIRST Tech Challenge hardware. It features:
- 12x Analog/Digital Sensor Inputs
- 3x I<sup>2</sup>C Buses
- 6x Quadrature Encoder Inputs/12x PWM Encoder Inputs
- All data is automatically bulk-read over a single I<sup>2</sup>C invocation, minimizing Control Hub I<sup>2</sup>C delays

Some of its advantages over the REV Control Hub are:
- Linear analog sensor reads
- Hardware quadrature reading, ensuring no skipped ticks
- Avoiding expensive Control and Expansion Hub bulk-reads

The SRS Hub is user-updatable (upload instructions coming soon), and firmware updates including new I<sup>2</sup>C device support will be released regularly.

## Wiring
<img src="/hub/pins.png" style="border-radius: 20px;" alt="SRSHub pins"></img>
All ports on the SRS Hub are JST PH and should work out-of-the-box with standard FTC sensors.

The SRS Hub connects to the control hub over I<sup>2</sup>C; use a JST PH cable from the port labeled `I2C` (shown above at the bottom of the board) on the SRS Hub to an I<sup>2</sup>C port on the control hub.

#### Analog/Digital Inputs
The ports labeled `AD1/2-11/12` are for Analog or Digital input (each pin can have its mode configured independently). **These pins are 3.3v tolerant, and will be damaged if supplied with anything more** (such as 5v). Each port supports two inputs and is labeled with their port numbers. For example, `AD1/2` has `AD1` on the left pin and `AD2` on the right pin.

Some FTC usage examples:
- <a href='https://www.melonbotics.com/products/encoder' target='_blank'>Melonbotics Encoder</a> (Analog)
- <a href='https://www.revrobotics.com/rev-31-1462/' target='_blank'>REV Magnetic Limit Switch</a> (Digital)
- <a href='https://www.revrobotics.com/rev-31-1425/' target='_blank'>REV Touch Sensor</a> (Digital)

#### Encoder Inputs
The ports labeled `E1-6` are the quadrature/PWM encoder inputs. These ports have an A and B pin, for the two signal pins of a quadrature encoder. These ports also support absolute-position PWM encoders, on the quadrature A pin (each encoder port supports one PWM encoder). Again, these pins are 3.3v tolerant only (and will be damaged if supplied with anything more).

#### I<sup>2</sup>C Buses
The SRS Hub has three I<sup>2</sup>C buses, meaning that the SRSHub can read three of each sensor since two sensors with the same I<sup>2</sup>C address are not compatible on a single bus. The buses are labeled `I2C1-3`, each with two ports with corresponding labels. You can add more sensors on each bus by soldering or using a cable adapter. As with the other ports, these ports are 3.3v tolerant only (and will be damaged if supplied with anything more).

Due to the nature of I<sup>2</sup>C, the SRSHub firmware `v1.0.0` only supports the following ICs (more extensive support will be added in the future according to community interest):

- `APDS9151` (<a href='https://www.revrobotics.com/rev-31-1557/' target='_blank'>REV Color Sensor V3</a>)
- `AS7341` (<a href='https://www.adafruit.com/product/4698' target='_blank'>Adafruit</a>)
- `VL53L5CX` (<a href='https://www.sparkfun.com/products/18642' target='_blank'>SparkFun</a>, <a href='https://www.pololu.com/product/3417' target='_blank'>Pololu</a>)
- `VL53L0X` (<a href='https://www.revrobotics.com/rev-31-1505/' target='_blank'>REV 2M Distance Sensor</a>, <a href='https://www.adafruit.com/product/3317' target='_blank'>Adafruit</a>)

Also note that the VL53L5CX and VL53L0X share the same I<sup>2</sup>C address, meaning they cannot be placed on the same bus. Similarly, **any two sensors with the same address CANNOT be placed on the same bus**.

As mentioned above, user-uploadable firmware updates including new device support will be released according to demand. If you have a device you wish to be supported, please fill out our [form](https://forms.gle/U7BiX86dGWLAb9Wh7).
