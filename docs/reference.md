---
title: Driver Reference
description: Configuring and interfacing with your SRSHub through Java
---

## Configuration Functions

The below functions belong to the `Config` class, which is later passed into the SRSHub instance.

### setAnalogDigitalDevice

```java
public void setAnalogDigitalDevice(int pin, AnalogDigitalDevice device)
```

Configures an analog-digital pin to be analog, digital, or none

**Parameters:**
- `pin`: The pin being configured, from 1 to 12
- `device`: The type of device on the pin (ANALOG, DIGITAL, or NONE)

**Throws:**
- `IndexOutOfBoundsException`: If the pin is not between 1 and 12, inclusive
- `IllegalStateException`: If init has already been called

### setEncoder

```java
public void setEncoder(int port, Encoder device)
```

Configures an encoder port to be quadrature, pwm, or none

**Parameters:**
- `port`: The port being configured, from 1 to 6
- `device`: The type of device on the port (QUADRATURE, PWM, or NONE)

**Throws:**
- `IndexOutOfBoundsException`: If the port is not between 1 and 6, inclusive
- `IllegalStateException`: If init has already been called

### addI2CDevice

```java
public void addI2CDevice(int bus, I2CDevice device)
```

Adds a device to an I2C bus

**Parameters:**
- `bus`: The bus to which the device is being added, from 1 to 3
- `device`: The (unique) type of device on the bus

**Throws:**
- `IndexOutOfBoundsException`: If the bus is not between 1 and 3, inclusive
- `IllegalStateException`: If init has already been called or if a device of the same type already exists on the bus

## Primary Functions

The below functions are part of the SRSHub class itself.

### init

```java
public void init(Config config)
```

Passes the configuration to the SRSHub and can only be called once

**Parameters:**
- `config`: The configuration details that will be passed to the SRSHub

**Throws:**
- `IllegalStateException`: If the SRSHub has already been initialized

### init (with update threading)

```java
public void init(Config config, boolean threadUpdates)
```

Passes the configuration to the SRSHub; can only be called once and must be called on the main thread

**Parameters:**
- `config`: The configuration details that will be passed to the SRSHub
- `threadUpdates`: Whether update calls should be threaded

**Throws:**
- `IllegalStateException`: If the SRSHub has already been initialized

### update

```java
public synchronized void update()
```

Bulk-reads data from the SRSHub as specified in the configuration

**Throws:**
- `IllegalStateException`: If the SRSHub has not yet been initialized
- `IllegalStateException`: If threadUpdates is set to true and the method is not called on the dedicated thread
- `RuntimeException`: If the SRSHub is unable to update according to the provided configuration

### readAnalogDigitalDevice

```java
public double readAnalogDigitalDevice(int pin)
```

Gets the current value of the AnalogDigitalDevice at the specified pin

**Parameters:**
- `pin`: The pin being read, from 1 to 12

**Returns:**
- The current value read from the AnalogDigitalDevice; from 0 to 1 for analog devices and 0 or 1 for digital devices

**Throws:**
- `IndexOutOfBoundsException`: If the pin is not between 1 and 12, inclusive
- `IllegalStateException`: If the SRSHub has not yet been initialized or if the pin was not configured

### readEncoder

```java
public PoseVel readEncoder(int port)
```

Gets the current position and velocity of the encoder at the specified port

**Parameters:**
- `port`: The port being read, from 1 to 6

**Returns:**
- The current position and velocity of the encoder:
- For quadrature encoders: ticks since last reset and ticks per second
- For PWM encoders: rotations since last reset and rotations per second

**Throws:**
- `IndexOutOfBoundsException`: If the port is not between 1 and 6, inclusive
- `IllegalStateException`: If the SRSHub has not yet been initialized or if the port was not configured

### resetEncoder

```java
public synchronized void resetEncoder(int port)
```

Resets the position of the encoder at the specified port

**Parameters:**
- `port`: The port being reset, from 1 to 6

**Throws:**
- `IndexOutOfBoundsException`: If the port is not between 1 and 6, inclusive
- `IllegalStateException`: If the SRSHub has not yet been initialized or if the port was not configured

### readI2CDevice

```java
public Map<String, Double> readI2CDevice(int bus, Class<? extends I2CDevice> deviceClass)
```

Gets the current value(s) read from the specified I2C device at the specified bus

**Parameters:**
- `bus`: The bus from which the device is being read, from 1 to 3
- `deviceClass`: The type of device being read

**Returns:**
- A map of current values returned by the I2C device

**Throws:**
- `IndexOutOfBoundsException`: If the bus is not between 1 and 3, inclusive
- `IllegalArgumentException`: If the device is not a valid I2C device class
- `IllegalStateException`: If the SRSHub has not yet been initialized or if the device was not configured on the bus

## Device Types

### AnalogDigitalDevice
- `ANALOG`: Analog input device
- `DIGITAL`: Digital input device
- `NONE`: No device connected

### Encoder
- `QUADRATURE`: Quadrature encoder
- `PWM`: PWM encoder
- `NONE`: No encoder connected

### Supported I2C Devices
- `APDS9151`: Color/proximity sensor (returns r, g, b, proximity)
- `AS7341`: Spectral sensor (returns r, g, b, proximity)
- `VL53L5CX`: Time-of-flight distance sensor (returns distance)
- `VL53L0X`: Time-of-flight distance sensor (returns distance)