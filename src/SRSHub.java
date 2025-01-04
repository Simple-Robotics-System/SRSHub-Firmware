package org.firstinspires.ftc.teamcode.devices;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReferenceArray;

@I2cDeviceType
@DeviceProperties(xmlTag = "SRSHub", name = "SRSHub")
public class SRSHub extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    private static final int I2C_ADDRESS = 0x57;

    private static final int DEVICE_ID = 0x61;
    private static final int DEVICE_MAJOR_VERSION = 0x01;
    private static final int DEVICE_MINOR_VERSION = 0x00;
    private static final int DEVICE_PATCH_VERSION = 0x00;

    private static final int BITS_PER_ANALOG_DIGITAL_DEVICE = 2;
    private static final int BITS_PER_ENCODER = 2;
    private static final int BITS_PER_I2C_DEVICE = 2;

    private static final ByteOrder BYTE_ORDER = ByteOrder.LITTLE_ENDIAN;

    private volatile Config config;

    private boolean threadUpdates;
    private volatile String updateThread;

    private int updateLength = 1;

    private final AtomicReferenceArray<Double> analogDigitalValues =
        new AtomicReferenceArray<>(12);
    private final AtomicReferenceArray<PoseVel> encoderValues =
        new AtomicReferenceArray<>(6);

    private final HashMap<Class<? extends I2CDevice>,
        ConcurrentHashMap<String, Double>>[] i2cBusValues =
        new HashMap[3];

    public enum AnalogDigitalDevice {
        ANALOG(0),
        DIGITAL(1),
        NONE(2);

        final int value;

        AnalogDigitalDevice(int value) {
            this.value = value;
        }
    }

    public enum Encoder {
        QUADRATURE(0),
        PWM(1),
        NONE(2);

        final int value;

        Encoder(int value) {
            this.value = value;
        }
    }

    private interface I2CDevice {
        int getValue();

        int getUpdateLength();

        int getAddress();

        BitSet getConfig();
    }

    public static class APDS9151 implements I2CDevice {
        private final BitSet config = new BitSet(0);

        public int getValue() {
            return 0;
        }

        public int getUpdateLength() {
            return 56;
        }

        public int getAddress() {
            return 0x52;
        }

        public BitSet getConfig() {
            return config;
        }
    }

    public static class AS7341 implements I2CDevice {
        private final BitSet config = new BitSet(0);

        public int getValue() {
            return 1;
        }

        public int getUpdateLength() {
            return 56;
        }

        public int getAddress() {
            return 0x39;
        }

        public BitSet getConfig() {
            return config;
        }
    }

    public static class VL53L5CX implements I2CDevice {
        private final BitSet config = new BitSet(0);

        public int getValue() {
            return 2;
        }

        public int getUpdateLength() {
            return 32;
        }

        public int getAddress() {
            return 0x29;
        }

        public BitSet getConfig() {
            return config;
        }
    }

    public static class VL53L0X implements I2CDevice {
        private final BitSet config = new BitSet(0);

        public int getValue() {
            return 3;
        }

        public int getUpdateLength() {
            return 32;
        }

        public int getAddress() {
            return 0x29;
        }

        public BitSet getConfig() {
            return config;
        }
    }

    public static class Config {
        boolean locked = false;

        private final AnalogDigitalDevice[] analogDigitalDevices =
            new AnalogDigitalDevice[12];

        private final Encoder[] encoders = new Encoder[6];

        private final ArrayList<I2CDevice>[] i2cBusses = new ArrayList[3];

        public Config() {
            Arrays.fill(
                analogDigitalDevices,
                AnalogDigitalDevice.NONE
            );

            Arrays.fill(
                encoders,
                Encoder.NONE
            );

            Arrays.fill(
                i2cBusses,
                new ArrayList<I2CDevice>()
            );
        }

        /**
         * configures an analog-digital pin to be analog, digital, or none
         *
         * @param pin the pin being configured, from 1 to 12
         * @param device the type of device on the pin
         *
         * @throws IndexOutOfBoundsException if the pin is not between 1 and
         *     12, inclusive
         * @throws IllegalStateException if init has already been called
         */
        public void setAnalogDigitalDevice(
            int pin,
            AnalogDigitalDevice device
        ) {
            if (pin < 1 || pin > 12) {
                RobotLog.addGlobalWarningMessage("AnalogDigitalDevice pin " +
                    "must be from 1 to 12");

                throw new IndexOutOfBoundsException();
            }

            if (locked) {
                RobotLog.addGlobalWarningMessage("Config has already been " +
                    "passed to the SRSHub; changes cannot be made");

                throw new IllegalStateException();
            }

            analogDigitalDevices[pin - 1] = device;
        }

        /**
         * configures an encoder port to be quadrature, pwm, or none
         *
         * @param port the port being configured, from 1 to 6
         * @param device the type of device on the port
         *
         * @throws IndexOutOfBoundsException if the port is not between 1
         *     and 6, inclusive
         * @throws IllegalStateException if init has already been called
         */
        public void setEncoder(int port, Encoder device) {
            if (port < 1 || port > 6) {
                RobotLog.addGlobalWarningMessage("Encoder port must " +
                    "be from 1 to 6");

                throw new IndexOutOfBoundsException();
            }

            if (locked) {
                RobotLog.addGlobalWarningMessage("Config has already been " +
                    "passed to the SRSHub; changes cannot be made");

                throw new IllegalStateException();
            }

            encoders[port - 1] = device;
        }

        /**
         * adds a device to an I2C bus
         *
         * @param bus the bus to which the device is being added, from 1 to
         *     3
         * @param device the (unique) type of the device on the bus
         *
         * @throws IndexOutOfBoundsException if the bus is not between 1 and
         *     3, inclusive
         * @throws IllegalStateException if init has already been called
         */
        public void addI2CDevice(int bus, I2CDevice device) {
            if (bus < 1 || bus > 3) {
                RobotLog.addGlobalWarningMessage("I2C bus must be from 1 to" +
                    " 3");

                throw new IndexOutOfBoundsException();
            }

            if (locked) {
                RobotLog.addGlobalWarningMessage("Config has already been " +
                    "passed to the SRSHub; changes cannot be made");

                throw new IllegalStateException();
            }

            for (I2CDevice i2cDevice : i2cBusses[bus - 1]) {
                if (i2cDevice.getClass() == device.getClass()) {
                    RobotLog.addGlobalWarningMessage("I2C Bus #" + bus + " " +
                        "already has a device of type " + device
                        .getClass()
                        .getName());

                    throw new IllegalStateException();
                }

                if (i2cDevice.getAddress() == device.getAddress()) {
                    RobotLog.addGlobalWarningMessage("I2C Bus #" + bus + " " +
                        "already has a bus of type " + i2cDevice
                        .getClass()
                        .getName() + " which has an I2C address conflicting " +
                        "with the " + device
                        .getClass()
                        .getName());

                    throw new IllegalStateException();
                }
            }

            i2cBusses[bus - 1].add(device);
        }

        protected void lock() {
            locked = true;
        }
    }

    public static class PoseVel {
        public final double pose;
        public final double vel;

        public PoseVel(double pose, double vel) {
            this.pose = pose;
            this.vel = vel;
        }
    }

    public SRSHub(
        I2cDeviceSynchSimple deviceClient,
        boolean deviceClientIsOwned
    ) {
        super(
            deviceClient,
            deviceClientIsOwned
        );

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    protected boolean doInitialize() {
        ((LynxI2cDeviceSynch) (deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        isInitialized = false;

        verifyInitialization();

        return true;
    }

    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    public String getDeviceName() {
        return "SRSHub";
    }

    enum Register {
        DEVICE_INFO(
            0x00,
            4
        ),

        INIT(
            0x01,
            -1
        ),

        READ(
            0x02,
            -1
        );

        public final byte address;
        public final int length;

        Register(int address, int length) {
            this.address = (byte) address;
            this.length = length;
        }
    }

    private void verifyInitialization() {
        if (!isInitialized) {
            byte[] deviceInfo = deviceClient.read(
                Register.DEVICE_INFO.address,
                Register.DEVICE_INFO.length
            );

            int deviceId = deviceInfo[0];

            if (deviceId != DEVICE_ID) {
                RobotLog.addGlobalWarningMessage(
                    "SRSHub does not report correct chip id; received 0x%X, " +
                        "expected 0x%X",
                    deviceId,
                    DEVICE_ID
                );

                throw new RuntimeException();
            }

            int deviceMajorVersion = deviceInfo[1];
            int deviceMinorVersion = deviceInfo[2];
            int devicePatchVersion = deviceInfo[3];

            if (deviceMajorVersion != DEVICE_MAJOR_VERSION || deviceMinorVersion != DEVICE_MINOR_VERSION || devicePatchVersion != DEVICE_PATCH_VERSION) {
                RobotLog.addGlobalWarningMessage(
                    "SRSHub does not report correct firmware version; " +
                        "received v" + deviceMajorVersion + "." + deviceMinorVersion + "." + devicePatchVersion + ", expected v" + DEVICE_MAJOR_VERSION + "." + DEVICE_MINOR_VERSION + "." + DEVICE_PATCH_VERSION
                );

                throw new RuntimeException();
            }

            isInitialized = true;
        }
    }

    public long getUnsignedInt(byte[] bytes) {
        return ((bytes[0] & 0xFF)) |
            ((bytes[1] & 0xFF) << 8) |
            ((bytes[2] & 0xFF) << 16) |
            ((long) (bytes[3] & 0xFF) << 24);
    }

    /**
     * passes the configuration to the SRSHub and can only be called once
     *
     * @param config the configuration details that will be passed to the
     *     SRSHub
     *
     * @throws IllegalStateException if the SRSHub has already been
     *     initialized
     */
    public void init(
        @NonNull Config config
    ) {
        init(
            config,
            false
        );
    }

    /**
     * passes the configuration to the SRSHub; can only be called once and must
     * be called on the main thread
     *
     * @param config the configuration details that will be passed to the
     *     SRSHub
     * @param threadUpdates whether update calls should be threaded
     *
     * @throws IllegalStateException if the SRSHub has already been
     *     initialized
     */
    public void init(
        @NonNull Config config,
        boolean threadUpdates
    ) {
        verifyInitialization();

        if (this.config != null) {
            RobotLog.addGlobalWarningMessage("The SRSHub may only be " +
                "initialized once");

            throw new IllegalStateException();
        }

        config.lock();
        this.config = config;

        for (int i = 0; i < analogDigitalValues.length(); i++) {
            analogDigitalValues.set(
                i,
                (double) 0
            );
        }

        for (int i = 0; i < encoderValues.length(); i++) {
            encoderValues.set(
                i,
                new PoseVel(
                    0,
                    0
                )
            );
        }

        int initLength =
            config.analogDigitalDevices.length * BITS_PER_ANALOG_DIGITAL_DEVICE + config.encoders.length * BITS_PER_ENCODER;

        for (int i = 0; i < config.i2cBusses.length; i++) {
            for (int j = 0; j < config.i2cBusses[i].size(); j++) {
                initLength += BITS_PER_I2C_DEVICE + config.i2cBusses[i]
                    .get(j)
                    .getConfig().size();
            }
        }

        BitSet init = new BitSet(initLength);

        int index = 0;

        for (int i = 0; i < config.analogDigitalDevices.length; i++) {
            switch (config.analogDigitalDevices[i]) {
                case ANALOG:
                    updateLength += 32;

                    break;
                case DIGITAL:
                    updateLength += 1;

                    break;
                case NONE:
                    break;
            }

            for (int j = 0; j < BITS_PER_ANALOG_DIGITAL_DEVICE; j++) {
                if ((config.analogDigitalDevices[i].value >> j & 1) == 1) {
                    init.set(index++);
                }
            }
        }

        for (int i = 0; i < config.encoders.length; i++) {
            if (config.encoders[i] != Encoder.NONE) {
                updateLength += 64;
            }

            for (int j = 0; j < BITS_PER_ENCODER; j++) {
                if ((config.encoders[i].value >> j & 1) == 1) {
                    init.set(index++);
                }
            }
        }

        for (int i = 0; i < config.i2cBusses.length; i++) {
            for (int j = 0; j < config.i2cBusses[i].size(); j++) {
                updateLength += config.i2cBusses[i]
                    .get(j).getUpdateLength();

                I2CDevice device = config.i2cBusses[i].get(j);

                i2cBusValues[i].put(
                    device.getClass(),
                    new ConcurrentHashMap<>()
                );

                if (device.getClass() == APDS9151.class || device.getClass() == AS7341.class) {
                    Objects
                        .requireNonNull(i2cBusValues[i].get(device.getClass()))
                        .put(
                            "r",
                            (double) 0
                        );

                    Objects
                        .requireNonNull(i2cBusValues[i].get(device.getClass()))
                        .put(
                            "g",
                            (double) 0
                        );

                    Objects
                        .requireNonNull(i2cBusValues[i].get(device.getClass()))
                        .put(
                            "b",
                            (double) 0
                        );

                    Objects
                        .requireNonNull(i2cBusValues[i].get(device.getClass()))
                        .put(
                            "proximity",
                            (double) 0
                        );
                }

                if (device.getClass() == VL53L5CX.class || device.getClass() == VL53L0X.class) {
                    Objects
                        .requireNonNull(i2cBusValues[i].get(device.getClass()))
                        .put(
                            "distance",
                            (double) 0
                        );
                }

                for (int k = 0; k < BITS_PER_I2C_DEVICE; k++) {
                    if ((device.getValue() >> k & 1) == 1) {
                        init.set(index++);
                    }
                }

                for (int k = 0; k < device.getConfig().size(); k++) {
                    if (device.getConfig().get(k)) {
                        init.set(index++);
                    }
                }
            }
        }

        this.threadUpdates = threadUpdates;

        updateLength = (updateLength + 7) / 8;

        deviceClient.write(
            Register.INIT.address,
            ByteBuffer.wrap(init.toByteArray()).order(BYTE_ORDER).array()
        );

        if (threadUpdates) {
            new Thread(() -> {
                if (updateThread == null) {
                    updateThread = Thread.currentThread().getName();
                }

                update();
            }).start();
        }
    }

    /**
     * bulk-reads data from the SRSHub as specified in the configuration
     *
     * @throws IllegalStateException if the SRSHub has not yet been
     *     initialized
     * @throws IllegalStateException if threadUpdates is set to true and the
     *     method is not called on the dedicated thread
     * @throws RuntimeException if the SRSHub is unable to update according
     *     to the provided configuration
     */
    public synchronized void update() {
        if (!isInitialized || config == null) {
            RobotLog.addGlobalWarningMessage("The SRSHub must be initialized " +
                "before updating");

            throw new IllegalStateException();
        }

        if (threadUpdates && !Thread
            .currentThread()
            .getName()
            .equals(updateThread)) {
            RobotLog.addGlobalWarningMessage("Update cannot be called when " +
                "threadUpdates is set to true");

            throw new IllegalStateException();
        }

        BitSet data = BitSet.valueOf(ByteBuffer.wrap(deviceClient.read(
            Register.READ.length,
            updateLength
        )).order(BYTE_ORDER).array());

        if (data.get(0)) {
            RobotLog.addGlobalWarningMessage("SRSHub failed to update; ensure" +
                " your configuration is accurate");

            throw new RuntimeException();
        }

        int index = 1;

        for (int i = 0; i < config.analogDigitalDevices.length; i++) {
            switch (config.analogDigitalDevices[i]) {
                case ANALOG:
                    analogDigitalValues.set(
                        i,
                        getUnsignedInt(ByteBuffer
                            .wrap(data
                                .get(
                                    index,
                                    index + 32
                                )
                                .toByteArray()).array()) / (double) 1000
                    );

                    index += 32;

                    break;
                case DIGITAL:
                    analogDigitalValues.set(
                        i,
                        (double) (data.get(index++) ? 1 : 0)
                    );

                    break;
                case NONE:
                    break;
            }
        }

        for (int i = 0; i < config.encoders.length; i++) {
            switch (config.encoders[i]) {
                case QUADRATURE:
                    encoderValues.set(
                        i,
                        new PoseVel(
                            encoderValues.get(i).pose + ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 32
                                    )
                                    .toByteArray())
                                .getInt(),
                            ByteBuffer
                                .wrap(data
                                    .get(
                                        index + 32,
                                        index + 64
                                    )
                                    .toByteArray())
                                .getInt() / (double) 1000
                        )
                    );

                    index += 64;

                    break;
                case PWM:
                    encoderValues.set(
                        i,
                        new PoseVel(
                            encoderValues.get(i).pose + ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 32
                                    )
                                    .toByteArray())
                                .getInt() / (double) 1000,
                            ByteBuffer
                                .wrap(data
                                    .get(
                                        index + 32,
                                        index + 64
                                    )
                                    .toByteArray())
                                .getInt() / (double) 1000
                        )
                    );

                    index += 64;

                    break;
                case NONE:
                    break;
            }

            if (config.encoders[i] != Encoder.NONE) {
                encoderValues.set(
                    i,
                    new PoseVel(
                        encoderValues.get(i).pose + ByteBuffer
                            .wrap(data
                                .get(
                                    index,
                                    index + 32
                                )
                                .toByteArray())
                            .getInt(),
                        ByteBuffer
                            .wrap(data
                                .get(
                                    index + 32,
                                    index + 64
                                )
                                .toByteArray())
                            .getInt() / (double) 1000
                    )
                );

                index += 64;
            }
        }

        for (int i = 0; i < config.i2cBusses.length; i++) {
            for (int j = 0; j < config.i2cBusses[i].size(); j++) {
                Class<? extends I2CDevice> device = config.i2cBusses[i]
                    .get(j)
                    .getClass();

                if (device.equals(APDS9151.class) || device.equals(AS7341.class)) {
                    Objects
                        .requireNonNull(i2cBusValues[i].get(device))
                        .put(
                            "r",
                            (double) ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 8
                                    )
                                    .toByteArray())
                                .getInt()
                        );

                    index += 8;

                    Objects
                        .requireNonNull(i2cBusValues[i].get(device))
                        .put(
                            "g",
                            (double) ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 8
                                    )
                                    .toByteArray())
                                .getInt()
                        );

                    index += 8;

                    Objects
                        .requireNonNull(i2cBusValues[i].get(device))
                        .put(
                            "b",
                            (double) ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 8
                                    )
                                    .toByteArray())
                                .getInt()
                        );

                    index += 8;

                    Objects.requireNonNull(i2cBusValues[i].get(device))
                        .put(
                            "proximity",
                            ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 32
                                    )
                                    .toByteArray())
                                .getInt() / (double) 1000
                        );

                    index += 32;
                }

                if (device.equals(VL53L5CX.class) || device.equals(VL53L0X.class)) {
                    Objects.requireNonNull(i2cBusValues[i]
                            .get(device))
                        .put(
                            "distance",
                            ByteBuffer
                                .wrap(data
                                    .get(
                                        index,
                                        index + 32
                                    )
                                    .toByteArray())
                                .getInt() / (double) 1000
                        );

                    index += 32;
                }
            }
        }
    }

    /**
     * gets the current value of the AnalogDigitalDevice at the specified pin
     *
     * @param pin the pin being read, from 1 to 12
     *
     * @return the current value read from the AnalogDigitalDevice; from 0 to 1
     *     for analog devices and 0 or 1 for digital devices
     *
     * @throws IndexOutOfBoundsException if the pin is not between 1 and 12,
     *     inclusive
     * @throws IllegalStateException if the SRSHub has not yet been
     *     initialized
     * @throws IllegalStateException if the pin was not configured
     */
    public double readAnalogDigitalDevice(int pin) {
        if (pin < 1 || pin > 12) {
            RobotLog.addGlobalWarningMessage("AnalogDigitalDevice pin " +
                "must be from 1 to 12");

            throw new IndexOutOfBoundsException();
        }

        if (!isInitialized || config == null) {
            RobotLog.addGlobalWarningMessage("The SRSHub must be initialized " +
                "before reading");

            throw new IllegalStateException();
        }

        if (config.analogDigitalDevices[pin - 1] == AnalogDigitalDevice.NONE) {
            RobotLog.addGlobalWarningMessage("AnalogDigitalDevice pin #" + pin +
                " was not configured");

            throw new IllegalStateException();
        }

        return analogDigitalValues.get(pin - 1);
    }

    /**
     * gets the current position and velocity of the encoder at the specified
     * port
     *
     * @param port the port being read, from 1 to 6
     *
     * @return the current position and velocity of the encoder; for quadrature
     *     encoders these are in ticks since last reset and ticks per second,
     *     respectively; for PWM encoders these are in rotations since last
     *     reset and rotations per second, respectively
     *
     * @throws IndexOutOfBoundsException if the port is not between 1 and 6,
     *     inclusive
     * @throws IllegalStateException if the SRSHub has not yet been
     *     initialized
     * @throws IllegalStateException if the port was not configured
     */
    public PoseVel readEncoder(int port) {
        if (port < 1 || port > 6) {
            RobotLog.addGlobalWarningMessage("Encoder port " +
                "must be from 1 to 6");

            throw new IndexOutOfBoundsException();
        }

        if (!isInitialized || config == null) {
            RobotLog.addGlobalWarningMessage("The SRSHub must be initialized " +
                "before reading");

            throw new IllegalStateException();
        }

        if (config.encoders[port - 1] == Encoder.NONE) {
            RobotLog.addGlobalWarningMessage("Encoder port #" + port +
                " was not configured");

            throw new IllegalStateException();
        }

        return encoderValues.get(port - 1);
    }

    /**
     * resets the position of the encoder at the specified port
     *
     * @param port the port being reset, from 1 to 6
     *
     * @throws IndexOutOfBoundsException if the port is not between 1 and 6,
     *     inclusive
     * @throws IllegalStateException if the SRSHub has not yet been
     *     initialized
     * @throws IllegalStateException if the port was not configured
     */
    public synchronized void resetEncoder(int port) {
        if (port < 1 || port > 6) {
            RobotLog.addGlobalWarningMessage("Encoder port " +
                "must be from 1 to 6");

            throw new IndexOutOfBoundsException();
        }

        if (!isInitialized || config == null) {
            RobotLog.addGlobalWarningMessage("The SRSHub must be initialized " +
                "before reading");

            throw new IllegalStateException();
        }

        if (config.encoders[port - 1] == Encoder.NONE) {
            RobotLog.addGlobalWarningMessage("Encoder port #" + port +
                " was not configured");

            throw new IllegalStateException();
        }

        encoderValues.set(
            port - 1,
            new PoseVel(
                0,
                encoderValues.get(port - 1).vel
            )
        );
    }

    /**
     * gets the current value(s) read from the specified I2C device at the
     * specified bus
     *
     * @param bus the bus from which the device is being read, from 1 to 3
     * @param deviceClass the type of device being read
     *
     * @return the current value(s) returned by the I2C device
     *
     * @throws IndexOutOfBoundsException if the bus is not between 1 and 3,
     *     inclusive
     * @throws IllegalArgumentException if the device is not a valid I2C
     *     device class
     * @throws IllegalStateException if the SRSHub has not yet been
     *     initialized
     * @throws IllegalStateException if the device was not configured on the
     *     bus encoder
     */
    public Map<String, Double> readI2CDevice(
        int bus,
        Class<? extends I2CDevice> deviceClass
    ) {
        if (bus < 1 || bus > 3) {
            RobotLog.addGlobalWarningMessage("I2C bus must be from 1 to" +
                " 3");

            throw new IndexOutOfBoundsException();
        }

        if (!I2CDevice.class.isAssignableFrom(deviceClass)) {
            RobotLog.addGlobalWarningMessage("I2C device must be a valid I2C " +
                "device class");

            throw new IllegalArgumentException();
        }

        if (!isInitialized || config == null) {
            RobotLog.addGlobalWarningMessage("The SRSHub must be initialized " +
                "before reading");

            throw new IllegalStateException();
        }

        if (!i2cBusValues[bus - 1].containsKey(deviceClass)) {
            RobotLog.addGlobalWarningMessage("I2C device " + deviceClass.getName() +
                " was not configured on bus #" + bus);

            throw new IllegalStateException();
        }

        return i2cBusValues[bus - 1].get(deviceClass);
    }
}
