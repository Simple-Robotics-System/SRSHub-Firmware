package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.devices.SRSHub;

@TeleOp(name = "HubExample")
public class HubExample extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // All pins are configured to "None" by default
        SRSHub.Config config = new SRSHub.Config();

        config.setAnalogDigitalDevice(1, SRSHub.AnalogDigitalDevice.ANALOG);
        config.setAnalogDigitalDevice(2, SRSHub.AnalogDigitalDevice.ANALOG);
        config.setAnalogDigitalDevice(3, SRSHub.AnalogDigitalDevice.DIGITAL);

        config.setEncoder(1, SRSHub.Encoder.PWM);
        config.setEncoder(2, SRSHub.Encoder.QUADRATURE);
        config.setEncoder(3, SRSHub.Encoder.QUADRATURE);
        config.setEncoder(4, SRSHub.Encoder.PWM);

        config.addI2CDevice(1, new SRSHub.VL53L5CX());
        config.addI2CDevice(2, new SRSHub.APDS9151());

        SRSHub hub = hardwareMap.get(SRSHub.class, "srshub");

        // Runs I2C reads in a separate thread; threadUpdates defaults to false: hub.init(config) will not thread updates
        hub.init(config, true);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // if you are not theading updates, you must call hub.update() before reading

            telemetry.addData("analog, a/d port 1", hub.readAnalogDigitalDevice(1));
            telemetry.addData("analog, a/d port 2", hub.readAnalogDigitalDevice(2));
            telemetry.addData("digital, a/d port 3", hub.readAnalogDigitalDevice(3));

            telemetry.addData("pwm, encoder port 1", hub.readAnalogDigitalDevice(1));
            telemetry.addData("quadrature, encoder port 2", hub.readAnalogDigitalDevice(2));
            telemetry.addData("quadrature, encoder port 3", hub.readAnalogDigitalDevice(3));
            telemetry.addData("pwm, encoder port 4", hub.readAnalogDigitalDevice(4));

            // I2C reads are returned as a map because each sensor returns different data
            for (String key : hub.readI2CDevice(1, SRSHub.VL53L5CX.class).keySet()) {
                telemetry.addData("vl53l5cx, i2c bus 1, " + key, hub.readI2CDevice(1, SRSHub.VL53L5CX.class).get(key));
            }

            for (String key : hub.readI2CDevice(2, SRSHub.APDS9151.class).keySet()) {
                telemetry.addData("apds9151, i2c bus 2, " + key, hub.readI2CDevice(2, SRSHub.APDS9151.class).get(key));
            }
        }
    }
}
