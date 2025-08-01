package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;

@TeleOp(name = "HubTest")
public class HubTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(
            telemetry,
            FtcDashboard.getInstance().getTelemetry()
        );

        // All ports default to NONE, buses default to empty
        SRSHub.Config config = new SRSHub.Config();

        config.setEncoder(
            1,
            SRSHub.Encoder.PWM
        );

        config.setEncoder(
            2,
            SRSHub.Encoder.QUADRATURE
        );

        config.addI2CDevice(
            1,
            new SRSHub.GoBildaPinpoint(
                -50,
                -75,
                19.89f,
                SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD
            )
        );

        RobotLog.clearGlobalWarningMsg();

        SRSHub hub = hardwareMap.get(
            SRSHub.class,
            "srshub"
        );

        hub.init(config);

        while (!hub.ready());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            hub.update();

            if (hub.disconnected()) {
                multipleTelemetry.addLine("srshub disconnected");
            } else {
                multipleTelemetry.addData(
                    "encoder 1 position",
                    hub.readEncoder(1).position
                );

                multipleTelemetry.addData(
                    "encoder 2 position",
                    hub.readEncoder(2).position
                );

                multipleTelemetry.addData(
                    "encoder 2 velocity",
                    hub.readEncoder(2).velocity
                );

                SRSHub.GoBildaPinpoint pinpoint = hub.getI2CDevice(
                    1,
                    SRSHub.GoBildaPinpoint.class
                );

                if (!pinpoint.disconnected) {
                    multipleTelemetry.addData(
                        "pose x (mm)",
                        pinpoint.xPosition
                    );

                    multipleTelemetry.addData(
                        "pose y (mm)",
                        pinpoint.yPosition
                    );

                    multipleTelemetry.addData(
                        "pose heading (rad)",
                        pinpoint.hOrientation
                    );
                }
            }

            multipleTelemetry.update();
        }
    }
}