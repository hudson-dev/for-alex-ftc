package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="test")
public class SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            drive.drive(this);

            telemetry.addData("leftIntake", drive.leftIntake.getVoltage());
            telemetry.addData("rightIntake", drive.rightIntake.getVoltage());
            telemetry.addData("depositSensor", drive.depositSensor.getVoltage());
            telemetry.addData("distLeft", drive.distLeft.getVoltage());
            telemetry.addData("distRight", drive.distRight.getVoltage());
            telemetry.addData("magLeft", drive.magLeft.getVoltage());
            telemetry.addData("magRight", drive.magRight.getVoltage());
            telemetry.addData("flex", drive.flex.getVoltage());

            telemetry.update();
        }
    }
}
