package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="test")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            drive.drive(this);

            telemetry.addData("leftFront encoder", drive.motors.get(0).getCurrentPosition());
            telemetry.addData("leftBack encoder", drive.motors.get(1).getCurrentPosition());
            telemetry.addData("rightBack encoder", drive.motors.get(2).getCurrentPosition());
            telemetry.addData("rightFront encoder", drive.motors.get(3).getCurrentPosition());

            telemetry.update();
        }
    }
}
