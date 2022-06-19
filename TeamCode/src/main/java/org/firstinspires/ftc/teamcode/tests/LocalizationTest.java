package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@TeleOp(group="testy boi")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(this);

            telemetry.addData("X: ", drive.localizer.x);
            telemetry.addData("Y: ", drive.localizer.y);
            telemetry.addData("Heading: ", drive.localizer.heading);

            telemetry.update();

        }
    }
}
