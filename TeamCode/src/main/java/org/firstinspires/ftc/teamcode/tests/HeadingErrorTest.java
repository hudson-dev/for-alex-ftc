package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="testy boi")
public class HeadingErrorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        double lastAngle = drive.imu.getAngularOrientation().firstAngle;
        double currentCumAngle = 0;

        while(opModeIsActive()) {
            drive.update();
            double turn = gamepad1.right_stick_x * 0.35;
            drive.setMotorPowers(turn, turn, -turn, -turn);

            double currentAngle = drive.imu.getAngularOrientation().firstAngle;
            double deltaAngle = currentAngle - lastAngle;
            lastAngle = currentAngle;

            while (deltaAngle >= Math.PI) {
                deltaAngle -= 2*Math.PI;
            }

            while (deltaAngle <= -Math.PI) {
                deltaAngle += 2*Math.PI;
            }

            currentCumAngle += deltaAngle;

            telemetry.addData("Heading", Math.toDegrees(drive.localizer.heading));
            telemetry.addData("Cum Angle", Math.toDegrees(currentCumAngle));
            telemetry.update();
        }
    }
}
