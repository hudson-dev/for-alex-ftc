package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp (group="tuning")
public class TrackWidthTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.6889764;
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;

        waitForStart();

        double lastAngle = drive.imu.getAngularOrientation().firstAngle;
        double currentCumAngle = 0;

        while(opModeIsActive()) {
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

            double right = drive.rightFront.getCurrentPosition() * ticksToInches * drive.localizer.encoders[0].scaleFactor;
            double left = drive.leftFront.getCurrentPosition() * ticksToInches * drive.localizer.encoders[1].scaleFactor;
            double back = drive.rightBack.getCurrentPosition() * ticksToInches * drive.localizer.encoders[2].scaleFactor;

            telemetry.addData("Cum Angle", Math.toDegrees(currentCumAngle));

            if(currentCumAngle != 0) {
                telemetry.addData("Right Odo Y", (-right)/currentCumAngle);
                telemetry.addData("Left Odo Y", (-left)/currentCumAngle);
                telemetry.addData("Back Odo Y", back/currentCumAngle);
            }
            telemetry.update();
        }
    }
}
