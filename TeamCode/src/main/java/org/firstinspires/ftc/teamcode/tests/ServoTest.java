package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Button;

@TeleOp(group="test")
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        int i = 0;

        Button leftBumper = new Button();
        Button rightBumper = new Button();

        Button x = new Button();
        Button b = new Button();

        while(opModeIsActive()) {
            if(leftBumper.isClicked(gamepad1.left_bumper)) {
                i = (i+1) % drive.servos.size();
            }

            if(rightBumper.isClicked(gamepad1.right_bumper)) {
                i = (i+drive.servos.size()-1) % drive.servos.size();
            }

            double currentPos = Math.max(0, Math.min(drive.servos.get(i).getPosition(), 1));

            if(x.isClicked(gamepad1.x)) {
                currentPos += 0.1;
                drive.servos.get(i).setPosition(currentPos);
            }
            if(b.isClicked(gamepad1.b)) {
                currentPos -= 0.1;
                drive.servos.get(i).setPosition(currentPos);
            }

            if(gamepad1.y) {
                currentPos += 0.01;
                drive.servos.get(i).setPosition(currentPos);
            }

            if(gamepad1.a) {
                currentPos -= 0.01;
                drive.servos.get(i).setPosition(currentPos);
            }

            switch(i) {
                case(0):
                    telemetry.addData("Servo", "rightIntake");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(1):
                    telemetry.addData("Motor", "leftIntake");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(2):
                    telemetry.addData("Motor", "deposit");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(3):
                    telemetry.addData("Motor", "odoLift");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(4):
                    telemetry.addData("Motor", "v4bar");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(5):
                    telemetry.addData("Motor", "rightCapstone");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(6):
                    telemetry.addData("Motor", "leftCapstone");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(7):
                    telemetry.addData("Motor", "duckSpinSpin");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(8):
                    telemetry.addData("Motor", "rightOdo");
                    telemetry.addData("ServoPos", currentPos);
                    break;
                case(9):
                    telemetry.addData("Motor", "leftOdo");
                    telemetry.addData("ServoPos", currentPos);
                    break;
            }

            telemetry.update();
        }
    }
}
