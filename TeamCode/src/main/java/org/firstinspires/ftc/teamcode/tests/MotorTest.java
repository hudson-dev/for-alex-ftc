package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="test")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        double[] motorPower = new double[drive.motors.size()];
        int i = 0;

        boolean lastA = false;

        while(opModeIsActive()) {
            motorPower[i] = gamepad1.right_stick_y * -1;
            boolean a = gamepad1.a;

            drive.motors.get(i).setPower(motorPower[i]);

            switch(i) {
                case(0):
                    telemetry.addData("Motor", "leftFront");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(1):
                    telemetry.addData("Motor", "leftBack");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(2):
                    telemetry.addData("Motor", "rightFront");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(3):
                    telemetry.addData("Motor", "rightBack");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(4):
                    telemetry.addData("Motor", "intake");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(5):
                    telemetry.addData("Motor", "slides");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(6):
                    telemetry.addData("Motor", "slides2");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(7):
                    telemetry.addData("Motor", "turret");
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
            }

            telemetry.update();

            if(a && !lastA) {
                if (i == 7) {
                    i = 0;
                } else {
                    i++;
                }
            }

            lastA = a;
        }
    }
}
