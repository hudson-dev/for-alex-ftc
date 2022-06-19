package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

@Config
@TeleOp(group="tuny boi")
public class headingPIDTuner extends LinearOpMode {
    public static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        double radius = 25;

        while(opModeIsActive()) {
            drive.followTrajectory(this,
                new Trajectory(new Pose2D(radius,radius,Math.toRadians(0),power), true)
                    .addLine(new Pose2D(-radius, radius, Math.toRadians(0), Math.toRadians(0), 15, power).rotateCoordinates())
                    .addLine(new Pose2D(-radius, -radius, Math.toRadians(0), Math.toRadians(0), 15, power).rotateCoordinates())
                    .addLine(new Pose2D(radius, -radius, Math.toRadians(0), Math.toRadians(0), 15, power).rotateCoordinates())
                    .addLine(new Pose2D(radius, radius, Math.toRadians(0), Math.toRadians(0), 15, power).rotateCoordinates())
            );
        }
    }
}
