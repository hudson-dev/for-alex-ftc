package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

@TeleOp(group="testy boi")
public class FollowTrajectory extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

//        drive.followTrajectory(this,
//            new Trajectory(new Pose2D(0,0,0,0.3), true)
//            .addLine(new Pose2D(-24, 30, Math.toRadians(40), Math.toRadians(0), 20, 0.5))
//            .addLine(new Pose2D(30, 28, Math.toRadians(-40), Math.toRadians(0), 20, 0.5))
//            .addLine(new Pose2D(-16, 20, Math.toRadians(20), Math.toRadians(0), 20, 0.5))
//            .addLine(new Pose2D(0, 0, Math.toRadians(0), Math.toRadians(0), 20, 0.5))
//            .end()
//        );

        drive.followTrajectory(this,
            new Trajectory(new Pose2D(0,0,0,0.75), true)
                .addLine(new Pose2D(30, 30, Math.toRadians(45), Math.toRadians(0), 15, 0.75).rotateCoordinates())
                .addLine(new Pose2D(-30, 30, Math.toRadians(-45), Math.toRadians(0), 15, 0.75).rotateCoordinates())
                .addLine(new Pose2D(-30, -30, Math.toRadians(-120), Math.toRadians(0), 15, 0.75).rotateCoordinates())
                .addLine(new Pose2D(30, -30, Math.toRadians(120), Math.toRadians(0), 15, 0.75).rotateCoordinates())
                .addLine(new Pose2D(30, 30, Math.toRadians(45), Math.toRadians(0), 15, 0.75).rotateCoordinates())
                .addLine(new Pose2D(0, -0, Math.toRadians(0), Math.toRadians(0), 15, 0.75).rotateCoordinates())
                .end()
        );
    }
}
