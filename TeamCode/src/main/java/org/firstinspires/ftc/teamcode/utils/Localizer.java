package org.firstinspires.ftc.teamcode.utils;

import android.icu.util.ULocale;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

import java.util.ArrayList;

public class Localizer {
    public Encoder[] encoders;
    long lastTime = System.nanoTime()-1000000;
    public double x = 0, y = 0, heading = 0, startingHeading = 0;

    public ArrayList<Pose2D> poseHistory = new ArrayList<>();
    public ArrayList<Pose2D> relativeHistory = new ArrayList<>();
    public ArrayList<Double> loopTimes = new ArrayList<>();

    public Pose2D currentPose = new Pose2D(0,0,0,0);
    public Pose2D lastPose = new Pose2D(0,0,0,0);
    public Pose2D currentVel = new Pose2D(0,0,0,0);
    public Pose2D relativeCurrentVel = new Pose2D(0,0,0,0);
    public Pose2D currentPowerVector = new Pose2D(0,0,0,0);

    public double lastIMUHeading = 0;

    public boolean callIMU = false;
    public int counter = 0;

    public BNO055IMU imu;

    public Pose2D leftSensor = new Pose2D(0,0,0);
    public Pose2D rightSensor = new Pose2D(0,0,0);

    public Localizer () {
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2D(0,-4.087365365470633), 1); // rightFront
        encoders[1] = new Encoder(new Pose2D(0, 5.2195710226290455), -1); // leftFront
        encoders[2] = new Encoder(new Pose2D(0,2.1001917100567495), -1); // rightBack

        Pose2D pos2D = new Pose2D(0,0,0);
    }

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / (1e9);
        loopTimes.add(0, loopTime);
        lastTime = currentTime;

        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();

        // random mathematics
        double deltaHeading = (deltaRight - deltaLeft) / (encoders[1].y - encoders[0].y);
        double relativeDeltaX = ((deltaRight*encoders[1].y) - (deltaLeft*encoders[0].y)) / (encoders[0].y - encoders[1].y);
        double relativeDeltaY = deltaBack-(encoders[2].x*deltaHeading);

        heading += deltaHeading;
        relativeHistory.add(0,new Pose2D(relativeDeltaX, relativeDeltaY, deltaHeading));

        // robot moves in arcs
        if(deltaHeading != 0) {
            double r1 = relativeDeltaX/deltaHeading;
            double r2 = relativeDeltaY/deltaHeading;

            relativeDeltaX = Math.sin(deltaHeading) * r1 + (1.0-Math.cos(deltaHeading)) * r2;
            relativeDeltaY = Math.sin(deltaHeading) * r2 + (1.0-Math.cos(deltaHeading)) * r1;
        }

        double lastHeading = heading - deltaHeading;

        x += relativeDeltaX*Math.cos(lastHeading) - relativeDeltaY*Math.sin(lastHeading);
        y += relativeDeltaY*Math.cos(lastHeading) + relativeDeltaX*Math.sin(lastHeading);

        currentPose = new Pose2D(x,y,heading);
        poseHistory.add(0, new Pose2D(x,y,heading));

        updateVelocity();

        // 10 loops ago
        if(counter == 0 && (Math.abs(relativeCurrentVel.getY()) > 6 || Math.abs(relativeCurrentVel.getY()) / Math.max(Math.abs(relativeCurrentVel.getX()),0.1) > 1)) {
            counter = 10;
            lastPose = new Pose2D(x,y,heading);
            lastIMUHeading = imu.getAngularOrientation().firstAngle;
        }

        if(counter > 0) {
            counter--;
            if(counter==0) {
                double headingError = (heading - imu.getAngularOrientation().firstAngle) - (lastPose.heading - lastIMUHeading);
                heading -= headingError;

                double deltaX = x - lastPose.x;
                double deltaY = y - lastPose.y;

                x = lastPose.x + Math.cos(-headingError) * deltaX - Math.sin(-headingError) * deltaY;
                y = lastPose.y + Math.cos(-headingError) * deltaY + Math.sin(-headingError) * deltaX;

            }
        }
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;

        for (int i = 0; i < loopTimes.size(); i++) {
            totalTime += loopTimes.get(i);
            if(totalTime <= targetVelTimeEstimate) {
                actualVelTime += loopTimes.get(i);
                relDeltaXTotal += relativeHistory.get(i).getX();
                relDeltaYTotal += relativeHistory.get(i).getY();
                lastIndex = i;
            }
        }

        currentVel = new Pose2D(
                (poseHistory.get(0).getX() - poseHistory.get(lastIndex).getX())/actualVelTime,
                (poseHistory.get(0).getY() - poseHistory.get(lastIndex).getY())/actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) /actualVelTime
        );

        relativeCurrentVel = new Pose2D(
                relDeltaXTotal / actualVelTime,
                relDeltaYTotal / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) /actualVelTime
        );

        while(lastIndex + 1 < loopTimes.size()) {
            loopTimes.remove( loopTimes.size()-1);
            poseHistory.remove(poseHistory.size()-1);
            relativeHistory.remove(relativeHistory.size()-1);
        }
    }

    public void updateEncoders(int[] array) {
        for (int i = 0; i < array.length; i++) {
            encoders[i].update(array[i]);
        }
    }

    public void distUpdate(double rightDist, double leftDist) {
        double rightSensorX = x + Math.cos(heading) * 8 - Math.sin(heading) * -6; // global x position of sensor
        double rightSensorY = y + Math.cos(heading) * -6 + Math.sin(heading) * 8; // global y position of sensor

        double leftSensorX = x + Math.cos(heading) * 8 - Math.sin(heading) * 6; // global x position of sensor
        double leftSensorY = y + Math.cos(heading) * 6 + Math.sin(heading) * 8; // global y position of sensor

        double xErrorLeft = 0;
        double yErrorLeft = 0;

        if(((Math.abs(leftSensorY) < 64) && (Math.abs(leftSensorX) < 64)) && leftDist > 24) { // make sure the sensor is not close enough to wall. Center of field is (0,0)
            leftSensorX += Math.cos(heading) * leftDist; // wall dist x
            leftSensorY += Math.sin(heading) * leftDist; // wall dist y

            if((Math.abs(Math.abs(leftSensorX) - 72) < 3) ^ (Math.abs(Math.abs(leftSensorY) - 72) < 3)) { // checks if pointing towards corner, if so not accurate
                // accounts for error. The error is equal to the mismatch in distances between the ultrasonic's value to the wall and the odometry's value to the wall
                if(Math.abs(Math.abs(leftSensorX) - 72) < 3) {
                    // Math.signum = -1 , 0, +1
                    xErrorLeft = 72 * Math.signum(leftSensorX) - leftSensorX;
                } else {
                    yErrorLeft = 72 * Math.signum(leftSensorY) - leftSensorY;
                }
            }

        } else {
            leftSensorX += Math.cos(heading) * leftDist;
            leftSensorY += Math.sin(heading) * leftDist;
        }

        double xErrorRight = 0;
        double yErrorRight = 0;

        // make sure the sensor is not too close to wall to avoid interference. Center of field is (0,0) so lower x,y coordinates means it's closer to the center and thus farther away from wall.
        if(((Math.abs(rightSensorY) < 64 && Math.abs(rightSensorX) < 64)) && rightDist > 24) {
            rightSensorX += Math.cos(heading) * rightDist; // wall dist x
            rightSensorY += Math.sin(heading) * rightDist; // wall dist y

            if((Math.abs(Math.abs(rightSensorX) - 72) < 3) ^ (Math.abs(Math.abs(rightSensorY) - 72) < 3)) { // checks if pointing towards corner, if so not accurate
                // accounts for error. The error is equal to the mismatch in distances between the ultrasonic's value to the wall and the odometry's value to the wall
                if(Math.abs(Math.abs(rightSensorX) - 72) < 3) {
                    // Math.signum = -1 , 0, +1
                    xErrorRight = 72 * Math.signum(rightSensorX) - rightSensorX;
                } else {
                    yErrorRight = 72 * Math.signum(rightSensorY) - rightSensorY;
                }
            }

        } else {
            rightSensorX += Math.cos(heading) * rightDist;
            rightSensorY += Math.sin(heading) * rightDist;
        }

        x += xErrorLeft * 0.005 * Math.pow(2, -(leftDist-32)/18) + xErrorRight * 0.005 * Math.pow(2, -(rightDist-32)/18);
        y += yErrorLeft * 0.005 * Math.pow(2, -(leftDist-32)/18) + yErrorRight * 0.005 * Math.pow(2, -(rightDist-32)/18);

        leftSensor = new Pose2D(leftSensorX, leftSensorY);
        rightSensor = new Pose2D(rightSensorX, rightSensorY);

    }

    public void updateFlex(double flexVal) {
        int valPressed = 333;
        if (flexVal < valPressed) {
            double sensorX = x + Math.cos(heading) * 5 - Math.sin(heading) * (-6.75 * Math.signum(y)); // x position of flex sensor relative to robot
            double sensorY = y + Math.cos(heading) * (-6.75 * Math.signum(y)) + Math.sin(heading) * 5; // y position of flex sensor relative to robot

            if(Math.abs(sensorX) >= 68) { // 4 inches of error
                x += (72 * Math.signum(sensorX) - sensorX) * 0.5;
            }

            if(Math.abs(sensorY) >= 68) { // 4 inches of error
                x += (72 * Math.signum(sensorY) - sensorY) * 0.5;
            }
        }
    }
}
