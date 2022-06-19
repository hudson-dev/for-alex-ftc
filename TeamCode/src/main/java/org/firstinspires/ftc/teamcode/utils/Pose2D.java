package org.firstinspires.ftc.teamcode.utils;

public class Pose2D {
    public double x, y, heading, headingOffset, radius, speed;

    public Pose2D(double x, double y, double heading, double headingOffset, double radius, double speed) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.headingOffset = headingOffset;
        this.radius = radius;
        this.speed = speed;
    }

    public Pose2D(double x, double y) {
        this(x,y,0,0,0,0);
    }

    public Pose2D(double x, double y, double heading) {
        this(x,y,heading,0,0,0);
    }

    public Pose2D(double x, double y, double heading, double speed) {
        this(x,y,heading,0,0, speed);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Pose2D rotateCoordinates() {
        return new Pose2D(this.y,-this.x, this.heading, this.headingOffset, this.radius, this.speed);
    }

    public double getHeading() {
        return heading;
    }
}
