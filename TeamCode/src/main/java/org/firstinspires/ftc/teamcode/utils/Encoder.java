package org.firstinspires.ftc.teamcode.utils;

public class Encoder {
    double ticksToInches;
    int lastValue;
    int currentValue;
    public double scaleFactor;
    double x, y;

    public Encoder (Pose2D point, double scaleFactor) {
        final double ticksPerRotation = 8092;
        final double wheelRadius = 0.6889764;

        ticksToInches = (wheelRadius * Math.PI * 2) / ticksPerRotation; // finding how many ticks equals an inch

        x = point.getX();
        y = point.getY();
        currentValue = 0;
        lastValue = currentValue;
        this.scaleFactor = scaleFactor;
    }

    public void update(int currentPos) {
        lastValue = currentValue;
        currentValue = currentPos;
    }

    public double getDelta() {
        return (double) (currentValue - lastValue) * ticksToInches * scaleFactor;
    }

    public double getCurrentDistance() {
        return (double) currentValue * ticksToInches * scaleFactor;
    }
}
