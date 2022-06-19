package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;

public class Trajectory {
    public ArrayList<Pose2D> points;
    boolean slowDown;

    public Trajectory(Pose2D a, boolean slowDown) {
        points = new ArrayList<Pose2D>();
        points.add(a);
        this.slowDown = slowDown;
    }

    public Trajectory addLine(Pose2D end) {
        Pose2D start = points.get(points.size() - 1);

        points.set(
            points.size() - 1,
            new Pose2D(
                start.x,
                start.y,
                start.heading,
                end.headingOffset,
                end.radius,
                start.speed
        ));
        double i = 0;
        ArrayList<Pose2D> newPoints = new ArrayList<Pose2D>();
        double d = Math.sqrt(Math.pow(start.x - end.x , 2) + Math.pow(start.y - end.y , 2));

        while (i <= 1) {
            i += 0.01;

            newPoints.add(
                    new Pose2D(
                        start.x + (end.x - start.x) * i,
                        start.y + (end.y - start.y) * i,
                        end.heading,
                        end.headingOffset,
                        end.radius,
                        Math.min(Math.max((d-end.radius-4) / 16 * i * (end.speed-start.speed) + start.speed, Math.min(start.speed, end.speed)), Math.max(start.speed, end.speed))
                    )
                );
        }

        points.addAll(newPoints);
        Trajectory a = new Trajectory(new Pose2D(0,0), slowDown);
        a.points = points;
        return a;
    }

    public Trajectory end() {
        if(slowDown) {
            points.get(points.size() - 1).radius = 4;
            points.get(points.size() - 1).speed= 0.2;

            for(int i = 2; i < points.size(); i++) {
                Pose2D one = points.get(points.size() - 1);
                Pose2D two = points.get(points.size() - i);

                double d = Math.sqrt(Math.pow(one.x - two.x , 2) + Math.pow(one.y - two.y , 2));
                double speed = 0.2 + Math.max((d-8) / 15, 0);
                double radius = 4 + Math.max((d-5) * 0.6666, 0);

                if (speed >= 1) {
                    Trajectory a = new Trajectory(new Pose2D(0,0), slowDown);
                    a.points = points;
                    return a;
                }

                points.get(points.size() - i).speed = Math.min(points.get(points.size() - i).speed, speed);
                points.get(points.size() - i).radius = Math.min(points.get(points.size() - i).radius, radius);
            }
        }
        Trajectory a = new Trajectory(new Pose2D(0,0), slowDown);
        a.points = points;
        return a;
    }

    public void update (Pose2D currentPose, Pose2D currentRelativeVelocity) {
        double vel = Math.sqrt(Math.pow(currentRelativeVelocity.x, 2) + Math.pow(currentRelativeVelocity.y, 2));

        while(points.size() > 1 && (Math.sqrt(Math.pow(points.get(0).x - currentPose.x, 2) + Math.pow(points.get(0).y - currentPose.y, 2)) <= points.get(0).radius)) {
            points.remove(0);
        }

        if (points.size() == 1) {
            if(slowDown) {
                double headingError = currentPose.heading - points.get(0).heading;
                while (Math.abs(headingError) > Math.PI) {
                    headingError -= 2*Math.PI * Math.signum(headingError);
                }

                if(vel <= 4 && (Math.sqrt(Math.pow(points.get(0).x - currentPose.x, 2) + Math.pow(points.get(0).y - currentPose.y, 2)) <= 2) && Math.abs(headingError) < Math.toRadians(5))  {
                    points.remove(0);
                }
            } else {
                if((Math.sqrt(Math.pow(points.get(0).x - currentPose.x, 2) + Math.pow(points.get(0).y - currentPose.y, 2)) <= points.get(0).radius)) {
                    points.remove(0);
                }
            }
        }
    }
}
