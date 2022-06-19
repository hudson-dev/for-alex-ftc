package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.Localizer;
import org.firstinspires.ftc.teamcode.utils.Logging;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Trajectory;
import org.firstinspires.ftc.teamcode.utils.UpdatePriority;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SampleMecanumDrive {

    //ftc-dashboard
    FtcDashboard dashboard;

    // bulk read sensors
    RevBulkData bulkData;

    // expansion hubs
    ExpansionHubEx expansionHub1, expansionHub2;

    // motors
    public ExpansionHubMotor leftFront, leftBack, rightFront, rightBack;
    public ExpansionHubMotor intake, slides, slides2, turret;

    // continuous rotation servos
    CRServo duckSpin;
    CRServo duckSpin2;

    // list of motors and servos
    public List<ExpansionHubMotor> motors;
    public List<Servo> servos;

    // analog sensors + voltage battery sensor
    public AnalogInput leftIntake, rightIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;
    VoltageSensor battery;

    // imu
    public BNO055IMU imu;

    // localizer
    public Localizer localizer = new Localizer();

    public int[] encoders = new int[3];

    public int loops;
    public long start;

    ArrayList<UpdatePriority> motorPriorities = new ArrayList<UpdatePriority>();

    boolean updateHub2 = false;

    // intake
    public double intakeTurretInterfaceHeading = Math.toRadians(57.5);
    public static double v4barInterfaceAngle = 0.15;//0; //Math.toRadians(7);
    public double depositAngle = Math.toRadians(-45);
    public double effectiveDepositAngle = Math.toRadians(-45);
    public static double depositInterfaceAngle = 0.8; //Math.toRadians(65);
    public double depositTransferAngle = Math.toRadians(135);

    public double leftIntakeDrop;
    public double leftIntakeRaise;
    public double rightIntakeDrop;
    public double rightIntakeRaise;
    public double leftIntakeMid;
    public double rightIntakeMid;

    int numRightIntake = 0;
    int numLeftIntake = 0;

    ArrayList<Double> depositHistory, intakeHistory, sideMoves;
    public double currentIntake = 0;
    double rightIntakeVal, leftIntakeVal, depositVal, sumIntakeSensor, intakeSensorLoops;
    public static int intakeMinValRight = 200;//35;//75
    public static int intakeMinValLeft = 100;// = 35;//75
    public int numZeroLeft = 0;
    public int numZeroRight = 0;
    public int intakeCase, lastIntakeCase;
    public long intakeTime, slideTime;
    public boolean transferMineral;
    public boolean startSlides = false;
    public boolean deposit = false;

    public double slideExtensionLength = 0;
    public double intakePos = 0;
    public double intakeSpeed = 0;
    public double intakeTicksPerRev = ((1.0+(46.0/11.0)) * 28.0) / (26.0/19.0);
    public double turretHeading = 0;
    public double targetSlideExtensionLength = 0;
    public double targetTurretHeading = 0;
    public double targetV4barOrientation = 0;
    public double slideTickToInch = 25.1372713591;
    public double turretTickToRadians = 578.3213;
    public double currentSlidesSpeed = 0;
    static double currentV4barAngle = 0;
    double targetV4barAngle = 0;

    public int dropIntakeTime = 380;
    public double intakePower = -1;
    public int liftIntakeTime = 700;
    public int transfer1Time = 215; //300
    public int transfer2Time = 235; //350
    public double transfer1Power = 1.0;
    //public double transfer2Power = 0.78;
    public int openDepositTime = 250; //400
    public int intakeLiftDelay = 100;
    public int effectiveDepositTime = openDepositTime;
    public double returnSlideLength = 0.35; //0.75
    double turretI = 0;

    public boolean intakeDepositTransfer = false, intakeHit = false;
    long startIntakeDepositTransfer, startIntakeHit;

    boolean firstSlide = false;
    long slideStart = System.currentTimeMillis();

    long transferTime = System.currentTimeMillis();

    double targetDepositAngle = 0;

    double targetSlidesPose = 0, slidesSpeed = 0, targetTurretPose = 0, turretPower = 0;
    double currentTargetSlidesPose = 0;

    double currentDepositAngle = depositInterfaceAngle;

    public Pose2D targetPoint = new Pose2D(0,0,0);

    // constructor
    public SampleMecanumDrive(HardwareMap hardwareMap) {
        initMotors(hardwareMap); // sets variables to physical motor + expansionHub
        initServos(hardwareMap); // sets variables to physical servos
        initSensors(hardwareMap); // sets variables to physical sensors

        localizer.imu = this.imu;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        Log.e("","-------------------Hudson's Code-------------------");
    }

    public void initMotors(HardwareMap hardwareMap) {
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");
        rightBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");
        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets encoders
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // runs without encoder since using odometry

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2);

        for (int i = 0; i < 4; i++) {
            motorPriorities.add(new UpdatePriority(3,5));
        }

        motorPriorities.add(new UpdatePriority(1,2)); // intake
        motorPriorities.add(new UpdatePriority(1,3)); // turret
        motorPriorities.add(new UpdatePriority(2,6)); // slides
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void initServos(HardwareMap hardwareMap) {
        servos = new ArrayList<>();

        servos.add(hardwareMap.servo.get("rightIntake"));
        servos.add(hardwareMap.servo.get("leftIntake"));
        servos.add(hardwareMap.servo.get("deposit"));
        servos.add(hardwareMap.servo.get("odoLift"));
        servos.add(hardwareMap.servo.get("v4bar"));
        servos.add(hardwareMap.servo.get("rightCapstone"));
        servos.add(hardwareMap.servo.get("leftCapstone"));
        servos.add(hardwareMap.servo.get("duckSpinSpin"));
        servos.add(hardwareMap.servo.get("rightOdo"));
        servos.add(hardwareMap.servo.get("leftOdo"));

        duckSpin = hardwareMap.crservo.get("duckSpin");
        duckSpin2 = hardwareMap.crservo.get("duckSpin2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void initSensors(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.analogInput.get("leftIntake");
        rightIntake = hardwareMap.analogInput.get("rightIntake");
        depositSensor = hardwareMap.analogInput.get("depositSensor");
        distLeft = hardwareMap.analogInput.get("distLeft");
        distRight = hardwareMap.analogInput.get("distRight");
        magLeft = hardwareMap.analogInput.get("magLeft");
        magRight = hardwareMap.analogInput.get("magRight");
        flex = hardwareMap.analogInput.get("flex");

        battery = hardwareMap.voltageSensor.iterator().next();

        for(LynxModule lynxModule : hardwareMap.getAll(LynxModule.class)) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // automatically caches bulkReads
        }
    }

    public void setMotorPowers(double one, double two, double three, double four) {
//        leftFront.setPower(one);
//        leftBack.setPower(two);
//        rightBack.setPower(three);
//        rightFront.setPower(four);

        motorPriorities.get(0).setTargetPower(one); // leftFront
        motorPriorities.get(1).setTargetPower(two); // leftBack
        motorPriorities.get(2).setTargetPower(three); // rightBack
        motorPriorities.get(3).setTargetPower(four); // rightFront
    }

    public void drive(LinearOpMode opMode) {
        double forward = -opMode.gamepad1.left_stick_y;
        double left = opMode.gamepad1.left_stick_x;
        double turn = opMode.gamepad1.right_stick_x;

        double p1 = forward+left+turn;
        double p2 = forward-left+turn;
        double p3 = forward+left-turn;
        double p4 = forward-left-turn;

        setMotorPowers(p1,p2,p3,p4);

        update();
    }
    double loopTime = 0;
    public void update() {
        // loop times
        long loopStart = System.nanoTime();

        if(loops == 0) {
            start = System.currentTimeMillis();
        }
        loops++;

        // bulkRead encoders
        getEncoders();
        updateHub2();

        // motorPriorities
        loopTime = (System.nanoTime() - loopStart) / (double) 1e9;
        double targetLoopLength = 0.01;
        double bestMotorUpdate = 1;
        int numMotorsUpdated = 0;

        while (bestMotorUpdate > 0 && (loopTime <= targetLoopLength)) {
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority();
            for(int i = 1; i < motorPriorities.size(); i++) {
                if(motorPriorities.get(i).getPriority() > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = motorPriorities.get(i).getPriority();
                }
            }

            if(bestMotorUpdate != 0) { // all the other motors (including slides1)
                numMotorsUpdated++;
                motors.get(bestIndex).setPower(motorPriorities.get(bestIndex).power);
                if (bestIndex == motorPriorities.size() - 1) { // slides2
                    numMotorsUpdated++;
                    slides2.setPower(motorPriorities.get(bestIndex).power);
                }
                motorPriorities.get(bestIndex).update();
            }
            loopTime = (System.nanoTime() - loopStart) / (double) 1e9;
        }

        // telemetry & ftc-dashboard
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.strokeCircle(localizer.x, localizer.y, 9);
        fieldOverlay.strokeLine(localizer.x, localizer.y, localizer.x + 11*Math.cos(localizer.heading), localizer.y+11*Math.sin(localizer.heading));
        fieldOverlay.strokeCircle(localizer.leftSensor.getX(), localizer.leftSensor.getY(), 2.4);
        fieldOverlay.strokeCircle(localizer.rightSensor.getX(), localizer.rightSensor.getY(), 2.4);
        fieldOverlay.setStroke("#ff0000");
        fieldOverlay.strokeCircle(targetPoint.getX(), targetPoint.getY(), 9);

        packet.put("Looooooooop speed ", (double) (System.currentTimeMillis() - start)/loops);
        packet.put("Loops: ", loops);
        packet.put("Num Motors Updated: ", numMotorsUpdated);
        packet.put("Heading Error: ", headingError);
        packet.put("Person: ", "Hudson");

        dashboard.sendTelemetryPacket(packet);

        updateHub2 = false; // only run once per loop
    }

    double currentIntakeSpeed;
    int flexSensorVal;
    double currentSlideLength;
    double currentSlideSpeed;
    double currentTurretAngle;
    double distValLeft, distValRight;
    int magValLeft, magValRight;
    double lastDistLeft, lastDistRight = 0.0;

    public void getEncoders() {
        bulkData = expansionHub1.getBulkInputData();
        if(bulkData != null) {
            try {
                encoders[0] = bulkData.getMotorCurrentPosition(leftFront);
                encoders[1] = bulkData.getMotorCurrentPosition(rightFront);
                encoders[2] = bulkData.getMotorCurrentPosition(rightBack);

                flexSensorVal = bulkData.getAnalogInputValue(flex);

                localizer.updateEncoders(encoders);
                localizer.updateFlex(flexSensorVal);
                localizer.update();
            } catch (Exception e) {
                Log.e("*** Exception class ***", e.getClass().getName());
                e.printStackTrace();
            }
        }
    }

    public void updateHub2() {
        if(!updateHub2) {
            updateHub2 = true;
            bulkData = expansionHub2.getBulkInputData();

            if(bulkData != null) {
                try {
                    // ticks to inches convert (gear ratio * circumference of spool / ticks per revolution of motor)
                    currentSlideLength = bulkData.getMotorCurrentPosition(slides2) / 25.1372713591;
                    currentSlideSpeed = bulkData.getMotorVelocity(slides2) / 25.1372713591;

                    // ticks to radians
                    currentTurretAngle = bulkData.getMotorCurrentPosition(turret) / 578.3213;

                    // volts to inches
                    distValLeft = bulkData.getAnalogInputValue(distLeft) / 3.2;
                    distValRight = bulkData.getAnalogInputValue(distRight) / 3.2;

                    // random number no conversion factor ¯\_(ツ)_/¯
                    magValLeft = bulkData.getAnalogInputValue(magLeft);
                    magValRight = bulkData.getAnalogInputValue(magRight);

                    if(distValLeft != lastDistLeft || distValRight != lastDistRight) {
                        localizer.distUpdate(distValLeft, distValRight);
                    }

                    lastDistLeft = distValLeft;
                    lastDistRight = distValRight;

                } catch(Exception e) {
                    Log.e("EXCEPTION: ", e.toString());
                }
            }
        }
    }

    public void driveToPoint (LinearOpMode opMode, Pose2D targetPoint) {
        this.targetPoint = targetPoint;
        double error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));

        while(opMode.opModeIsActive() && error > 3) {
            error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));
            update();

            double targetAngle = Math.atan2(targetPoint.y - localizer.y, targetPoint.x - localizer.x); // finds slope of line between currentPose and targetPose and then finds angle to there.
            double headingError = targetAngle - localizer.heading;

            // for heading errors outside of -180 --> 180 degrees
            while(headingError > Math.PI) {
                headingError -= 2*Math.PI;
            }

            while(headingError < -Math.PI) {
                headingError += 2*Math.PI;
            }

            double relativeErrorX = Math.cos(headingError) * error;
            double relativeErrorY = Math.sin(headingError) * error;

            double turn = Math.toDegrees(headingError) * 0.3 / 15; // 15 degrees per second
            double forward = (relativeErrorX / error) * 0.5 / (1.0 - Math.abs(turn));

            double left = (relativeErrorY / error) * 0.5 / (1.0 - Math.abs(turn));

            setMotorPowers(forward - left - turn, forward + left - turn, forward - left + turn, forward + left + turn);
        }
    }

    public static double headingP = 1.0;
    public static double headingI = 0.0;
    public static double headingD = 0.0;
    double headingIntegral = 0;
    double lastHeadingError = 0;
    double headingError = 0;

    public void followTrajectory (LinearOpMode opMode, Trajectory trajectory) {
        targetPoint = new Pose2D(0,0,0);

        while(opMode.opModeIsActive() && trajectory.points.size() != 0) {
            update();

            targetPoint = trajectory.points.get(0);
            double error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));
            double lastError = Math.sqrt(Math.pow(localizer.x - trajectory.points.get(trajectory.points.size() - 1).x, 2) + Math.pow(localizer.y - trajectory.points.get(trajectory.points.size() - 1).y, 2));

            double targetAngle = Math.atan2(targetPoint.y - localizer.y, targetPoint.x - localizer.x); // finds slope of line between currentPose and targetPose and then finds angle to there.
            headingError = targetAngle - localizer.heading;
            double relativeErrorX = Math.cos(headingError) * error;
            double relativeErrorY = Math.sin(headingError) * error;

            if(lastError < 8 && trajectory.points.size() < 100) {
                headingError = trajectory.points.get(trajectory.points.size() - 1).heading - localizer.heading;
            }

            // for heading errors outside of -180 --> 180 degrees
            while(headingError > Math.PI) {
                headingError -= 2*Math.PI;
            }

            while(headingError < -Math.PI) {
                headingError += 2*Math.PI;
            }

            headingIntegral += headingError * loopTime;
            double dHeadingError = (headingError - lastHeadingError) / loopTime;
            lastHeadingError = headingError;

            double turn = headingError * headingP + headingIntegral * headingI + dHeadingError*headingD;
            //Math.min(Math.pow(Math.toDegrees(headingError)/3,2) * 4/15 * targetPoint.speed, Math.min(0.6 * targetPoint.speed,0.4)) * Math.signum(headingError); // 15 degrees per second
            double forward = (relativeErrorX / error) * targetPoint.speed * (1.0 - Math.abs(turn));

            double left = (relativeErrorY / error) * targetPoint.speed * (1.0 - Math.abs(turn));

            setMotorPowers(forward - left - turn, forward + left - turn, forward - left + turn, forward + left + turn);
            trajectory.update(localizer.currentPose, localizer.currentVel);
        }
    }
}
