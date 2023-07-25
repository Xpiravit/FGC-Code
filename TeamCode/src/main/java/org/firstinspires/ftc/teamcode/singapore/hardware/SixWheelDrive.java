package org.firstinspires.ftc.teamcode.singapore.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.utilities.IMUWraper;
import org.firstinspires.ftc.teamcode.singapore.utils.SCurveControllerAcceleration;

public class SixWheelDrive
{

    private final DcMotor Left;
    private final DcMotor Right;
    public final IMUWraper gyro;

    protected final Telemetry telemetry;

    private final SCurveControllerAcceleration motionProfile;

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private int previousLeftEncoder;
    private int previousRightEncoder;
    private static final double TICKS_PER_REVOLUTION = 336;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
//    private static final double ROBOT_WIDTH_INCHES = 12;

    public SixWheelDrive(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 1.0, 0.1);
    }
    public SixWheelDrive(@NonNull HardwareMap hardwareMap, Telemetry telemetry, double maxPower, double accelerationRate)
    {
        this.Left = hardwareMap.get(DcMotor.class, "left_motor");
        this.Right = hardwareMap.get(DcMotor.class, "right_motor");

        this.gyro = new IMUWraper(hardwareMap, "imu");

        this.gyro.reset();

        this.telemetry = telemetry;

        this.motionProfile = new SCurveControllerAcceleration(maxPower, accelerationRate);

        initializeMotors();
        stopAndResetEncoders();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeMotors()
    {
        Left.setDirection(DcMotor.Direction.FORWARD);
        Right.setDirection(DcMotor.Direction.REVERSE);

        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopAndResetEncoders()
    {
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
    {
        Left.setZeroPowerBehavior(behavior);
        Right.setZeroPowerBehavior(behavior);
    }

    public void setPower(double drive, double turn)
    {
       setPower(false, drive, turn, false);
    }
    public void setPower(double drive, double turn, boolean motioncontrolEnabled) {
        setPower(false, drive, turn, motioncontrolEnabled);
    }
    public void setPower(boolean debugMode, double drive, double turn) {
        setPower(debugMode, drive, turn,false);
    }
    public void setPower(boolean debugMode, double drive, double turn, boolean motioncontrolEnabled)
    {
        double leftPower;
        double rightPower;

        if (motioncontrolEnabled)
        {
            double[] powers = motionProfile.getPowers(drive, turn);
            rightPower = powers[0];
            leftPower = powers[1];
        }
        else
        {
            leftPower = drive + turn;
            rightPower = drive - turn;
        }

        Right.setPower(rightPower);
        Left.setPower(leftPower);

        if (debugMode)
        {
            telemetry.addData("Stick_X", turn);
            telemetry.addData("Stick_Y", drive);
            telemetry.addData("Left", leftPower);
            telemetry.addData("Right", rightPower);
        }

        updatePositionAndHeading();
    }

    private void updatePositionAndHeading()
    {
        int currentLeftEncoder = Left.getCurrentPosition();
        int currentRightEncoder = Right.getCurrentPosition();

        int deltaLeftEncoder = currentLeftEncoder - previousLeftEncoder;
        int deltaRightEncoder = currentRightEncoder - previousRightEncoder;

        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;

        double leftDistance = deltaLeftEncoder / TICKS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;
        double rightDistance = deltaRightEncoder / TICKS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;

//        double deltaHeading = (leftDistance - rightDistance) / ROBOT_WIDTH_INCHES;
//        The IMU is used to calculate the current heading.

        double deltaDistance = (leftDistance + rightDistance) / 2.0;

        double deltaX = deltaDistance * Math.cos(Math.toRadians(robotHeading));
        double deltaY = deltaDistance * Math.sin(Math.toRadians(robotHeading));

        robotX += deltaX;
        robotY += deltaY;
        robotHeading = gyro.getCurrentHeading();
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    public double getHeading() {
        return robotHeading;
    }

    public void resetGyro() {
        gyro.reset();
    }


}
