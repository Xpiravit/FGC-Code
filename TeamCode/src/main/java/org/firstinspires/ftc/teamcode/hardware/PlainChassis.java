package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.utilities.IMUWraper;


/**
 * A Java Class meant to represents a robot's chassis for a square 45 degree omniwheel drivetrain.
 * This class is utilizing the REV Core Hex motors' encoders to calculate the robot's position at each given time and a basic
 * PID controller to make the rotations around its axis smoother.
 *
 * @author Xpiravit
 */
public class PlainChassis
{
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMUWraper gyro;

    private final Telemetry telemetry;

    private final PIDFController turnController;

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    private static final double TICKS_PER_REVOLUTION = 336;

    public PlainChassis(@NonNull HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        this.frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        this.backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        this.backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        this.gyro = new IMUWraper(hardwareMap, "imu");

        this.gyro.reset();

        this.telemetry = telemetry;

        this.turnController = new PIDFController(new PIDCoefficients(0.015,0.000,0.004));

        initializeMotors();
        stopAndResetEncoders();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeMotors()
    {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopAndResetEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
    {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
    }


    public void setPower(double drive, double strafe, double rotate)
    {
        double frontLeftPower = drive - strafe - turnController.update(rotate);
        double frontRightPower = drive + strafe + turnController.update(rotate);
        double backLeftPower = drive + strafe - turnController.update(rotate);
        double backRightPower = drive - strafe + turnController.update(rotate);

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0)
        {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        updatePositionAndHeading();
    }

    private void updatePositionAndHeading()
    {
        double currentHeading = gyro.getCurrentHeading();

        double frontLeftDistance = frontLeft.getCurrentPosition() / TICKS_PER_REVOLUTION;
        double frontRightDistance = frontRight.getCurrentPosition() / TICKS_PER_REVOLUTION;
        double backLeftDistance = backLeft.getCurrentPosition() / TICKS_PER_REVOLUTION;
        double backRightDistance = backRight.getCurrentPosition() / TICKS_PER_REVOLUTION;

        double averageDistance = (frontLeftDistance + frontRightDistance + backLeftDistance + backRightDistance) / 4.0;

        double deltaX = averageDistance * Math.sin(Math.toRadians(currentHeading));
        double deltaY = averageDistance * Math.cos(Math.toRadians(currentHeading));

        robotX += deltaX;
        robotY += deltaY;
        robotHeading = currentHeading;
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    public double getRobotHeading() {
        return robotHeading;
    }

    public void resetGyro() { gyro.reset(); }

}


