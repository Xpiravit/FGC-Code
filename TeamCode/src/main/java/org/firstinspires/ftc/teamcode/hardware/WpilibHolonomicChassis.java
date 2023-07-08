package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.utilities.IMUWraper;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A Java Class meant to represents a robot's chassis for a square 45 degree omniwheel drivetrain.
 * This class is utilizing tools found in wpilib's packages. Thus, pleases familiarize yourself with wpilib before making changes to code below
 * Thus any errors in the localization, PID calculations or the management of the robots trajectory derive from use of mistuned algorithm parameters, all parameters should be empirically tested and changed accordingly.
 *
 * @author Xpiravit
 */

public class WpilibHolonomicChassis
{

//    private static final double WHEEL_RADIUS = 0.09 / 2.0; //meters
//    private static final int ENCODER_TICKS_PER_REVOLUTION = 336;
    private static final double WHEEL_DISTANCE_FROM_CENTER = 0.29; //meters
    private static final int MAX_LINEAR_VELOCITY = 1;
    private static final int MAX_ANGULAR_VELOCITY = 1;

    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;

    private final Rotation2d r = new Rotation2d(0);
    public Pose2d currentPose = new Pose2d(0 ,0, r);
    private final HolonomicDriveController driveController;
    public final MecanumDrivePoseEstimator poseEstimator;

    IMUWraper gyro;
    Telemetry telemetry;

    /**
     * Constructs a WpilibHolonomicChassis with 4 motors, an IMU, a HolonomicDriveController (implementing PID for the x,y and angular velocities)
     * and a <a href = "https://www.motioncontroltips.com/what-is-a-motion-profile/">trapezoidal motion profile</a> for better motion control.
     *
     * <p><i>The names of the motors must be configured in the following format: "front_left_motor"</i>
     *
     * @param hardwareMap An object of the HardwareMap class used to assign the motors and imu
     * @param telemetry Used when debug mode is activated to show all values as they are being processed
     */


    public WpilibHolonomicChassis(@NonNull HardwareMap hardwareMap, Telemetry telemetry)
    {
//        Initializing Hardware
        this.frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        this.frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        this.backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        this.backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        this.gyro = new IMUWraper(hardwareMap, "imu");

        this.gyro.reset();

        this.telemetry = telemetry;

        Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(gyro.getCurrentHeading()));

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1,1);

        // ALL PID VALUES MUST BE EMPIRICALLY TESTED AND ACCORDINGLY TUNED
        PIDController xController = new PIDController(.015, 0.0000, .0004);
        PIDController yController = new PIDController(.05, 0.0000, .004);
        ProfiledPIDController thetaController = new ProfiledPIDController(.015, 0.000, .0004, constraints);

        this.driveController = new HolonomicDriveController(xController, yController, thetaController);

        Translation2d frontLeftMeters = new Translation2d(0, 0);
        Translation2d frontRightMeters = new Translation2d(0, 0);
        Translation2d backLeftMeters = new Translation2d(0, 0);
        Translation2d backRightMeters = new Translation2d(0, 0);

        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftMeters, frontRightMeters, backLeftMeters, backRightMeters);

        Rotation2d angle = new Rotation2d(gyro.getCurrentHeading()); // Degrees

        MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(WHEEL_DISTANCE_FROM_CENTER, WHEEL_DISTANCE_FROM_CENTER, WHEEL_DISTANCE_FROM_CENTER, WHEEL_DISTANCE_FROM_CENTER); //0?

        this.poseEstimator = new MecanumDrivePoseEstimator(kinematics, angle, wheelPositions, initialPose);
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

    public void setZeroPowerBehavior(ZeroPowerBehavior behavior)
    {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
    }

    public void setPower(double drive, double strafe, double rotate, boolean debugMode)
    {
        double maxPower;

        double desiredLinearVelocity = drive * MAX_LINEAR_VELOCITY;

        Rotation2d desiredHeading = new Rotation2d((rotate - gyro.getCurrentHeading()) * MAX_ANGULAR_VELOCITY); // Maybe the constant and/or the subtraction can be skipped

        Pose2d desiredPose = new Pose2d(drive, strafe, desiredHeading);
        Pose2d currentPose = poseEstimator.getEstimatedPosition();

        ChassisSpeeds chassisSpeeds = driveController.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredHeading);

//      double flPower = chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
//      double frPower = chassisSpeeds.vxMetersPerSecond - chassisSpeeds.vyMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;
//      double blPower = chassisSpeeds.vxMetersPerSecond - chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
//      double brPower = chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;

        double flPower = + chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;
        double frPower = - chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
        double blPower = - chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;
        double brPower = + chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;


        Rotation2d currentAngle = new Rotation2d(gyro.getCurrentHeading());
        // According to the Documentation the update() method takes as a parameter an object of the MechanumDriveWheelSpeed class
        // However the constructor takes one of MecanumDriveWheelPositions, so...
        MecanumDriveWheelPositions currentWheelSpeed = new MecanumDriveWheelPositions(flPower,frPower,blPower,brPower);
        this.currentPose = poseEstimator.update(currentAngle, currentWheelSpeed);

        maxPower = Math.max(Math.abs(flPower), Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

        //Normalizing Values
        if (maxPower > 1.0)
        {
            flPower = flPower / maxPower;
            frPower = frPower / maxPower;
            blPower = blPower / maxPower;
            brPower = brPower / maxPower;
        }

        this.frontLeft.setPower(flPower);
        this.frontRight.setPower(frPower);
        this.backLeft.setPower(blPower);
        this.backRight.setPower(brPower);

        if (debugMode)
        {
            telemetry.addData("Stick_X", drive);
            telemetry.addData("Stick_Y", strafe);
            telemetry.addData("Magnitude", Math.sqrt(Math.pow(drive, 2) + Math.pow(strafe, 2)));
            telemetry.addData("Front Left", flPower);
            telemetry.addData("Back Left", blPower);
            telemetry.addData("Back Right", brPower);
            telemetry.addData("Front Right", frPower);
        }
    }

    public void setCardinalAngle(boolean northInput, boolean eastInput, boolean southInput, boolean westInput)
    {
        if (northInput) setPower(0, 0, -StrictMath.abs(gyro.getCurrentHeading() - 0.0), true);
        else if (eastInput) setPower(0, 0, -StrictMath.abs(gyro.getCurrentHeading() - 270.0), true);
        else if (southInput) setPower(0, 0, -StrictMath.abs(gyro.getCurrentHeading() - 180.0), true);
        else if (westInput) setPower(0, 0, -StrictMath.abs(gyro.getCurrentHeading() - 90.0), true);
    }

    public void resetGyro(){
        gyro.reset();
    }





}
