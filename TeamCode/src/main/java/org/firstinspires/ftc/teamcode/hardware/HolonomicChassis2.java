package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Translation2d;


public class HolonomicChassis2
{

//    private static final double WHEEL_RADIUS = 0.09 / 2.0; //meters
    private static final double WHEEL_DISTANCE_FROM_CENTER = 0.29; //meters
//    private static final int ENCODER_TICKS_PER_REVOLUTION = 336;
    private static final int MAX_LINEAR_VELOCITY = 1;
    private static final int MAX_ANGULAR_VELOCITY = 1;

    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;

    private final Pose2d initialPose;
    private final Rotation2d r = new Rotation2d(0);
    public Pose2d currentPose = new Pose2d(0 ,0, r);
    private final HolonomicDriveController driveController;
    private final MecanumDrivePoseEstimator poseEstimator;

    Gyro gyro;
    Telemetry telemetry;

//    // Initialization
//    public void initPoseEstimator() {
//        // Specify the initial pose of the robot
//        Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
//
//        // Specify the positions of the wheel encoders relative to the robot's center
//        Translation2d frontLeftEncoderPosition = new Translation2d(0.5, 0.5);
//        Translation2d frontRightEncoderPosition = new Translation2d(0.5, -0.5);
//        Translation2d rearLeftEncoderPosition = new Translation2d(-0.5, 0.5);
//        Translation2d rearRightEncoderPosition = new Translation2d(-0.5, -0.5);
//
//        // Specify the gyroscope measurement noise in radians
//        double gyroMeasurementNoise = 0.01;
//
//        // Create the MecanumDrivePoseEstimator
//        poseEstimator = new MecanumDrivePoseEstimator(
//                initialPose,
//                frontLeftEncoderPosition,
//                frontRightEncoderPosition,
//                rearLeftEncoderPosition,
//                rearRightEncoderPosition,
//                gyroMeasurementNoise);
//    }
//
//    // Update the pose estimator with new sensor readings
//    public void updatePoseEstimator(MecanumDriveWheelSpeeds wheelSpeeds, double gyroAngle) {
//        poseEstimator.update(wheelSpeeds, gyroAngle);
//    }
//
//    // Get the current estimated pose
//    public Pose2d getCurrentPose() {
//        return poseEstimator.getPoseMeters();
//    }


    public HolonomicChassis2(DcMotor frontLeft, DcMotor frontRight, DcMotor backRight, DcMotor backLeft, Gyro gyro, Telemetry telemetry, Pose2d pose)
    {
//        Initializing Hardware
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.gyro = gyro;

        this.telemetry = telemetry;

        this.initialPose = pose;

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1,1);
        PIDController xController = new PIDController(.015, 0.0000, .0004);
        PIDController yController = new PIDController(.05, 0.0000, .004);
        ProfiledPIDController thetaController = new ProfiledPIDController(.05, 0.000, .004, constraints);

        this.driveController = new HolonomicDriveController(xController, yController, thetaController);


        Translation2d frontLeftMeters = new Translation2d(0, 0);
        Translation2d frontRightMeters = new Translation2d(0, 0);
        Translation2d backLeftMeters = new Translation2d(0, 0);
        Translation2d backRightMeters = new Translation2d(0, 0);

        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftMeters, frontRightMeters, backLeftMeters, backRightMeters);

        Rotation2d angle = new Rotation2d(0); // Degrees

        MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(WHEEL_DISTANCE_FROM_CENTER, WHEEL_DISTANCE_FROM_CENTER, WHEEL_DISTANCE_FROM_CENTER, WHEEL_DISTANCE_FROM_CENTER); //0?

        this.poseEstimator = new MecanumDrivePoseEstimator(kinematics, angle, wheelPositions, this.initialPose);
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

    public void setPower(double drive, double strafe, double rotate, double power, boolean debugMode)
    {
        double maxPower;

        double desiredLinearVelocity = drive * MAX_LINEAR_VELOCITY;

        Rotation2d desiredHeading = new Rotation2d(rotate * MAX_ANGULAR_VELOCITY);

        Pose2d desiredPose = new Pose2d(drive, strafe, desiredHeading);
        Pose2d currentPose = poseEstimator.getEstimatedPosition();

        ChassisSpeeds chassisSpeeds = driveController.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredHeading);

        double flPower = chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
        double frPower = chassisSpeeds.vxMetersPerSecond - chassisSpeeds.vyMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;
        double blPower = chassisSpeeds.vxMetersPerSecond - chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
        double brPower = chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;

        Rotation2d currentAngle = new Rotation2d(gyro.getRawAngle());
        // According to the Documentation the update() method takes as a parameter an object of the MechanumDriveWheelSpeed
        // However the constructor takes one of MecanumDriveWheelPositions, so...
        MecanumDriveWheelPositions currentWheelSpeed = new MecanumDriveWheelPositions(flPower,frPower,blPower,brPower);
        currentPose = poseEstimator.update(currentAngle, currentWheelSpeed);

        maxPower = Math.max(Math.abs(flPower), Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

//        Normalizing Values
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

    public void resetGyro(){
        gyro.reset();
//        targetAngle = closestTarget(gyro.getRawAngle());
    }





}
