package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import edu.wpi.first.math.controller.PIDController;

/**
 * A Java Class meant to represents a robot's chassis for a square 45 degree omniwheel drivetrain.
 * This class is essentially a wrapper for the SampleMecanumDrive class of the RoadRunner library meant to make it more accessible.
 * Thus any errors in the localization or the management of the robots trajectory derive from use of mistuned RoadRunner algorithms and will not be resolved by making changes to the code below.
 *
 * <p><b>REMINDER</b>: Since the RoadRunner library is being used, make sure to adjust the DriveConstants file (org/firstinspires/ftc/teamcode/drive/DriveConstants.java)
 *
 * @author Xpiravit
 */

public class RoadRunnerChassis
{
    private final PIDController headingController;
    private final SampleMecanumDrive drive;
    Telemetry telemetry;

    //    Constructor
    public RoadRunnerChassis(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.headingController = new PIDController(0.015, 0.000, 0.0004);

        this.drive = new SampleMecanumDrive(hardwareMap);

        this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.drive.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new PIDFCoefficients(0.015, 0.000, 0.0004, 0));

        this.telemetry = telemetry;
    }

    public void stopAndResetEncoders()
    {
        this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
    {
        drive.setZeroPowerBehavior(behavior);
    }

    public void resetGyro()
    {
        this.drive.setExternalHeading(0);
    }

    public double getHeading()
    {
        return this.drive.getExternalHeading();
    }

    public double getCurrentX()
    {
        return this.drive.getPoseEstimate().getX();
    }

    public double getCurrentY()
    {
        return this.drive.getPoseEstimate().getY();
    }

    public void setPower(double drive, double strafe, double rotate, boolean debugMode)
    {
        this.drive.setWeightedDrivePower(
              new Pose2d(-drive, -strafe, -headingController.calculate(rotate))
        );

        this.drive.update();

        if (debugMode)
        {
            Pose2d poseEstimate = this.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


}