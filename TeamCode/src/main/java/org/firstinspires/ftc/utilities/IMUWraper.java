package org.firstinspires.ftc.utilities;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * A Java Class/Wrapper meant to make the use of the REV Hub's IMU easier.
 *
 * @author Xpiravit
 */

public class IMUWraper
{
    private final BNO055IMU imu;

    private Orientation lastOrientation;
    private double initialHeading;
    private double currentOrientation;

    public IMUWraper(@NonNull HardwareMap hardwareMap, String imuName, boolean Degrees)
    {
        this.imu = hardwareMap.get(BNO055IMU.class, imuName);
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        if (Degrees) imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        else imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(imuParams);

        reset();
    }
    public IMUWraper(@NonNull HardwareMap hardwareMap, String imuName)
    {
        this.imu = hardwareMap.get(BNO055IMU.class, imuName);
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;
        this.imu.initialize(imuParams);

        reset();
    }

    public void reset()
    {
        Orientation angles = imu.getAngularOrientation();
        initialHeading = angles.firstAngle;
    }

    public double getCurrentHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;
        double headingOffset = currentHeading - initialHeading;

        // Normalize the heading to a range of -180 to 180 degrees
        headingOffset = normalizeDegrees(headingOffset);

        return headingOffset;
    }

    public double getCurrentHeading(AxesReference reference, AxesOrder order, AngleUnit units)
    {
        Orientation angles = imu.getAngularOrientation(reference, order, units);
        double currentHeading = angles.firstAngle;
        double headingOffset = currentHeading - initialHeading;

        // Normalize the heading to a range of -180 to 180 degrees
        headingOffset = normalizeDegrees(headingOffset);

        return headingOffset;
    }

    private double normalizeDegrees(double degrees)
    {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;

        return degrees;
    }
}
