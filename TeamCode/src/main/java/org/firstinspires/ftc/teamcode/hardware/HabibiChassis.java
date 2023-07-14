package org.firstinspires.ftc.teamcode.hardware;

import static org.apache.commons.math3.util.FastMath.atan2;
import static org.apache.commons.math3.util.FastMath.hypot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.utilities.IMUWraper;

import edu.spa.ftclib.internal.drivetrain.OmniwheelDrivetrain;


/**
 * A Java Class meant to represents a robot's chassis for a square 45 degree omniwheel drivetrain.
 * This class is utilizing tools found in the Habibi Library. Thus, please familiarize yourself with it before making changes to code below.
 * It is a child of the {@link OmniwheelDrivetrain}
 * class and acts essentially as a wrapper for it.
 * Any errors in the localization, PID calculations or the management of the robots trajectory derive from use of mistuned algorithm parameters, all parameters should be empirically tested and changed accordingly.
 *
 * @author Xpiravit
 */

public class HabibiChassis extends OmniwheelDrivetrain
{
    private final IMUWraper gyro;
    private final Telemetry telemetry;
//    private PIDController turnController;

    public HabibiChassis(Telemetry telemetry, @NonNull HardwareMap hardwareMap)
    {
        super(new DcMotor[] {
                hardwareMap.get(DcMotor.class, "front_left_motor"),
                hardwareMap.get(DcMotor.class, "front_right_motor"),
                hardwareMap.get(DcMotor.class, "back_left_motor"),
                hardwareMap.get(DcMotor.class, "back_right_motor")
        });

        this.telemetry = telemetry;
        this.gyro = new IMUWraper(hardwareMap, "imu");
        this.gyro.reset();

//        this.turnController = new PIDController(0.015, 0.000, 0.004);

        initializeMotors();
    }

    private void initializeMotors()
    {
        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor i : motors) i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double drive, double strafe, double rotate, boolean debugMode) {
//        turnController.input(rotate);
//        setRotation(turnController.output());

        setRotation(rotate);
        setVelocity(hypot(drive, strafe));
        setCourse(atan2(drive, strafe));

        if (debugMode)
        {
            telemetry.addData("Stick_X", drive);
            telemetry.addData("Stick_Y", strafe);
            telemetry.addData("Velocity", hypot(drive, strafe));
        }
    }

    public void setPower(double drive, double strafe, double rotate){
        setPower(drive, strafe, rotate, false);
    }

    public void setCardinalAngle (boolean northInput, boolean eastInput, boolean southInput, boolean westInput) {
        if (northInput) setCourse(0);
        else if (eastInput) setCourse(-Math.PI / 2);
        else if (southInput) setCourse(Math.PI);
        else if (westInput) setCourse(Math.PI / 2);
    }

    public void resetGyro() {
        gyro.reset();
    }

}
