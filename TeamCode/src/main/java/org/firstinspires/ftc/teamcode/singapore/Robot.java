package org.firstinspires.ftc.teamcode.singapore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.singapore.hardware.Bucket;
import org.firstinspires.ftc.teamcode.singapore.hardware.SixWheelDrive;

public class Robot extends SixWheelDrive
{
    public Bucket bucket;

    private DcMotor intakeMotor;
    private boolean intakeState = false;

    public Robot (HardwareMap hardwareMap, Telemetry telemetry)
    {
        super(hardwareMap, telemetry);

        bucket = new Bucket(hardwareMap, telemetry);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    private void startIntake(){
        intakeMotor.setPower(1);
        intakeState = true;
    }

    private void stopIntake(){
        intakeMotor.setPower(0);
        intakeState = false;
    }

    public void toggleIntake()
    {
        if (intakeState) stopIntake();
        else startIntake();
    }

    public void updateTelemetry()
    {
        telemetry.addData("Heading", "Heading = %.2f Degrees", super.getHeading());
        telemetry.addData("Position", "X (meters) %.2f Y (meters) %.2f", super.getRobotX(), super.getRobotY());
        bucket.updateTelemetry();
        if (intakeState) telemetry.addData("Intake State: ", "Running");
        else telemetry.addData("Intake State: ", "Stopped");
    }

}
