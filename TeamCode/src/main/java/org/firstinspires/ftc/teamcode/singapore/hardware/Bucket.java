package org.firstinspires.ftc.teamcode.singapore.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bucket
{
    private static Servo servo = null;
    private static DcMotor liftMotor = null;
    private static boolean servoPosition;
    private final Telemetry telemetry;
    public int liftPosition;


    public Bucket (@NonNull HardwareMap hardwareMap, Telemetry telemetry)
    {
        servo = hardwareMap.get(Servo.class, "bucket_servo");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        servoPosition = false;

        this.telemetry = telemetry;

        init();
    }

    public void init()
    {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo.setPosition(0);
    }

    public void startLift() {
        liftMotor.setPower(1);
        liftPosition = 2;
    }

    public void stopLift(){
        liftMotor.setPower(0);
    }

    private void empty(){
        servo.setPosition(1);
        servoPosition = false;
    }

    private void reset(){
        servo.setPosition(0);
        servoPosition = true;
    }

    public void toggle(){
        if (servoPosition) empty();
        else reset();
    }

    public void resetLift(){
        liftMotor.setPower(-1);
        liftPosition = 3;
    }

    public void updateTelemetry()
    {
        if (servoPosition) telemetry.addData("Bucket Servo State: ", "Armed");
        else telemetry.addData("Bucket Servo State: ", "Released");
        switch (liftPosition)
        {
            case 0:
                telemetry.addData("Lift State: ", "Retracted");
                break;
            case 1:
                telemetry.addData("Lift State: ", "Extended");
                break;
            case 2:
                telemetry.addData("Lift State", "Extending...");
                break;
            case 3:
                telemetry.addData("Lift State", "Retracting...");
                break;
        }
    }

}
