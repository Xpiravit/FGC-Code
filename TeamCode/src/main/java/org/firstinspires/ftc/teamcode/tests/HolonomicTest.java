package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.PlainChassis;
import org.firstinspires.ftc.utilities.Controller;
import org.firstinspires.ftc.utilities.IMUWraper;

@TeleOp(name = "Holonomic Test", group = "Test")
public class HolonomicTest extends OpMode
{
    private Controller driver;
    private IMUWraper gyro;
    private PlainChassis robot;
    private final double perspectiveError = 0;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        driver = new Controller(gamepad1);
        robot = new PlainChassis(hardwareMap, telemetry);

        robot.stopAndResetEncoders();
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.resetGyro();
    }

    @Override
    public void loop() {
        if(driver.R3StateUpdate()) robot.resetGyro();

        //Controller configuration
        Controller.Thumbstick driverRightStick = driver.getRightThumbstick();
        Controller.Thumbstick driverLeftStick = driver.getLeftThumbstick();
        driverLeftStick.setShift(gyro.getCurrentHeading() - perspectiveError);
        driverRightStick.setShift(0);

        //Driver controls
        robot.setPower(driverLeftStick.getShiftedX(), driverLeftStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX());

//      robot.setCardinalAngle(driver.dpadUpStateUpdate(), driver.dpadRightStateUpdate(), driver.dpadDownStateUpdate(), driver.dpadLeftStateUpdate());

        //Telemetry
        telemetry.addData("Gyro", "Heading = %.2f Degrees", gyro.getCurrentHeading());
        telemetry.addData("Position", "X (meters) %.2f Y (meters) %.2f", robot.getRobotX(), robot.getRobotY());

    }
}
