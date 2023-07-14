package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HabibiChassis;
import org.firstinspires.ftc.utilities.Controller;
import org.firstinspires.ftc.utilities.IMUWraper;

@Disabled
@TeleOp(name = "Robot Tele-op", group = "Omni")

public class HabibiTest extends OpMode
{
    private HabibiChassis robot;
    private Controller driver;
    private IMUWraper gyro;


    private final double perspectiveError = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        driver = new Controller(gamepad1);
        robot = new HabibiChassis(telemetry, hardwareMap);
    }

    @Override
    public void loop() {
        if (driver.R3StateUpdate()) robot.resetGyro();

        //Controller configuration
        Controller.Thumbstick driverRightStick = driver.getRightThumbstick();
        Controller.Thumbstick driverLeftStick = driver.getLeftThumbstick();
        driverLeftStick.setShift(gyro.getCurrentHeading() - perspectiveError);
        driverRightStick.setShift(0);

        //Driver controls
        robot.setPower(driverLeftStick.getShiftedX(), driverLeftStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX());

        robot.setCardinalAngle(driver.dpadUpStateUpdate(), driver.dpadRightStateUpdate(), driver.dpadDownStateUpdate(), driver.dpadLeftStateUpdate());

        //Telemetry
        telemetry.addData("Gyro", "Heading = %.2f Degrees", robot.getCourse());
//        telemetry.addData("Position", "X (meters) %.2f Y (meters) %.2f", robot.getRobotX(), robot.getRobotY());

    }
}