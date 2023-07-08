package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RoadRunnerChassis;
import org.firstinspires.ftc.utilities.Controller;

@TeleOp(name = "RoadRunner Holonomic Test", group = "Test")
public class RoadRunnerHolonomicTest extends OpMode
{
    private Controller driver;

    private RoadRunnerChassis robot;

    private final double perspectiveError = 0;

    @Override
    public void init() {
        this.telemetry.addData("Status", "Initialized");
        this.driver = new Controller(gamepad1);

        this.robot = new RoadRunnerChassis(hardwareMap, telemetry);
        this.robot.stopAndResetEncoders();
        this.robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        driverLeftStick.setShift(robot.getHeading() - perspectiveError);
        driverRightStick.setShift(0);

        //Driver controls
        robot.setPower(driverLeftStick.getShiftedX(), driverLeftStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(), false);

//      robot.setCardinalAngle(driver.dpadUpStateUpdate(), driver.dpadRightStateUpdate(), driver.dpadDownStateUpdate(), driver.dpadLeftStateUpdate());

        //Telemetry
        telemetry.addData("Gyro", "Heading = %.2f Degrees", robot.getHeading());
        telemetry.addData("Position", "X (meters) %.2f Y (meters) %.2f", robot.getCurrentX(), robot.getCurrentY());

    }
}

