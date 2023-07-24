package org.firstinspires.ftc.teamcode.singapore;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.singapore.utils.Controller23;

@Disabled
@TeleOp(name = "FGC Tele-op", group = "Final")
public class FGCTeleop extends OpMode
{
    private Controller23 driver;
    private Robot robot;
    private DigitalChannel liftTop;
    private DigitalChannel liftBottom;
    private static final double perspectiveError = 0;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        driver = new Controller23(gamepad1);
        robot = new Robot(hardwareMap, telemetry);

        liftTop = hardwareMap.get(DigitalChannel.class, "lift_top");
        liftTop.setMode(DigitalChannel.Mode.INPUT);

        liftBottom = hardwareMap.get(DigitalChannel.class, "lift_bottom");
        liftBottom.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop()
    {
        if(driver.R3()) robot.resetGyro();

        //Controller configuration
        driver.updateStickValues();
        driver.left_stick.setShift(robot.getHeading() - perspectiveError);
        driver.right_stick.setShift(0);

        //Driver controls
        robot.setPower(true, driver.left_stick.shiftedY, driver.left_stick.shiftedY, true);

//      robot.setCardinalAngle(driver.dpadUpStateUpdate(), driver.dpadRightStateUpdate(), driver.dpadDownStateUpdate(), driver.dpadLeftStateUpdate());

        if (driver.toggleState(driver.B())) robot.toggleIntake();

        if (driver.dpadUp()) robot.bucket.startLift();
        if (liftTop.getState())
        {
            robot.bucket.stopLift();
            robot.bucket.liftPosition = 1;
        }
        if (liftBottom.getState())
        {
            robot.bucket.stopLift();
            robot.bucket.liftPosition = 0;
        }
        if (driver.toggleState(driver.L2())) robot.bucket.empty();
        if (driver.dpadDown()) robot.bucket.resetLift();

        //Telemetry
        telemetry.addData("Heading", "Heading = %.2f Degrees", robot.getHeading());
        telemetry.addData("Position", "X (meters) %.2f Y (meters) %.2f", robot.getRobotX(), robot.getRobotY());

    }
}
