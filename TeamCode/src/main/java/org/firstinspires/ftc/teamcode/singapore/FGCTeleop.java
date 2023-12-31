package org.firstinspires.ftc.teamcode.singapore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.singapore.utils.Controller23;

//@Disabled
@TeleOp(name = "FGC Teleop", group = "Final")
public class FGCTeleop extends OpMode
{
    private Controller23 driver;
    private Robot robot;
    private static final double perspectiveError = 0;

    @Override
    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);

        driver = new Controller23(gamepad1);
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop()
    {
        if(driver.R3()) robot.resetGyro();

        //Controller configuration
        driver.update();
        driver.left_stick.setShift(robot.getHeading() - perspectiveError);
        driver.right_stick.setShift(0);

        //Driver controls
        robot.setPower(true, driver.left_stick.shiftedY, driver.left_stick.shiftedX, true);

        robot.turnToHorizon(driver.dpad);

        if (driver.toggleState(driver.B())) robot.toggleIntake();

        if (driver.dpadUp()) robot.bucket.startLift();
        if (robot.liftTopEnd.getState())
        {
            robot.bucket.stopLift();
            robot.bucket.liftPosition = 1;
        }
        if (driver.toggleState(driver.L2())) robot.bucket.toggle();
        if (driver.dpadDown()) robot.bucket.resetLift();
        if (robot.liftBottomEnd.getState())
        {
            robot.bucket.stopLift();
            robot.bucket.liftPosition = 0;
        }
        while (driver.X()) robot.brake();

        //Telemetry
        robot.updateTelemetry();
    }
}
