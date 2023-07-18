package org.firstinspires.ftc.teamcode.drive.petroschallenge;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import edu.spa.ftclib.internal.drivetrain.TankDrivetrain;
import edu.spa.ftclib.internal.state.ToggleBoolean;

@Disabled
@TeleOp(name = "Tank Teleop", group = "Petros")
public class first extends OpMode
{
    private TankDrivetrain drivetrain;
    private ToggleBoolean driveMode;

    @Override
    public void init() {
        DcMotor left = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor right = hardwareMap.get(DcMotor.class, "motor2");
        right.setDirection(DcMotor.Direction.REVERSE);
        drivetrain = new TankDrivetrain(left, right);
        driveMode = new ToggleBoolean();
    }

    @Override
    public void loop() {
        driveMode.input(gamepad1.x);
        if (driveMode.output()) {
            drivetrain.setVelocity(absMax(-gamepad1.left_stick_y, -gamepad1.right_stick_y));
            drivetrain.setRotation(absMax(gamepad1.left_stick_x, gamepad1.right_stick_x));
        } else {
            drivetrain.left.setPower(-gamepad1.left_stick_y);
            drivetrain.right.setPower(-gamepad1.right_stick_y);
        }
    }

    private double absMax(double a, double b) { //Returns the argument whose absolute value is greater (similar to Math.max() but compares absolute values)
        return (Math.abs(a) > Math.abs(b)) ? a : b;
    }


}
