package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.utilities.Controller;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;
import org.firstinspires.ftc.teamcode.hardware.HolonomicChassis2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


@TeleOp(name = "Holonomic Test", group = "Test")
public class HolonomicTest extends OpMode {
    private Controller driver;
    private Gyro gyro;
    private HolonomicChassis2 robot;

    private double perspectiveError = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        driver = new Controller(gamepad1);

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");

        Utils.setHardwareMap(hardwareMap);
        IMU imu = new IMU("imu");

        gyro = new Gyro(imu, 0);

        Rotation2d startingAngle = new Rotation2d(gyro.getRawAngle());
        Pose2d startingPose = new Pose2d(0, -7.0,startingAngle);

        robot = new HolonomicChassis2(frontLeft, frontRight, backRight, backLeft, gyro, telemetry, startingPose);
        robot.stopAndResetEncoders();
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
        if(driver.R3StateUpdate()){
            robot.resetGyro();
        }

        Controller.Thumbstick driverRightStick = driver.getRightThumbstick();
        Controller.Thumbstick driverLeftStick = driver.getLeftThumbstick();
        driverLeftStick.setShift(gyro.getModAngle() - perspectiveError);
        driverRightStick.setShift(0);

        //driver controls
        robot.setPower(driverLeftStick.getShiftedX(), driverLeftStick.getInvertedShiftedY(),
                driverRightStick.getInvertedShiftedX(), 1.0,true);
//        robot.setCardinalAngle(driver.dpadUpStateUpdate(), driver.dpadRightStateUpdate(), driver.dpadDownStateUpdate(), driver.dpadLeftStateUpdate());

        telemetry.addData("Gyro", "Raw Angle %.2f Mod Angle %.2f", gyro.getRawAngle(), gyro.getModAngle());
//        robot.setPowerAuto(driverLeftStick.getShiftedX(), driverLeftStick.getInvertedShiftedY(), 1);
        telemetry.addData("Position", "X (meters) %.2f Y (meters) %.2f", robot.currentPose.getX(), robot.currentPose.getY());
//        robot.setTargetAngle(driverRightStick);

    }
}
