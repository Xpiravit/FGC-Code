package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.floor;
import static java.lang.Math.floorMod;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.utilities.Controller;
import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBuffer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Hardware Class that represents the robot's chassis for a 45 degree omniwheel drivetrain. I'm pretty sure it works with Mecanum wheels too.
 * Inspired from the FTC team Wolfpack Machina
 * @author gabkion
 */
public class HolonomicChassis {
    private static final double WHEEL_RADIUS = 0.09 / 2.0; //meters
    private static final double WHEEL_DISTANCE_FROM_CENTER = 0.29; //meters
    private static final int ENCODER_TICKS_PER_REVOLUTION = 336;
    public Vector2D positionVector = new Vector2D(0,0);

    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;

    private final PID telePID = new PID(.015, 0.0000, .0004);
    private final PID autoPID = new PID(.05, 0.0000, .004);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1,1);
    private final PIDController xController = new PIDController(.015, 0.0000, .0004);
    private final PIDController yController = new PIDController(.05, 0.0000, .004);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(.05, 0.000, .004, constraints);

    private final HolonomicDriveController holonomicPID = new HolonomicDriveController(xController, yController, thetaController);
    private final double turnTolerance = 0.1;

    private double turnTime = 0;
    private double turn, targetAngle, bigTurn, previousAngle, deltaAngle;
    private boolean currentTurningState;
    private boolean pressTrun;
    private double turnWeight = 0;
    private boolean precisionMode = false;

    private static final double TELE_ACCEL = .002;

    private final RingBuffer<Double> positionRingFL = new RingBuffer<>(20, 0.0);
    private final RingBuffer<Double> positionRingFR = new RingBuffer<>(20, 0.0);
    private final RingBuffer<Double> positionRingBL = new RingBuffer<>(20, 0.0);
    private final RingBuffer<Double> positionRingBR = new RingBuffer<>(20, 0.0);
    private final RingBuffer<Long> timeRing = new RingBuffer<>(20, (long)0);

    private double previousHeading;

//    public Pose2d position = new Pose2d(0,0,0);
    public double x,y,heading;

    Gyro gyro;

    Telemetry telemetry;

//    Constructor
    public HolonomicChassis(DcMotor frontLeft, DcMotor frontRight, DcMotor backRight, DcMotor backLeft, Gyro gyro, Telemetry telemetry)
    {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.gyro = gyro;

        this.telemetry = telemetry;
    }

    public void stopAndResetEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isTUrning()
    {
        boolean pastTurningState = currentTurningState;
        currentTurningState = turn != 0 ;
//      pressTrun = currentTurningState;
        return (currentTurningState && !pastTurningState);
    }

    public void setPower(double drive, double strafe, double rotate, double power, boolean debugMode)
    {
        double maxPower;

        double flPower = (drive + strafe - rotate) * power;
        double frPower = (-drive + strafe + rotate) * power;
        double blPower = (-drive + strafe - rotate) * power;
        double brPower = (drive + strafe + rotate) * power;

        maxPower = Math.max(Math.abs(flPower), Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

//        Normalizing
        if (maxPower > 1.0)
        {
            flPower = flPower / maxPower;
            frPower = frPower / maxPower;
            blPower = blPower / maxPower;
            brPower = brPower / maxPower;
        }

        this.frontLeft.setPower(flPower);
        this.frontRight.setPower(frPower);
        this.backLeft.setPower(blPower);
        this.backRight.setPower(brPower);

        if (debugMode)
        {
            telemetry.addData("Stick_X", drive);
            telemetry.addData("Stick_Y", strafe);
            telemetry.addData("Magnitude", Math.sqrt(Math.pow(drive, 2) + Math.pow(strafe, 2)));
            telemetry.addData("Front Left", flPower);
            telemetry.addData("Back Left", blPower);
            telemetry.addData("Back Right", brPower);
            telemetry.addData("Front Right", frPower);
        }
    }

//    public void setPower(double x, double y, double turn, double power)
//    {
//        setPower(x, y, turn, power, false);
//    }
//
//
//    public void setPower(double x, double y, double turn) {setPower(x, y, turn, 1, false);
//    }

    public void setPowerTele(double drive, double strafe, double rotate, double power, boolean debugMode)
    {
        this.turn = rotate;

        double currentTime = System.currentTimeMillis();
        double deltaTime = currentTime - turnTime;

        double inputTurn;

        if (isTUrning()) previousAngle = gyro.getRawAngle();

        if (rotate != 0)
        {
            /* The driver is turning the robot */
            inputTurn = rotate;
            bigTurn = rotate;
            turnWeight = rotate * 2/3;
            turnTime = currentTime;
            targetAngle = gyro.getRawAngle();
        }
        else
        {
/*
          If the robot turned more than the turn tolerance (minimum turn),
          the bigTurn value is updated until it is less than 0 and the target angle is constantly updated for the PID.
          This is done to compensate for the robots initial inertia and to ease out the PID effect.
          Not sure if it works tho ¯\_(ツ)_/¯
*/
            if (bigTurn > turnTolerance)
            { // one direction
                /*if enough time has passed from the turn, bigTurn becomes 0, PID kicks in*/
                bigTurn = Math.max(0, -TELE_ACCEL * deltaTime + turnWeight);
                /* eases out the turn */
                targetAngle = gyro.getRawAngle();

            }
            else if (bigTurn < -turnTolerance)
            { //other direction
                /*if enough time has passed from the turn, bigTurn becomes 0, PID kicks in*/
                bigTurn = Math.min(0, TELE_ACCEL * deltaTime + turnWeight);
                /* eases out the turn */
                targetAngle = gyro.getRawAngle();
            }
            else bigTurn = 0;

            /* A PID controller takes care of the input turn. */
            inputTurn = telePID.update((targetAngle - gyro.getRawAngle()) / ((getRPM() / 600) + .5));
        }

        if (debugMode)
        {
            telemetry.addData("Turn", rotate);
            telemetry.addData("Big Turn: ", bigTurn);
            telemetry.addData("Turn Y Int: ", turnWeight);
            telemetry.addData("Target Angle: ", targetAngle);
            telemetry.addData("Current Time: ", currentTime);
            telemetry.addData("Delta Time: ", deltaTime);
            telemetry.addData("Input Turn: ", inputTurn);
        }

        if (precisionMode)
        {
            inputTurn = rotate; //turns off the PID
            power = power/5;
        }
        setPower(drive, strafe, inputTurn, power, false);
    }

//    public void setPowerTele(double x, double y, double turn){
//        setPowerTele(x, y, turn, 1,false);
//    }

    public void setPowerAuto(double x, double y, double targetAngle, double power)
    {
        this.targetAngle = targetAngle;
        turn = autoPID.update(this.targetAngle - gyro.getRawAngle());
        setPower(x, y, turn, power, false);
    }

    public void setPowerAuto(double x, double y, double power){
        setPowerAuto(x, y, this.targetAngle, power);
    }

    /**
     * Finds the closest way to a target angle
     * @return {double} returns the closest target to the target angle
     */
    public double closestTarget(double targetAngle)
    {
        /* We calculate the angle offset between the target angle and the current angle, similarly to the Gyro class.
        The multiplication and division by 1e6 is done to avoid floating point errors and to ensure accuracy */
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + gyro.getRawAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;

        /*Here we calculate the alternate delta, in order to later compare it with the simple one and find the closest way*/
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);

        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? gyro.getRawAngle() - simpleTargetDelta : gyro.getRawAngle() - alternateTargetDelta;
    }

    /**
     * If the boolean is true, it sets the targetAngle for the PID system to the closest coterminal angle
     * to the input's respective angle (North 0, East 270, South 180, West 90) with priority being in that order
     */
    public void setCardinalAngle(boolean northInput, boolean eastInput, boolean southInput, boolean westInput){
        if (northInput) {
            targetAngle = closestTarget(0);
        }
        if (eastInput) {
            targetAngle = closestTarget(270);
        }
        if (southInput) {
            targetAngle = closestTarget(180);
        }
        if (westInput) {
            targetAngle = closestTarget(90);
        }
    }

//    public void setSinkAlignment(boolean align){
//        double tangent = y/x;
//        double vectorAngle = x >= 0? 90 + Math.toDegrees(Math.atan(tangent)) : 270 + Math.toDegrees(Math.atan(tangent));
//
//        if(align){
//            targetAngle = closestTarget(vectorAngle);
//        }
//    }

//    public void setSinkAlignment()
//    {
//        double tangent = y/x;
//        double vectorAngle = x >= 0? 90 + Math.toDegrees(Math.atan(tangent)) : 270 + Math.toDegrees(Math.atan(tangent));
//
//        targetAngle = closestTarget(vectorAngle);
//    }

    public void setPositionMeters(double x, double y, double angle){
        this.x = x;
        this.y = y;
    }

    /**
     * Sets the targetAngle for the PID system to the angle the input thumbstick is being pushed in if it is beyond a .1 deadzone
     * this may be needed in case of shooting
     */
    public void setTargetAngle(Controller.Thumbstick thumbstickAngleInput){
        if(abs(thumbstickAngleInput.getX()) > .1 || abs(thumbstickAngleInput.getY()) > .1){
            targetAngle = closestTarget(thumbstickAngleInput.getAngle());
        }
    }

    public void setPrecisionMode(boolean precisionToggle){
        this.precisionMode = precisionToggle;
            if(this.precisionMode){
                telemetry.addData("Precision Mode:", "ON");
            }else{
                telemetry.addData("Precision Mode:", "OFF");
            }
    }

    public void updatePositionVector()
    {
        double currentPositionBL = backLeft.getCurrentPosition();
        double currentPositionFL = frontLeft.getCurrentPosition();
        double currentPositionBR = backRight.getCurrentPosition();
        double currentPositionFR = frontRight.getCurrentPosition();


        int incrementalTicksBL = (int) (backLeft.getCurrentPosition() - positionRingBL.getValue(currentPositionBL));
        int incrementalTicksFL = (int) (frontLeft.getCurrentPosition() - positionRingFL.getValue(currentPositionFL));
        int incrementalTicksFR = (int) (frontRight.getCurrentPosition() - positionRingFR.getValue(currentPositionFR));
        int incrementalTicksBR = (int) (backRight.getCurrentPosition() - positionRingBR.getValue(currentPositionBR));

        double ticksPerMeter = ENCODER_TICKS_PER_REVOLUTION / WHEEL_RADIUS * 2;


//        double wbl = incrementalTicksBL * 2*PI*WHEEL_RADIUS / ENCODER_TICKS_PER_REVOLUTION;
//        double wfl = incrementalTicksFL * 2*PI*WHEEL_RADIUS / ENCODER_TICKS_PER_REVOLUTION;
//        double wbr = incrementalTicksBR * 2*PI*WHEEL_RADIUS / ENCODER_TICKS_PER_REVOLUTION;
//        double wfr = incrementalTicksFR * 2*PI*WHEEL_RADIUS / ENCODER_TICKS_PER_REVOLUTION;

        double wbl = incrementalTicksBL / ticksPerMeter;
        double wfl = incrementalTicksFL / ticksPerMeter;
        double wbr = incrementalTicksBR / ticksPerMeter;
        double wfr = incrementalTicksFR / ticksPerMeter;

        double deltaXr = -0.25 * sqrt(2) * (wbl - wfl + wfr - wbr);
        double deltaYr = 0.25 * sqrt(2) * (wbl + wfl + wfr + wbr);

        double averageHeading = previousHeading + 0.5 *
                MathUtils.normalizeAngle(Math.toRadians(gyro.getRawAngle()) - previousHeading, 0.0);

//        double averageHeading = MathUtils.normalizeAngle(Math.toRadians((gyro.getModAngle())), 0.0);

        System.out.println(averageHeading);

        double deltaXf = deltaXr  * cos(averageHeading) - deltaYr * sin(averageHeading);
        double deltaYf = deltaXr * sin(averageHeading) + deltaYr * cos(averageHeading);

        this.x += deltaXf;
        this.y += deltaYf;
        this.heading = gyro.getRawAngle();

        previousHeading = Math.toRadians(gyro.getModAngle());
    }


    /**
     * Uses Ring Buffers to calculate the average RPM of the chassis wheels.
     * @return {double} Returns the average value of the wheels' RPM
     */
    public double getRPM(){
        double retVal;

        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        double currentPositionFL = frontLeft.getCurrentPosition();
        double currentPositionFR = frontRight.getCurrentPosition();
        double currentPositionBL = backLeft.getCurrentPosition();
        double currentPositionBR = backRight.getCurrentPosition();

        double deltaRotationsFL = abs(currentPositionFL - positionRingFL.getValue(currentPositionFL)) / 537.6;
        double deltaRotationsFR = abs(currentPositionFR - positionRingFR.getValue(currentPositionFR)) / 537.6;
        double deltaRotationsBL = abs(currentPositionBL - positionRingBL.getValue(currentPositionBL)) / 537.6;
        double deltaRotationsBR = abs(currentPositionBR - positionRingBR.getValue(currentPositionBR)) / 537.6;

        retVal = ((deltaRotationsFL + deltaRotationsFR + deltaRotationsBL + deltaRotationsBR) / 4.0) / deltaMinutes;

        return retVal;
    }

    public int getTurnTicks(double angle){
        double wheelRotations = angle * (WHEEL_DISTANCE_FROM_CENTER / WHEEL_RADIUS) / 360;
        return (int) floor(wheelRotations * ENCODER_TICKS_PER_REVOLUTION);
    }

    /**
     * Resets the gyro and sets the target angle to the
     * gyro's value to ensure that the robot has the correct orientation
     * */
    public void resetGyro(){
        gyro.reset();
        targetAngle = closestTarget(gyro.getRawAngle());
    }

    public void updatePosition(){
        updatePositionVector();
        if (turn != 0){
//            calculate the turn

        }
    }
}
