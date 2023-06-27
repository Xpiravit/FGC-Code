package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.RingBuffer;

public class Gyro {

    private IMU imu;
    private double startingAngle;
    private RingBuffer<Double> timeRing = new RingBuffer<>(4, 0.0);
    private RingBuffer<Double> angleRing = new RingBuffer<>(4, 0.0);

    public Gyro(IMU imu, double startingAngle) {
        this.imu = imu;
        this.startingAngle = startingAngle;
    }

//    public void setImu(IMU imu) {
//        this.imu = imu;
//    }

    public void setStartingAngle(double startingAngle) {
        this.startingAngle = startingAngle;
    }

    public void reset() {
        this.startingAngle = imu.getAngle();
    }

    /**
     @return {double} returns the raw angle based on the starting angle
     */
    public double getRawAngle() {
        return imu.getAngle() - startingAngle;
    }

    /**
     @return {double} returns the angle offset in relation to the starting angle
     */
    public double getModAngle() {
        return (imu.getAngle() - startingAngle) % 360;
    }

    public double rateOfChange()
    {
        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaSeconds = deltaMili / 1000.0;

        double currentAngle = getRawAngle();
        double deltaAngle = currentAngle - angleRing.getValue(currentAngle);

        return deltaAngle / deltaSeconds;
    }

}