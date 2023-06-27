package org.firstinspires.ftc.utilities;

public class PID {
//    private Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    private final double kp;
    private final double ki;
    private final double kd;

    private double result = 0;
    private double integralSum = 0;
    private double previousError = 0;
    long previousTime;


    public PID(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        previousTime = System.currentTimeMillis();
    }

    public Double update(double error)
    {
        integralSum += error;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - previousTime) / 1000.0;
        double rateOfChange = (error - previousError) / deltaTime;

        previousTime = currentTime;
        previousError = error;

        double proportional = error * kp;
        double integral = integralSum * ki;
        double derivative = rateOfChange * kd;

        this.result = proportional + integral + derivative;
        return result;
    }

    public Double getResult() {return this.result;}
}
