package org.firstinspires.ftc.teamcode.example.utils;

import com.qualcomm.robotcore.util.Range;

public class SCurveControllerAcceleration
{
    private double MAX_POWER = 1;
    private double ACCELERATION_FACTOR = 0.1;

    private double previousDrive = 0;
    private double previousTurn = 0;

    public SCurveControllerAcceleration(double maxPower, double accelerationFactor)
    {
        MAX_POWER = maxPower;
        ACCELERATION_FACTOR = accelerationFactor;
    }

    public double[] getPowers(double drive, double turn)
    {
        double[] powers = new double[] {0.0, 0.0};

        double acceleratedDrive = applySCurveAcceleration(drive);
        double acceleratedTurn = applySCurveAcceleration(turn);

        // Calculate the delta of the accelerated inputs
        double deltaDrive = acceleratedDrive - previousDrive;
        double deltaTurn = acceleratedTurn - previousTurn;

        // Calculate the new drive and turn values by adding the acceleration
        double newDrive = previousDrive + Math.min(Math.abs(deltaDrive), ACCELERATION_FACTOR) * Math.signum(deltaDrive);
        double newTurn = previousTurn + Math.min(Math.abs(deltaTurn), ACCELERATION_FACTOR) * Math.signum(deltaTurn);

        // Limit the drive and turn values to the range [-1, 1]
        newDrive = Range.clip(newDrive, -1.0, 1.0);
        newTurn = Range.clip(newTurn, -1.0, 1.0);

        // Apply a maximum power limit
        newDrive *= MAX_POWER;
        newTurn *= MAX_POWER;

        previousDrive = newDrive;
        previousTurn = newTurn;

        powers[0] = newDrive + newTurn;
        powers[1] = newDrive - newTurn;

        return powers;
    }

    private double applySCurveAcceleration(double input) {
        return 1 / (1 + Math.exp(-ACCELERATION_FACTOR * input));
    }

}
