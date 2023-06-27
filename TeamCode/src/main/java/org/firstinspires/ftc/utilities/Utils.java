package org.firstinspires.ftc.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class Utils {

    private static HardwareMap hardwareMap;

    public static HardwareMap getHardwareMap(){
        return hardwareMap;
    }

    public static void setHardwareMap(HardwareMap hardwareMap){
        Utils.hardwareMap = hardwareMap;
    }

    public static Vector2D convertToGlobalVector(Vector2D relativeVector){

        double phi = (3 * Math.PI / 4) - Math.atan2(relativeVector.getY(), relativeVector.getX());
        double d = Math.sqrt(Math.pow(relativeVector.getX(), 2) + Math.pow(relativeVector.getY(), 2));

        return new Vector2D(Math.cos(phi) * d, Math.sin(phi) * d);
    }

}