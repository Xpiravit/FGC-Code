package org.firstinspires.ftc.teamcode.singapore.utils;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.Gamepad;
import edu.spa.ftclib.internal.state.ToggleBoolean;

public class Controller23 extends Gamepad
{
    private final Gamepad controller;
    public Thumbstick left_stick;
    public Thumbstick right_stick;

    public Controller23(@NonNull Gamepad controller)
    {
        this.controller = controller;
        left_stick = new Thumbstick(controller.left_stick_x, controller.left_stick_y);
        right_stick = new Thumbstick(controller.right_stick_x, controller.right_stick_y);
    }

    public boolean toggleState(boolean currentState)
    {
        ToggleBoolean button = new ToggleBoolean();

        button.input(currentState);
        return button.output();
    }

    public boolean X(){
        return controller.x;
    }
    public boolean Y(){
        return controller.y;
    }
    public boolean A(){
        return controller.a;
    }
    public boolean B(){
        return controller.b;
    }
    public boolean R1(){
        return controller.right_bumper;
    }
    public boolean L1(){
        return controller.left_bumper;
    }
    public boolean R2 (){
        return controller.right_trigger != 0;
    }
    public boolean L2 (){
        return controller.left_trigger != 0;
    }
    public boolean R3(){
        return controller.right_stick_button;
    }
    public boolean L3(){
        return controller.left_stick_button;
    }
    public boolean dpadUp(){
        return controller.dpad_up;
    }
    public boolean dpadDown(){
        return controller.dpad_down;
    }
    public boolean dpadLeft(){
        return controller.dpad_left;
    }
    public boolean dpadRight(){
        return controller.dpad_right;
    }



    public void updateStickValues(){
        right_stick.rawX = controller.right_stick_x;
        left_stick.rawX = controller.left_stick_x;
    }

    public static class Thumbstick
    {
        public double rawX;
        public double rawY;

        public double shiftedX;
        public double shiftedY;

        public Thumbstick(Float x, Float y)
        {
            this.rawX = x;
            this.rawY = y;
        }

        public double getX() {
            return rawX;
        }
        public double getY() {
            return rawY;
        }

        public void setShift(double shiftAngle)
        {
            this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
            this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }
    }
}
