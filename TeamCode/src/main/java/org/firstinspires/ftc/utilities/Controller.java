package org.firstinspires.ftc.utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller
{
    private final Gamepad gamepad;
    private boolean squareCurrentState;
    private boolean triangleCurrentState;
    private boolean circleCurrentState;
    private boolean crossCurrentState;
    private boolean dpadUpCurrentState;
    private boolean dpadLeftCurrentState;
    private boolean dpadDownCurrentState;
    private boolean dpadRightCurrentState;
    private boolean L3CurrentState;
    private boolean R3CurrentState;
    private boolean L1CurrentState;
    private boolean R1CurrentState;
    private boolean toggleSq = false;
    private boolean toggleTr = false;
    private boolean toggleCr = false;
    private boolean toggleCi = false;
    private boolean toggleUp = false;
    private boolean toggleDown = false;
    private boolean toggleLeft = false;
    private boolean toggleRight = false;
    private boolean toggleLS = false;
    private boolean toggleRS = false;
    private boolean toggleLB = false;
    private boolean toggleRB = false;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public Thumbstick getRightThumbstick() {
        return new Thumbstick(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public Thumbstick getLeftThumbstick() {
        return new Thumbstick(gamepad.left_stick_x, gamepad.left_stick_y);
    }

    //cross
    public boolean cross() {
        return gamepad.a;
    }

    public boolean crossStateUpdate()
    {
        boolean crossPastState = crossCurrentState;
        crossCurrentState = gamepad.a;
//      pressCr = crossCurrentState;
        return (crossCurrentState && !crossPastState);
    }

    public boolean crossState() {
        return crossCurrentState;
    }

    public boolean isCrossToggled()
    {
        boolean crossPastState = crossCurrentState;
        crossCurrentState = gamepad.a;
//        if (crossCurrentState && !crossPastState) {
//            toggleCr = !toggleCr;
//        }
        return crossCurrentState && !crossPastState ? (toggleCr = !toggleCr) : toggleCr;
    }


    //circle
    public boolean circle() {
        return gamepad.b;
    }

    public boolean circleStateUpdate()
    {
        boolean circlePastState = circleCurrentState;
        circleCurrentState = gamepad.b;
//      pressCi = circleCurrentState

        return (circleCurrentState && !circlePastState);
    }

    public boolean circleState() { return circleCurrentState; }

    public boolean isCircleToggled()
    {
        boolean circlePastState = circleCurrentState;
        circleCurrentState = gamepad.b;
//        if ((circleCurrentState = gamepad.b) && !circlePastState) {
//            toggleCi = !toggleCi;
//        }
        return circleCurrentState && !circlePastState ? (toggleCi = !toggleCi) : toggleCi;
    }


    //square
    public boolean square() {
        return gamepad.x;
    }

    public boolean squarePressUpdate()
    {
        boolean squarePastState = squareCurrentState;
        squareCurrentState = gamepad.x;
//      pressSq = gamepad.x;

        return (squareCurrentState && !squarePastState);
    }

    public boolean squarePress() {
        return squareCurrentState;
    }

    public boolean isSquareToggled()
    {
        boolean squarePastState = squareCurrentState;
        squareCurrentState = gamepad.x;
//        if ((squareCurrentState = gamepad.x) && !squarePastState) {
//            toggleSq = !toggleSq;
//        }
        return squareCurrentState && !squarePastState ? (toggleSq = !toggleSq) : toggleSq;
    }


    //triangle
    public boolean triangle() {
        return gamepad.y;
    }

    public boolean triangleStateUpdate()
    {
        boolean trianglePastState = triangleCurrentState;
        triangleCurrentState = gamepad.y;
//      pressTr = triangleCurrentState;

        return (triangleCurrentState && !trianglePastState);
    }

    public boolean trianglePress() {
        return triangleCurrentState;
    }

    public boolean triangleToggle()
    {
        boolean trianglePastState = triangleCurrentState;
        triangleCurrentState = gamepad.y;
//        if ((triangleCurrentState = gamepad.y) && !trianglePastState) {
//            toggleTr = !toggleTr;
//        }
        return triangleCurrentState && !trianglePastState ? (toggleTr = !toggleTr) : toggleTr;
    }


    //dpad up
    public boolean dpadUp() {
        return gamepad.dpad_up;
    }

    public boolean dpadUpStateUpdate() {
        boolean dpadUpPastState = dpadUpCurrentState;
        dpadUpCurrentState = gamepad.dpad_up;
//      pressUp = dpadUpCurrentState;

        return (dpadUpCurrentState && !dpadUpPastState);
    }

    public boolean dpadUpPress() {
        return dpadUpCurrentState;
    }

    public boolean isDpadUpToggled()
    {
        boolean dpadUpPastState = dpadUpCurrentState;
        dpadUpCurrentState = gamepad.dpad_up;
//        if ((dpadUpCurrentState = gamepad.dpad_up) && !wasUp) {
//            toggleUp = !toggleUp;
//        }
        return dpadUpCurrentState && !dpadUpPastState ? (toggleUp = !toggleUp) : toggleUp;
    }


    //dpad down
    public boolean dpadDown() {
        return gamepad.dpad_down;
    }

    public boolean dpadDownStateUpdate()
    {
        boolean dpadDownPastState = dpadDownCurrentState;
        dpadDownCurrentState = gamepad.dpad_down;
//      pressDown = dpadDownCurrentState;

        return (dpadDownCurrentState && !dpadDownPastState);
    }

    public boolean dpadDownPress() { return dpadDownCurrentState; }

    public boolean isDpadDownToggled()
    {
        boolean dpadDownPastState = dpadDownCurrentState;
        dpadDownCurrentState = gamepad.dpad_down;

        return dpadDownCurrentState && !dpadDownPastState ? (toggleDown = !toggleDown) : toggleDown;
    }


    //dpad left
    public boolean dpadLeft() {
        return gamepad.dpad_left;
    }

    public boolean dpadLeftStateUpdate()
    {
        boolean dpadLeftPastState = dpadLeftCurrentState;
        dpadLeftCurrentState = gamepad.dpad_down;
//      pressLeft = dpadLeftCurrentState;

        return (dpadLeftCurrentState && !dpadLeftPastState);
    }

    public boolean dpadLeftPress() {
        return dpadLeftCurrentState;
    }

    public boolean isDpadLeftToggled()
    {
        boolean dpadLeftPastState = dpadLeftCurrentState;
        dpadLeftCurrentState = gamepad.dpad_left;

        return dpadLeftCurrentState && !dpadLeftPastState ? (toggleLeft = !toggleLeft) : toggleLeft;
    }


    //dpad right
    public boolean dpadRight() {
        return gamepad.dpad_right;
    }

    public boolean dpadRightStateUpdate()
    {
        boolean dpadRightPastState = dpadRightCurrentState;
        dpadRightCurrentState = gamepad.dpad_right;
//      pressRight = dpadRightCurrentState;

        return (dpadRightCurrentState && !dpadRightPastState);
    }

    public boolean dpadRightPress() {
        return dpadRightCurrentState;
    }

    public boolean isDpadRightToggled()
    {
        boolean dpadRightPastState = dpadRightCurrentState;
        dpadRightCurrentState = gamepad.dpad_right;

        return dpadRightCurrentState && !dpadRightPastState ? (toggleRight = !toggleRight) : toggleRight;
    }


    //left stick button
    public boolean L3() {
        return gamepad.left_stick_button;
    }

    public boolean L3StateUpdate()
    {
        boolean L3PastState = L3CurrentState;
        L3CurrentState = gamepad.left_stick_button;
//      pressLS = L3CurrentState;

        return (L3CurrentState && !L3PastState);
    }

    public boolean L3Press() {
        return L3CurrentState;
    }

    public boolean isL3Toggled()
    {
        boolean L3PastState = L3CurrentState;
        L3CurrentState = gamepad.left_stick_button;

        return L3CurrentState && !L3PastState ? (toggleLS = !toggleLS) : toggleLS;
    }


    //right stick button
    public boolean R3() {
        return gamepad.left_stick_button;
    }

    public boolean R3StateUpdate()
    {
        boolean R3PastState = R3CurrentState;
        R3CurrentState = gamepad.right_stick_button;
//      pressRS = R3CurrentState;

        return (L3CurrentState && !R3PastState);
    }

    public boolean R3Press() {
        return R3CurrentState;
    }

    public boolean isR3Toggled()
    {
        boolean R3PastState = R3CurrentState;
        R3CurrentState = gamepad.right_stick_button;

        return R3CurrentState && !R3PastState ? (toggleRS = !toggleRS) : toggleRS;
    }


    //left bumper
    public boolean L1() {
        return gamepad.left_bumper;
    }

    public boolean L1StateUpdate()
    {
        boolean L1PastState = L1CurrentState;
        L1CurrentState = gamepad.left_bumper;
//      pressLB = L1CurrentState;

        return (L1CurrentState && !L1PastState);
    }

    public boolean L1Press() {
        return L1CurrentState;
    }

    public boolean isL1Toggled()
    {
        boolean L1PastState = L1CurrentState;
        L1CurrentState = gamepad.left_bumper;

        return L1CurrentState && !L1PastState ? (toggleLB = !toggleLB) : toggleLB;
    }

    //right bumper
    public boolean R1() {
        return gamepad.right_bumper;
    }

    public boolean R1StateUpdate()
    {
        boolean R1PastState = R1CurrentState;
        R1CurrentState = gamepad.right_bumper;
//      pressRB = R1CurrentState;

        return (R1CurrentState && !R1PastState);
    }

    public boolean R1Press() {
        return R1CurrentState;
    }

    public boolean isR1Toggled()
    {
        boolean R1PastState = R1CurrentState;
        R1CurrentState = gamepad.right_bumper;

        return R1CurrentState && !R1PastState ? (toggleRB = !toggleRB) : toggleRB;
    }


    //left trigger
    public float L2() {
        return gamepad.left_trigger;
    }


    //right trigger
    public float R2() {
        return gamepad.right_trigger;
    }


    public boolean options() {
        return gamepad.options;
    }


    public boolean share() {
        return gamepad.share;
    }


    public class Thumbstick
    {

        private final double rawX;
        private final double rawY;
        private double shiftedX;
        private double shiftedY;

//        public Thumbstick(Double x, Double y) {
//            this.rawX = x;
//            this.rawY = y;
//        }

        public Thumbstick(Float x, Float y)
        {
            this.rawX = x;
            this.rawY = y;
        }

        public boolean isInput() {
            return (getX() != 0) || (getY() != 0);
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

        public double getShiftedX() {
            return shiftedX;
        }

        public double getShiftedY() {
            return shiftedY;
        }

        public double getShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getInvertedX() {
            return rawX * -1;
        }

        public double getInvertedY() {
            return rawY * -1;
        }

        public double getInvertedShiftedX() {
            return shiftedX * -1;
        }

        public double getInvertedShiftedY() {
            return shiftedY * -1;
        }

        public double getInvertedShiftedX(Double shiftAngle) {
            return getShiftedX(shiftAngle) * -1;
        }

        public double getInvertedShiftedY(Double shiftAngle) {
            return getShiftedY(shiftAngle) * -1;
        }

        public double getAngle(){
            return ((270 - (Math.atan2(0 - getInvertedY(), 0 - getInvertedX())) * 180 / Math.PI) % 360);
        }
    }
}
