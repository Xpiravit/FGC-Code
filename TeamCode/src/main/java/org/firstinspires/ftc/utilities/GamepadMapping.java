package org.firstinspires.ftc.utilities;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Created by Victo on 10/23/2017.
 */

public class GamepadMapping
{
    public enum BindType
    {
        LOOP,
        TOGGLE,
        HELD_DOWN,
        HELD_UP,
        PRESSED,
        RELEASED
    }

    public enum ButtonType
    {
        A,
        B,
        X,
        Y,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        LEFT_BUMPER,
        RIGHT_BUMPER
    }
    public static class Bind
    {
        public BindType type;
        public ButtonType button;
        public String toCall;

        public Bind(BindType type, ButtonType button, String toCall)
        {
            this.type = type;
            this.button = button;
            this.toCall = toCall;
        }
    }

    public HashMap<ButtonType, List<Bind>> Binds = new HashMap<>();
    private final RAWRController gamepad;
    private final OpMode parent;

    public GamepadMapping(RAWRController gamepad, OpMode parent){
        this.gamepad = gamepad;
        this.parent = parent;
    }

    public void AddBind(BindType type,ButtonType button, String toCall)
    {
        Bind newBind = new Bind(type, button, toCall);

        if (Binds.get(button) != null) Binds.get(button).add(newBind);
        else
        {
            List<Bind> newList =  new ArrayList<>();

            newList.add(newBind);
            Binds.put(button,newList);
        }

    }

    public void Update(){
        gamepad.Update();

        for (int i = 0; i < ButtonType.values().length; i++)
        {
            ButtonType button = ButtonType.values()[i];

            List<Bind> binds = Binds.get(button);

            if(binds != null || binds.size() > 0)
            {
                RAWRController.ButtonState _bState = GetButtonState(button);
                CheckBind(_bState, binds);
            }
        }
    }

    private RAWRController.ButtonState GetButtonState(@NonNull ButtonType button)
    {
        switch (button)
        {
            case A:
                return gamepad.AState;
            case B:
                return gamepad.BState;
        }
        return RAWRController.ButtonState.RELEASED;
    }

    private void CheckBind(RAWRController.ButtonState bState, @NonNull List<Bind> binds){
        for (int i = 0; i < binds.size(); i++)
        {
            Bind bind = binds.get(i);
            switch (bind.type)
            {
                case PRESSED:
                    if (bState == RAWRController.ButtonState.JUST_PRESSED)
                        CallBind(bind);
                    break;
                case RELEASED:
                    if (bState == RAWRController.ButtonState.JUST_RELEASED)
                        CallBind(bind);
                    break;
                case HELD_DOWN:
                    if (bState == RAWRController.ButtonState.PRESSED)
                        CallBind(bind);
                    break;
                case HELD_UP:
                    if (bState == RAWRController.ButtonState.RELEASED)
                        CallBind(bind);
                    break;
            }
        }
    }

    private void CallBind(@NonNull Bind bindToCall)
    {
        try
        {
            parent.getClass().getMethod(bindToCall.toCall);
        }
        catch (NoSuchMethodException e)
        {
            e.printStackTrace();
        }
    }
}
