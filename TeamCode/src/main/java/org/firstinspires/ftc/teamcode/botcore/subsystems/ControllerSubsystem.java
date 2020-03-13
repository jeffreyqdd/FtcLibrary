package org.firstinspires.ftc.teamcode.botcore.subsystems;

import org.firstinspires.ftc.teamcode.botcore.framework.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.framework.Subsystem;


/*
 * to anyone who is reading this
 * I have no clue why I added this many joystick scaling options
 */


public class ControllerSubsystem extends Subsystem {

    //define joystick scaling method / curves

    public enum JoystickScale
    {
        linearTenth, linearHalf, linearFull,


        //input^n, weak has low n while strong has high n
        exponentialWeak, exponentialModerate, exponentialStrong,

        sigmoid,

        binary,

    }


    private MeasurementPackage ref;
    private JoystickScale leftCurveX, leftCurveY, rightCurveX, rightCurveY;
    private JoystickScale leftCurveTrigger, rightCurveTrigger;
    private boolean invertYAxis;


    public ControllerSubsystem(String name, BotManager botmgr, double rate) {
        super(name, botmgr, rate);

        //store a reference to the global measurement package
        this.ref = botmgr.getMeasurementPackage();

        //set default joystick tracking methods
        this.leftCurveX = JoystickScale.linearFull;
        this.leftCurveY = JoystickScale.linearFull;
        this.rightCurveX = JoystickScale.linearFull;
        this.rightCurveY = JoystickScale.linearFull;

        this.leftCurveTrigger = JoystickScale.linearFull;
        this.rightCurveTrigger = JoystickScale.linearFull;

        //inverts y axis
        this.invertYAxis = true;
    }

    public void setLeftCurveX(JoystickScale leftCurveX) {
        this.leftCurveX = leftCurveX;
    }

    public void setLeftCurveY(JoystickScale leftCurveY) {
        this.leftCurveY = leftCurveY;
    }

    public void setRightCurveX(JoystickScale rightCurveX) {
        this.rightCurveX = rightCurveX;
    }

    public void setRightCurveY(JoystickScale rightCurveY) {
        this.rightCurveY = rightCurveY;
    }

    public void setLeftCurveTrigger(JoystickScale leftCurveTrigger) {
        this.leftCurveTrigger = leftCurveTrigger;
    }

    public void setRightCurveTrigger(JoystickScale rightCurveTrigger) {
        this.rightCurveTrigger = rightCurveTrigger;
    }

    public void setInvertYAxis(boolean invertYAxis)
    {
        this.invertYAxis = invertYAxis;
    }


    @Override
    public void execute() {
        readControllerInput();
    }

    private void readControllerInput()
    {
        //read gamepad 1;
        ref.gamepad1.buttonA = opmode.gamepad1.a ? 1.0 : 0.0;
        ref.gamepad1.buttonB = opmode.gamepad1.b ? 1.0 : 0.0;
        ref.gamepad1.buttonX = opmode.gamepad1.x ? 1.0 : 0.0;
        ref.gamepad1.buttonY = opmode.gamepad1.y ? 1.0 : 0.0;

        ref.gamepad1.dpadUp = opmode.gamepad1.dpad_up? 1.0 : 0.0;
        ref.gamepad1.dpadDown = opmode.gamepad1.dpad_down? 1.0 : 0.0;
        ref.gamepad1.dpadLeft = opmode.gamepad1.dpad_left? 1.0 : 0.0;
        ref.gamepad1.dpadRight = opmode.gamepad1.dpad_right? 1.0 : 0.0;

        ref.gamepad1.leftBumper = opmode.gamepad1.left_bumper? 1.0 : 0.0;
        ref.gamepad1.rightBumper = opmode.gamepad1.right_bumper? 1.0 : 0.0;

        ref.gamepad1.leftTrigger = applyCurve(opmode.gamepad1.left_trigger, leftCurveTrigger);
        ref.gamepad1.rightTrigger = applyCurve(opmode.gamepad1.right_trigger, rightCurveTrigger);

        ref.gamepad1.leftStickX = applyCurve(opmode.gamepad1.left_stick_x, leftCurveX);
        ref.gamepad1.leftStickY = applyCurve(opmode.gamepad1.left_stick_y, leftCurveY) * (invertYAxis ? -1.0 : 1.0);
        ref.gamepad1.rightStickX = applyCurve(opmode.gamepad1.right_stick_x, rightCurveX);
        ref.gamepad1.rightStickY = applyCurve(opmode.gamepad1.right_stick_y, rightCurveY) * (invertYAxis ? -1.0 : 1.0);


        //read gamepad 2;
        ref.gamepad2.buttonA = opmode.gamepad2.a ? 1.0 : 0.0;
        ref.gamepad2.buttonB = opmode.gamepad2.b ? 1.0 : 0.0;
        ref.gamepad2.buttonX = opmode.gamepad2.x ? 1.0 : 0.0;
        ref.gamepad2.buttonY = opmode.gamepad2.y ? 1.0 : 0.0;

        ref.gamepad2.dpadUp = opmode.gamepad2.dpad_up? 1.0 : 0.0;
        ref.gamepad2.dpadDown = opmode.gamepad2.dpad_down? 1.0 : 0.0;
        ref.gamepad2.dpadLeft = opmode.gamepad2.dpad_left? 1.0 : 0.0;
        ref.gamepad2.dpadRight = opmode.gamepad2.dpad_right? 1.0 : 0.0;

        ref.gamepad2.leftBumper = opmode.gamepad2.left_bumper? 1.0 : 0.0;
        ref.gamepad2.rightBumper = opmode.gamepad2.right_bumper? 1.0 : 0.0;

        ref.gamepad2.leftTrigger = applyCurve(opmode.gamepad2.left_trigger, leftCurveTrigger);
        ref.gamepad2.rightTrigger = applyCurve(opmode.gamepad2.right_trigger, rightCurveTrigger);

        ref.gamepad2.leftStickX = applyCurve(opmode.gamepad2.left_stick_x, leftCurveX);
        ref.gamepad2.leftStickY = applyCurve(opmode.gamepad2.left_stick_y, leftCurveY) * (invertYAxis ? -1.0 : 1.0);
        ref.gamepad2.rightStickX = applyCurve(opmode.gamepad2.right_stick_x, rightCurveX);
        ref.gamepad2.rightStickY = applyCurve(opmode.gamepad2.right_stick_y, rightCurveY) * (invertYAxis ? -1.0 : 1.0);

    }


    private double applyCurve(double value, JoystickScale curveType)
    {
        double ret = 0;

        switch(curveType)
        {
            case binary:
                if(value == 0)
                {
                    ret = 0;
                }
                else if(value > 0)
                {
                    ret = 1;
                }
                else
                {
                    ret = -1;
                }
                break;

            case sigmoid:
                ret = ( (2.0) / (1.0 + Math.pow(Math.E, -4.0 * value) )  ) - 1.0;
                break;

            case linearFull:
                ret = value;
                break;

            case linearHalf:
                ret = value * 0.5;
                break;

            case linearTenth:
                ret = value * 0.1;
                break;

            case exponentialWeak:
                ret = Math.pow(value, 3);
                break;

            case exponentialModerate:
                ret = Math.pow(value, 5);
                break;

            case exponentialStrong:
                ret = Math.pow(value, 7);
                break;
        }

        return ret;
    }

}
