// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.joysticks;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.resources.math.Math;

/** Add your docs here. */
public class HighAltitudeJoystick {

    private Joystick joystick;
    private JoystickType joystickType;
    private Haptics haptics;

    public enum JoystickType {
        PS4,
        XBOX,
        UNKNOWN
    }

    public enum ButtonType {
        A,
        B,
        X,
        Y,
        LB,
        RB,
        BACK,
        START,
        LS,
        RS,
        PS,
        TOUCHPAD,
        POV_N,
        POV_NE,
        POV_E,
        POV_SE,
        POV_S,
        POV_SW,
        POV_W,
        POV_NW,
        POV_NULL,
        LT,
        RT,
        JOYSTICK_L_X,
        JOYSTICK_L_Y,
        JOYSTICK_R_X,
        JOYSTICK_R_Y
    }

    public enum AxisType {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    private HashMap<Integer, JoystickButton> availableJoystickButtons;
    private HashMap<Integer, POVButton> availablePOVButtons;
    private HashMap<Integer, Trigger> availableAxisButtons;

    private HashMap<ButtonType, Trigger> joystickButtonConfiguration;

    private HashMap<AxisType, Integer> axisConfiguration;
    private HashMap<AxisType, Double> axisDeadzoneConfiguration;
    private HashMap<AxisType, Double> axisMultiplierConfiguration;

    /**
     * 
     * Creates a new {@link HighAltitudeJoystick}, which can be either XBOX or PS4.
     * This Joystick has associated buttons.
     * 
     * Though there's some support for an
     * UNKNOWN {@link JoystickType}, the specific case will have to be handled
     * manually by the programmer using {@link #getJoystickButtonObj(int)}.
     * 
     * 
     * @param port The port of the controller.
     * @param type The {@link JoystickType} of the controller.
     */

    public HighAltitudeJoystick(int port, JoystickType type) {
        this.joystick = new Joystick(port);
        this.joystickType = type;

        switch (type) {
            case XBOX:
                configureXboxJoystick();
                break;
            case PS4:
                configurePs4Joystick();
                break;
            case UNKNOWN:
                configureUnknownJoystick();
                break;
        }
        configureDefaultDeadzoneAndMultiplier(0, 1);
        haptics = new Haptics(joystick);
    }

    private void configureXboxJoystick() {
        availableJoystickButtons = new HashMap<Integer, JoystickButton>();
        for (int i = 1; i <= 10; i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        availablePOVButtons = new HashMap<Integer, POVButton>();
        availablePOVButtons.put(-1, new POVButton(joystick, -1));
        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        availableAxisButtons = new HashMap<Integer, Trigger>();

        // MAPPING STARTS HERE
        axisConfiguration = new HashMap<AxisType, Integer>();
        axisConfiguration.put(AxisType.LEFT_X, 0);
        axisConfiguration.put(AxisType.LEFT_Y, 1);
        axisConfiguration.put(AxisType.LEFT_TRIGGER, 2);
        axisConfiguration.put(AxisType.RIGHT_TRIGGER, 3);
        axisConfiguration.put(AxisType.RIGHT_X, 4);
        axisConfiguration.put(AxisType.RIGHT_Y, 5);

        joystickButtonConfiguration = new HashMap<ButtonType, Trigger>();
        joystickButtonConfiguration.put(ButtonType.A, availableJoystickButtons.get(1));
        joystickButtonConfiguration.put(ButtonType.B, availableJoystickButtons.get(2));
        joystickButtonConfiguration.put(ButtonType.X, availableJoystickButtons.get(3));
        joystickButtonConfiguration.put(ButtonType.Y, availableJoystickButtons.get(4));
        joystickButtonConfiguration.put(ButtonType.LB, availableJoystickButtons.get(5));
        joystickButtonConfiguration.put(ButtonType.RB, availableJoystickButtons.get(6));
        joystickButtonConfiguration.put(ButtonType.BACK, availableJoystickButtons.get(7));
        joystickButtonConfiguration.put(ButtonType.START, availableJoystickButtons.get(8));
        joystickButtonConfiguration.put(ButtonType.LS, availableJoystickButtons.get(9));
        joystickButtonConfiguration.put(ButtonType.RS, availableJoystickButtons.get(10));

        joystickButtonConfiguration.put(ButtonType.POV_NULL, availablePOVButtons.get(-1));
        joystickButtonConfiguration.put(ButtonType.POV_N, availablePOVButtons.get(0));
        joystickButtonConfiguration.put(ButtonType.POV_NE, availablePOVButtons.get(45));
        joystickButtonConfiguration.put(ButtonType.POV_E, availablePOVButtons.get(90));
        joystickButtonConfiguration.put(ButtonType.POV_SE, availablePOVButtons.get(135));
        joystickButtonConfiguration.put(ButtonType.POV_S, availablePOVButtons.get(180));
        joystickButtonConfiguration.put(ButtonType.POV_SW, availablePOVButtons.get(225));
        joystickButtonConfiguration.put(ButtonType.POV_W, availablePOVButtons.get(270));
        joystickButtonConfiguration.put(ButtonType.POV_NW, availablePOVButtons.get(315));
        // MAPPING ENDS HERE

        for (AxisType axisType : AxisType.values()) {
            int port = axisConfiguration.get(axisType);
            BooleanSupplier booleanSupplier = () -> isAxisPressed(axisType);
            availableAxisButtons.put(port, new Trigger(booleanSupplier));
        }

        joystickButtonConfiguration.put(ButtonType.LT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.RT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_Y)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_Y)));

    }

    private void configurePs4Joystick() {
        availableJoystickButtons = new HashMap<Integer, JoystickButton>();
        for (int i = 1; i <= 14; i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        availablePOVButtons = new HashMap<Integer, POVButton>();
        availablePOVButtons.put(-1, new POVButton(joystick, -1));
        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        availableAxisButtons = new HashMap<Integer, Trigger>();

        // MAPPING STARTS HERE
        axisConfiguration = new HashMap<AxisType, Integer>();
        axisConfiguration.put(AxisType.LEFT_X, 0);
        axisConfiguration.put(AxisType.LEFT_Y, 1);
        axisConfiguration.put(AxisType.LEFT_TRIGGER, 3);
        axisConfiguration.put(AxisType.RIGHT_TRIGGER, 4);
        axisConfiguration.put(AxisType.RIGHT_X, 2);
        axisConfiguration.put(AxisType.RIGHT_Y, 5);

        joystickButtonConfiguration = new HashMap<ButtonType, Trigger>();
        joystickButtonConfiguration.put(ButtonType.A, availableJoystickButtons.get(2));
        joystickButtonConfiguration.put(ButtonType.B, availableJoystickButtons.get(3));
        joystickButtonConfiguration.put(ButtonType.X, availableJoystickButtons.get(1));
        joystickButtonConfiguration.put(ButtonType.Y, availableJoystickButtons.get(4));
        joystickButtonConfiguration.put(ButtonType.LB, availableJoystickButtons.get(5));
        joystickButtonConfiguration.put(ButtonType.RB, availableJoystickButtons.get(6));
        joystickButtonConfiguration.put(ButtonType.BACK, availableJoystickButtons.get(9));
        joystickButtonConfiguration.put(ButtonType.START, availableJoystickButtons.get(10));
        joystickButtonConfiguration.put(ButtonType.LS, availableJoystickButtons.get(11));
        joystickButtonConfiguration.put(ButtonType.RS, availableJoystickButtons.get(12));
        joystickButtonConfiguration.put(ButtonType.PS, availableJoystickButtons.get(13));
        joystickButtonConfiguration.put(ButtonType.TOUCHPAD, availableJoystickButtons.get(14));

        joystickButtonConfiguration.put(ButtonType.POV_NULL, availablePOVButtons.get(-1));
        joystickButtonConfiguration.put(ButtonType.POV_N, availablePOVButtons.get(0));
        joystickButtonConfiguration.put(ButtonType.POV_NE, availablePOVButtons.get(45));
        joystickButtonConfiguration.put(ButtonType.POV_E, availablePOVButtons.get(90));
        joystickButtonConfiguration.put(ButtonType.POV_SE, availablePOVButtons.get(135));
        joystickButtonConfiguration.put(ButtonType.POV_S, availablePOVButtons.get(180));
        joystickButtonConfiguration.put(ButtonType.POV_SW, availablePOVButtons.get(225));
        joystickButtonConfiguration.put(ButtonType.POV_W, availablePOVButtons.get(270));
        joystickButtonConfiguration.put(ButtonType.POV_NW, availablePOVButtons.get(315));
        // MAPPING ENDS HERE

        for (AxisType axisType : AxisType.values()) {
            int port = axisConfiguration.get(axisType);
            BooleanSupplier booleanSupplier = () -> isAxisPressed(axisType);
            availableAxisButtons.put(port, new Trigger(booleanSupplier));
        }

        joystickButtonConfiguration.put(ButtonType.LT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.RT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_Y)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_Y)));
    }

    private void configureUnknownJoystick() {
        availableJoystickButtons = new HashMap<Integer, JoystickButton>();
        for (int i = 1; i <= joystick.getButtonCount(); i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        availablePOVButtons = new HashMap<Integer, POVButton>();
        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        availableAxisButtons = new HashMap<Integer, Trigger>();
        for (int i = 0; i < joystick.getAxisCount(); i++) {
            int currentPort = i;
            BooleanSupplier booleanSupplier = () -> isAxisPressed(currentPort);
            availableAxisButtons.put(i, new Trigger(booleanSupplier));
        }

    }

    private void configureDefaultDeadzoneAndMultiplier(double deadzone, double multiplier) {
        axisDeadzoneConfiguration = new HashMap<AxisType, Double>();
        axisMultiplierConfiguration = new HashMap<AxisType, Double>();
        for (AxisType axisType : AxisType.values()) {
            axisDeadzoneConfiguration.put(axisType, deadzone);
            axisMultiplierConfiguration.put(axisType, multiplier);
        }
    }

    /**
     * Will return the RAW value of the given {@link AxisType}. The mapping that
     * relates {@link AxisType} with the corresponding port is
     * {@link axisConfiguration}. PS4 triggers will return a value from -1 to 1
     * instead of returning from 0-1.
     * 
     * @param axisType The desired axis
     * @return Raw axis value
     */
    public double getRawAxis(AxisType axisType) {
        Integer port = axisConfiguration.get(axisType);
        if (port != null)
            return joystick.getRawAxis(axisConfiguration.get(axisType));
        DriverStation.reportError("Axis " + axisType + " not found. Returning 0.", true);
        return 0;
    }

    /**
     * Will return the RAW value of the given axis port. PS4 triggers will return a
     * value from -1 to 1 instead of returning from 0-1.
     * 
     * @param axisType The port of the desired axis
     * @return Raw axis value
     */
    public double getRawAxis(int axisPort) {
        return joystick.getRawAxis(axisPort);
    }

    /**
     * Will return the PROCESSED value of the chosen axis, applying both the set
     * <b>deadzone</b> and <b>multiplier</b>. PS4 Triggers are given in standard 0
     * to 1 instead of -1 to 1. Use {@link #setAxisDeadzone(AxisType, double)} and
     * {@link #setAxisMultiplier(AxisType, double)} to modify these values.
     * 
     * @param axis The desired axis
     * @return Processed axis value
     */
    public double getAxis(AxisType axis) {
        double input = getRawAxis(axis);
        if (joystickType == JoystickType.PS4
                && (axis.equals(AxisType.LEFT_TRIGGER) || axis.equals(AxisType.RIGHT_TRIGGER))) {
            input = (input + 1) / 2;
        }
        double clampedInput = Math.clampToDeadzone(input, axisDeadzoneConfiguration.get(axis));
        double multipliedInput = clampedInput * axisMultiplierConfiguration.get(axis);
        return multipliedInput;
    }

    public double getTriggers() {
        double left = getAxis(AxisType.LEFT_TRIGGER);
        double right = getAxis(AxisType.RIGHT_TRIGGER);

        return right - left;
    }

    public boolean isAxisPressed(AxisType axisType) {
        return Math.abs(getAxis(axisType)) > 0.5;
    }

    public boolean isAxisPressed(int axisPort) {
        return Math.abs(getRawAxis(axisPort)) > 0.5;
    }

    public void setAxisDeadzone(AxisType axis, double deadzone) {
        axisDeadzoneConfiguration.put(axis, Math.abs(deadzone));
    }

    public void setAxisMultiplier(AxisType axis, double multiplier) {
        axisMultiplierConfiguration.put(axis, multiplier);
    }

    public Trigger getButtonObj(ButtonType buttonType) {
        return joystickButtonConfiguration.get(buttonType);
    }

    public JoystickButton getJoystickButtonObj(int port) {
        return availableJoystickButtons.get(port);
    }

    public Trigger getPOVButtonObj(int angle) {
        return availablePOVButtons.get(angle);
    }

    public Trigger getAxisButtonObj(int axis) {
        return availableAxisButtons.get(axis);
    }

    public Haptics getHaptics() {
        return haptics;
    }

    // METHODS FOR ASSOCIATING COMMANDS YES THEY'RE A LOT BUT WE'D RATHER HAVE IT
    // THIS WAY

    /**
     * Starts the given command whenever the button changes from 'unpressed' to
     * 'pressed'.
     * Won't cancel the command.
     * 
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void onTrue(ButtonType buttonType, CommandBase command) {
        Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.onTrue(command);
        else
            reportButtonError(buttonType, command);
    }

    /**
     * Starts the given command whenever the condition changes from 'unpressed' to
     * 'pressed'.
     * Cancels the given command whenever the condition changes from 'pressed' to
     * 'unpressed'.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void whileTrue(ButtonType buttonType, CommandBase command) {
        Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.whileTrue(command);
        else
            reportButtonError(buttonType, command);
    }

    /**
     * When the condition changes from 'unpressed' to 'pressed', starts the command
     * if it's not running
     * and cancels the command if it's already running.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void toggleOnTrue(ButtonType buttonType, CommandBase command) {
        Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.toggleOnTrue(command);
        else
            reportButtonError(buttonType, command);
    }

    /**
     * Starts the given command whenever the condition changes from 'pressed' to
     * 'unpressed'.
     * Won't cancel the command.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void onFalse(ButtonType buttonType, CommandBase command) {
        Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.onFalse(command);
        else
            reportButtonError(buttonType, command);
    }

    /**
     * Starts the given command whenever the condition changes from 'pressed' to
     * 'unpressed'.
     * Cancels the given command whenever the condition changes from 'unpressed' to
     * 'pressed'.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void whileFalse(ButtonType buttonType, CommandBase command) {
        Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.whileFalse(command);
        else
            reportButtonError(buttonType, command);
    }

    /**
     * When the condition changes from 'pressed' to 'unpressed', starts the command
     * if it's not running
     * and cancels the command if it's already running.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void toggleOnFalse(ButtonType buttonType, CommandBase command) {
        Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.toggleOnFalse(command);
        else
            reportButtonError(buttonType, command);
    }

    /**
     * Starts the given command whenever the condition of ALL chosen buttons
     * is 'pressed'. Won't cancel the command.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will trigger the command
     */
    public void onTrueCombo(CommandBase command, ButtonType... buttons) {
        Trigger triggerList = joystickButtonConfiguration.get(buttons[0]);
        for (int i = 1; i < buttons.length; i++) {
            Trigger chosenButton = joystickButtonConfiguration.get(buttons[i]);
            if (chosenButton != null)
                triggerList = triggerList.and(chosenButton);
            else
                reportButtonErrorCombo(buttons[i], command);
        }
        Trigger buttonList = new Trigger(triggerList);

        buttonList.onTrue(command);
    }

    /**
     * Starts the given command whenever the condition of ALL chosen buttons
     * is 'pressed'. Cancells the command when at least one of the buttons is
     * 'unpressed'.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will trigger the command
     */
    public void whileTrueCombo(CommandBase command, ButtonType... buttons) {
        Trigger triggerList = joystickButtonConfiguration.get(buttons[0]);
        for (int i = 1; i < buttons.length; i++) {
            Trigger chosenButton = joystickButtonConfiguration.get(buttons[i]);
            if (chosenButton != null)
                triggerList = triggerList.and(chosenButton);
            else
                reportButtonErrorCombo(buttons[i], command);
        }
        Trigger buttonList = new Trigger(triggerList);

        buttonList.whileTrue(command);
    }

    /**
     * When all buttons are 'pressed', starts the command if it's not running
     * and cancels the command if it's already running.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will trigger the command
     */
    public void toggleOnTrueCombo(CommandBase command, ButtonType... buttons) {
        Trigger triggerList = joystickButtonConfiguration.get(buttons[0]);
        for (int i = 1; i < buttons.length; i++) {
            Trigger chosenButton = joystickButtonConfiguration.get(buttons[i]);
            if (chosenButton != null)
                triggerList = triggerList.and(chosenButton);
            else
                reportButtonErrorCombo(buttons[i], command);
        }
        Trigger buttonList = new Trigger(triggerList);

        buttonList.toggleOnTrue(command);
    }

    private void reportButtonError(ButtonType b, CommandBase c) {
        DriverStation.reportWarning("Button " + b + " not found! The command " + c + " won't be assigned.", true);
    }

    private void reportButtonErrorCombo(ButtonType b, CommandBase c) {
        DriverStation.reportWarning("Button " + b + " not found when assigning combo for " + c, true);
    }
}
