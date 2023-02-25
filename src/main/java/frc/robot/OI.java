package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.ToggleIntakePosition;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.compound.TransportGoTo;
import frc.robot.commands.transport.compound.TransportGoTo.TransportTarget;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot, copilot;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
        copilot = new HighAltitudeJoystick(1, JoystickType.UNKNOWN);

        pilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        pilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));
        pilot.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL), ButtonType.START, ButtonType.BACK);

        pilot.onTrue(ButtonType.A, new ToggleIntakePosition());
        pilot.onTrue(ButtonType.LB, new IntakeIn());
        pilot.onTrue(ButtonType.RB, new IntakeOut());

        copilot.onTrue(ButtonType.A, new TransportGoTo(TransportTarget.BOTTOM_ROW));
        copilot.onTrue(ButtonType.X, new TransportGoTo(TransportTarget.MIDDLE_ROW));
        copilot.onTrue(ButtonType.Y, new TransportGoTo(TransportTarget.TOP_ROW));
        copilot.onTrue(ButtonType.B, new TransportGoTo(TransportTarget.INTAKE));
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getDefaultDriveX() {
        return pilot.getAxis(AxisType.LEFT_X);
    }

    public double getDefaultDriveY() {
        return pilot.getAxis(AxisType.LEFT_Y);
    }

    public double getDefaultDriveTurn() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getDefaultDriveDragonfly() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getWristInput() {
        return pilot.getPovXAxis();
    }

    public double getArmInput() {
        return pilot.getPovYAxis();
    }

    public double getExtensorInput() {
        return pilot.getTriggers();
    }

    public HighAltitudeJoystick getPilot() {
        return pilot;
    }

    public HighAltitudeJoystick getCopilot() {
        return copilot;
    }

}
