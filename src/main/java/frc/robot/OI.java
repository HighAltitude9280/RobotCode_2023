package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.drivetrain.drivingParameters.transmission.DrivetrainToggleTransmissionMode;
import frc.robot.commands.drivetrain.drivingSensors.ResetOdometry;
import frc.robot.commands.drivetrain.follower.FollowTargetJolt;
import frc.robot.commands.pieceHandlers.compound.GlobalIntake;
import frc.robot.commands.pieceHandlers.compound.GlobalOuttake;
import frc.robot.commands.pieceHandlers.intake.ToggleIntakePosition;
import frc.robot.commands.robotParameters.ResetNavx;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.robotParameters.ToggleShouldExtensorBeLimitedManual;
import frc.robot.commands.robotParameters.ToggleShouldManualHaveLimits;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.ResetTransportEncoders;
import frc.robot.commands.transport.compound.TransportGoTo;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot; // SUBSYSTEMS
    private HighAltitudeJoystick copilot; // CHASSIS, IF HIGHALTITUDECONSTANTS.SINGLE_DRIVER IS FALSE

    private HighAltitudeJoystick pit;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
        copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

        pit = new HighAltitudeJoystick(2, 12, 4); // Logitech Extreme 3D Pro

        pilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        pilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));
        pilot.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL),
                ButtonType.START, ButtonType.BACK);

        pilot.whileTrue(ButtonType.LB, new GlobalIntake());
        pilot.whileTrue(ButtonType.RB, new GlobalOuttake());
        pilot.onTrueCombo(new ToggleShouldExtensorBeLimitedManual(),
                ButtonType.LB, ButtonType.RB);

        pilot.onTrue(ButtonType.POV_N, new ToggleIntakePosition());
        pilot.onTrue(ButtonType.POV_S, new ToggleShouldManualHaveLimits());

        pilot.whileTrue(ButtonType.JOYSTICK_R_X, new FollowTargetJolt());

        pilot.whileTrue(ButtonType.Y, new TransportGoTo(TransportTarget.TOP_ROW));
        pilot.whileTrue(ButtonType.B, new TransportGoTo(TransportTarget.FEEDER));
        pilot.whileTrue(ButtonType.A, new TransportGoTo(TransportTarget.RESTING));
        pilot.whileTrue(ButtonType.X, new TransportGoTo(TransportTarget.MIDDLE_ROW));

        if (HighAltitudeConstants.SINGLE_DRIVER)
            pilot.onTrue(ButtonType.RS, new DrivetrainToggleTransmissionMode()); // SINGLE DRIVER
        else
            copilot.onTrue(ButtonType.A, new DrivetrainToggleTransmissionMode()); // COPILOT

        pit.getJoystickButtonObj(7).onTrue(new ResetTransportEncoders());
        pit.getJoystickButtonObj(8).onTrue(new ResetNavx());
        pit.getJoystickButtonObj(9).onTrue(new ResetOdometry(0, 0));
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getDefaultDriveX() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return pilot.getAxis(AxisType.LEFT_X) * 0.75 * 0.75;
        else
            return copilot.getAxis(AxisType.LEFT_X) * 0.75 * 0.75;
    }

    public double getDefaultDriveY() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return -pilot.getAxis(AxisType.LEFT_Y) * 0.75;
        else
            return -copilot.getAxis(AxisType.LEFT_Y) * 0.75;
    }

    public double getDefaultDriveTurn() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getDefaultDriveDragonfly() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getWristInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? pilot.getPovXAxis() * 0.125
                : pilot.getPovXAxis() * 0.5;
    }

    public double getArmInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? -pilot.getAxis(AxisType.RIGHT_Y) * 0.25
                : -pilot.getAxis(AxisType.RIGHT_Y) * 1.0;
    }

    public double getExtensorInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? pilot.getTriggers() * 0.25
                : pilot.getTriggers() * 0.75;
    }

    public HighAltitudeJoystick getPilot() {
        return pilot;
    }

}
