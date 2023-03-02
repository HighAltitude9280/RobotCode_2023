package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.drivetrain.drivingParameters.transmission.DrivetrainToggleTransmissionMode;
import frc.robot.commands.drivetrain.follower.FollowTargetJolt;
import frc.robot.commands.pieceHandlers.compound.GlobalIntake;
import frc.robot.commands.pieceHandlers.compound.GlobalOuttake;
import frc.robot.commands.pieceHandlers.intake.ToggleIntakePosition;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.robotParameters.ToggleShouldExtensorBeLimitedManual;
import frc.robot.commands.robotParameters.ToggleShouldManualBeLimited;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.TransportGoTo;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

        pilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        pilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));
        pilot.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL),
                ButtonType.START, ButtonType.BACK);

        pilot.whileTrue(ButtonType.LB, new GlobalIntake());
        pilot.whileTrue(ButtonType.RB, new GlobalOuttake());
        pilot.onTrueCombo(new ToggleShouldExtensorBeLimitedManual(),
                ButtonType.LB, ButtonType.RB);

        pilot.onTrue(ButtonType.POV_N, new ToggleIntakePosition());
        pilot.onTrue(ButtonType.POV_S, new ToggleShouldManualBeLimited());

        pilot.whileTrue(ButtonType.JOYSTICK_R_X, new FollowTargetJolt());

        pilot.whileTrue(ButtonType.Y, new TransportGoTo(TransportTarget.TOP_ROW));
        pilot.whileTrue(ButtonType.B, new TransportGoTo(TransportTarget.FEEDER));
        pilot.whileTrue(ButtonType.A, new TransportGoTo(TransportTarget.RESTING));
        pilot.whileTrue(ButtonType.X, new TransportGoTo(TransportTarget.MIDDLE_ROW));

        pilot.onTrue(ButtonType.RS, new DrivetrainToggleTransmissionMode());
    }

    /*
     * public void ConfigureButtonBindings() {
     * pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
     * copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);
     * 
     * copilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
     * copilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));
     * copilot.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL),
     * ButtonType.START, ButtonType.BACK);
     * pilot.whileTrue(ButtonType.LB, new IntakeIn());
     * pilot.whileTrue(ButtonType.RB, new IntakeOut());
     * pilot.whileTrue(ButtonType.Y, new GripperIn());
     * pilot.whileTrue(ButtonType.X, new GripperOut());
     * pilot.onTrue(ButtonType.A, new
     * DrivetrainSetTransmission(TransmissionMode.speed));
     * pilot.onTrue(ButtonType.B, new
     * DrivetrainSetTransmission(TransmissionMode.torque));
     * pilot.onTrue(ButtonType.BACK, new SetIntakePosition(IntakePosition.STORED));
     * pilot.onTrue(ButtonType.START, new
     * SetIntakePosition(IntakePosition.LOWERED));
     * pilot.onTrue(ButtonType.START, new ToggleGamePieceMode());
     * 
     * copilot.onTrue(ButtonType.POV_N, new ResetTransportEncoders());
     * copilot.onTrue(ButtonType.POV_S, new ToggleShouldManualBeLimited());
     * 
     * // MIDDLE ROW CONE
     * // copilot.whileTrue(ButtonType.A, new DriveWristToPosition(407, 0.4125));
     * // copilot.whileTrue(ButtonType.B, new DriveArmToPosition(198, 0.875));
     * // copilot.whileTrue(ButtonType.Y, new DriveExtensorToPosition(0.2, 1));
     * 
     * // TOP ROW CONE (JALA)
     * // copilot.whileTrue(ButtonType.A, new DriveWristToPosition(402, 0.4125));
     * // copilot.whileTrue(ButtonType.B, new DriveArmToPosition(177, 0.875));
     * // copilot.whileTrue(ButtonType.Y, new DriveExtensorToPosition(0.47, 1));
     * 
     * copilot.whileTrue(ButtonType.X, new
     * TransportGoTo(TransportTarget.MIDDLE_ROW));
     * copilot.whileTrue(ButtonType.Y, new TransportGoTo(TransportTarget.TOP_ROW));
     * copilot.whileTrue(ButtonType.A, new TransportGoTo(TransportTarget.RESTING));
     * copilot.whileTrue(ButtonType.B, new TransportGoTo(TransportTarget.INTAKE));
     * // pilot.onTrue(ButtonType.A, new ToggleIntakePosition());
     * // pilot.onTrue(ButtonType.LB, new IntakeIn());
     * // pilot.onTrue(ButtonType.RB, new IntakeOut());
     * 
     * // pilot.whileTrue(ButtonType.A, new
     * TransportGoTo(TransportTarget.MIDDLE_ROW));
     * 
     * // copilot.onTrue(ButtonType.X, new
     * TransportGoTo(TransportTarget.MIDDLE_ROW));
     * // copilot.onTrue(ButtonType.Y, new TransportGoTo(TransportTarget.TOP_ROW));
     * // copilot.onTrue(ButtonType.B, new TransportGoTo(TransportTarget.INTAKE));
     * }
     */
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getDefaultDriveX() {
        return pilot.getAxis(AxisType.LEFT_X) * 0.75;
    }

    public double getDefaultDriveY() {
        return -pilot.getAxis(AxisType.LEFT_Y);
    }

    public double getDefaultDriveTurn() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getDefaultDriveDragonfly() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getWristInput() {
        return pilot.getPovXAxis() * 0.25;
    }

    public double getArmInput() {
        return -pilot.getAxis(AxisType.RIGHT_Y) * 0.25;
    }

    public double getExtensorInput() {
        return pilot.getTriggers() * 0.75;
    }

    public HighAltitudeJoystick getPilot() {
        return pilot;
    }

}
