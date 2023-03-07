package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.drivetrain.drivingParameters.transmission.DrivetrainToggleTransmissionMode;
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
import frc.robot.commands.transport.compound.WristArmGoTo;
import frc.robot.commands.transport.wrist.DriveWrist;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot;
    private HighAltitudeJoystick copilot;

    private Joystick pit;
    private JoystickButton pit_7;
    private JoystickButton pit_8;
    private JoystickButton pit_12;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
        pit = new Joystick(1);
        copilot = new HighAltitudeJoystick(2, JoystickType.XBOX);
        // pit_7 = new JoystickButton(pit, 7);
        // pit_8 = new JoystickButton(pit, 8);
        // pit_12 = new JoystickButton(pit, 12);

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

        pilot.onTrue(ButtonType.RS, new DrivetrainToggleTransmissionMode()); // MORA
        // copilot.onTrue(ButtonType.A, new DrivetrainToggleTransmissionMode()); // ITAI
        // pit_12.onTrue(new DrivetrainToggleTransmissionMode());

        // pilot.whileTrue(ButtonType.POV_E, new DriveWrist());
        // pilot.whileTrue(ButtonType.POV_W, new DriveWrist());

        // pit_7.onTrue(new ResetTransportEncoders());
        // pit_8.onTrue(new ResetNavx());
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
        return pilot.getAxis(AxisType.LEFT_X) * 0.75 * 0.75; // CAMBIAR DE VUELTA A MORA MAYBE
        // return 0.75 * pit.getRawAxis(0) * ((-pilot.getRawAxis(3) + 1) / 2);
    }

    public double getDefaultDriveY() {
        return -pilot.getAxis(AxisType.LEFT_Y) * 0.75; // CAMBIAR DE VUELTA A MORA
        // return (-pit.getAxisType(1)) * ((-pilot.getRawAxis(3) + 1) / 2);
    }

    public double getDefaultDriveTurn() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getDefaultDriveDragonfly() {
        return pilot.getAxis(AxisType.RIGHT_X);
    }

    public double getWristInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? pilot.getPovXAxis() * 0.125
                : pilot.getPovXAxis() * 0.25;
    }

    public double getArmInput() {
        return -pilot.getAxis(AxisType.RIGHT_Y) * 0.25;
    }

    public double getExtensorInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? pilot.getTriggers() * 0.25
                : pilot.getTriggers() * 0.75;
    }

    public HighAltitudeJoystick getPilot() {
        return pilot;
    }

}
