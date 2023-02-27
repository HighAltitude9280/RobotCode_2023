package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.gripper.GripperIn;
import frc.robot.commands.gripper.GripperOut;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.SetIntakePosition;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.robotParameters.ToggleGamePieceMode;
import frc.robot.commands.robotParameters.ToggleShouldManualBeLimited;
import frc.robot.commands.transport.compound.ResetTransportEncoders;
import frc.robot.commands.transport.compound.TransportGoTo;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.extensor.DriveExtensorToPosition;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot, copilot;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.PS4);
        copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

        copilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        copilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));
        copilot.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL),
                ButtonType.START, ButtonType.BACK);
        pilot.whileTrue(ButtonType.LB, new IntakeIn());
        pilot.whileTrue(ButtonType.RB, new IntakeOut());
        pilot.whileTrue(ButtonType.Y, new GripperIn());
        pilot.whileTrue(ButtonType.X, new GripperOut());
        // pilot.onTrue(ButtonType.A, new
        // DrivetrainSetTransmission(TransmissionMode.speed));
        // pilot.onTrue(ButtonType.B, new
        // DrivetrainSetTransmission(TransmissionMode.torque));
        pilot.onTrue(ButtonType.BACK, new SetIntakePosition(IntakePosition.STORED));
        // pilot.onTrue(ButtonType.START, new
        // SetIntakePosition(IntakePosition.LOWERED));
        pilot.onTrue(ButtonType.START, new ToggleGamePieceMode());

        copilot.onTrue(ButtonType.POV_N, new ResetTransportEncoders());
        copilot.onTrue(ButtonType.POV_S, new ToggleShouldManualBeLimited());

        // MIDDLE ROW CONE
        // copilot.whileTrue(ButtonType.A, new DriveWristToPosition(407, 0.4125));
        // copilot.whileTrue(ButtonType.B, new DriveArmToPosition(198, 0.875));
        // copilot.whileTrue(ButtonType.Y, new DriveExtensorToPosition(0.2, 1));

        // TOP ROW CONE (JALA)
        // copilot.whileTrue(ButtonType.A, new DriveWristToPosition(402, 0.4125));
        // copilot.whileTrue(ButtonType.B, new DriveArmToPosition(177, 0.875));
        // copilot.whileTrue(ButtonType.Y, new DriveExtensorToPosition(0.47, 1));

        // TODO: Habilitar comandos cuando sea posible
        // pilot.onTrue(ButtonType.A, new ToggleIntakePosition());
        // pilot.onTrue(ButtonType.LB, new IntakeIn());
        // pilot.onTrue(ButtonType.RB, new IntakeOut());

        pilot.whileTrue(ButtonType.A, new TransportGoTo(TransportTarget.MIDDLE_ROW));
        pilot.whileTrue(ButtonType.B, new DriveExtensorToPosition(0.47, 1));

        // copilot.onTrue(ButtonType.X, new TransportGoTo(TransportTarget.MIDDLE_ROW));
        // copilot.onTrue(ButtonType.Y, new TransportGoTo(TransportTarget.TOP_ROW));
        // copilot.onTrue(ButtonType.B, new TransportGoTo(TransportTarget.INTAKE));
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
        return -pilot.getAxis(AxisType.LEFT_Y);
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
