package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.drivetrain.drivingParameters.drivingModes.DrivetrainToggleDrivingMode;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;
import frc.robot.subsystems.chassis.DriveTrain.DrivingMode;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot, copilot;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.PS4);
        copilot = new HighAltitudeJoystick(1, JoystickType.UNKNOWN);

        pilot.setAxisDeadzone(AxisType.LEFT_X, 0.09);
        pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.09);
        pilot.setAxisDeadzone(AxisType.LEFT_TRIGGER, 0.2);

        pilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        pilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));

        pilot.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL), ButtonType.START, ButtonType.BACK);

        // pilot.onTrue(ButtonType.B, new ResetTransportEncoders());

        // pilot.onTrue(ButtonType.Y, new ToggleGamePieceMode());

        // pilot.onTrue(ButtonType.X, new DrivetrainToggleDragonflySolenoid());

        pilot.onTrue(ButtonType.Y, new DrivetrainToggleDrivingMode(DrivingMode.Mecanum));
        pilot.onTrue(ButtonType.X, new DrivetrainToggleDrivingMode(DrivingMode.Swerve));

        // pilot.whileTrue(ButtonType.B, new FollowAprilTag());

        pilot.whileTrue(ButtonType.LB, new IntakeIn());
        pilot.whileTrue(ButtonType.RB, new IntakeOut());

        // pilot.toggleOnTrue(ButtonType.A, new FollowAprilTag());
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public HighAltitudeJoystick getPilot() {
        return pilot;
    }

    public HighAltitudeJoystick getCopilot() {
        return copilot;
    }

}
