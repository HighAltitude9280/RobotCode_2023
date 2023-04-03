package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.drivetrain.swerve.swerveParameters.RecalculateWheelDirection;
import frc.robot.commands.drivetrain.swerve.swerveParameters.ToggleIsFieldOriented;
import frc.robot.commands.pieceHandlers.gripper.GripperIn;
import frc.robot.commands.pieceHandlers.gripper.GripperOut;
import frc.robot.commands.robotParameters.ResetNavx;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.robotParameters.ToggleShouldExtensorBeLimitedManual;
import frc.robot.commands.robotParameters.ToggleShouldManualHaveLimits;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.NewTransportGoTo;
import frc.robot.commands.transport.compound.ResetTransportEncoders;
import frc.robot.commands.transport.compound.TransportGoTo;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick subsystems; // SUBSYSTEMS
    private HighAltitudeJoystick chassis; // CHASSIS, IF HIGHALTITUDECONSTANTS.SINGLE_DRIVER IS FALSE

    // private HighAltitudeJoystick pit;

    public void ConfigureButtonBindings() {
        subsystems = new HighAltitudeJoystick(0, JoystickType.XBOX);
        chassis = new HighAltitudeJoystick(1, JoystickType.XBOX);

        // pit = new HighAltitudeJoystick(2, 12, 4); // Logitech Extreme 3D Pro

        subsystems.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        subsystems.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));
        subsystems.onTrueCombo(new SetGamePieceMode(GamePieceMode.MANUAL),
                ButtonType.START, ButtonType.BACK);

        subsystems.whileTrue(ButtonType.LB, new GripperIn());
        subsystems.whileTrue(ButtonType.RB, new GripperOut());
        subsystems.onTrueCombo(new ToggleShouldExtensorBeLimitedManual(),
                ButtonType.LB, ButtonType.RB);
        /*
         * subsystems.whileTrue(ButtonType.Y, new
         * TransportGoTo(TransportTarget.TOP_ROW_BACK));
         * subsystems.whileTrue(ButtonType.B, new
         * TransportGoTo(TransportTarget.FEEDER));
         * subsystems.whileTrue(ButtonType.A, new
         * TransportGoTo(TransportTarget.RESTING));
         * subsystems.whileTrue(ButtonType.X, new
         * TransportGoTo(TransportTarget.MIDDLE_ROW_BACK));
         * // pilot.onTrue(ButtonType.POV_N, new ToggleIntakePosition());
         */

        subsystems.whileTrueCombo(new NewTransportGoTo(TransportTarget.TOP_ROW_BACK), ButtonType.Y); // not configured
                                                                                                     // for cube
        subsystems.whileTrueCombo(new NewTransportGoTo(TransportTarget.FEEDER), ButtonType.B, ButtonType.POV_N);
        subsystems.whileTrueCombo(new NewTransportGoTo(TransportTarget.INTAKE), ButtonType.B, ButtonType.POV_S);
        subsystems.whileTrueCombo(new NewTransportGoTo(TransportTarget.RESTING), ButtonType.A);
        subsystems.whileTrueCombo(new NewTransportGoTo(TransportTarget.MIDDLE_ROW_BACK), ButtonType.X,
                ButtonType.POV_S);
        subsystems.whileTrueCombo(new NewTransportGoTo(TransportTarget.MIDDLE_ROW_FRONT), ButtonType.X,
                ButtonType.POV_N);

        if (HighAltitudeConstants.SINGLE_DRIVER) {
            chassis.onTrue(ButtonType.POV_N, new ResetTransportEncoders());
            chassis.onTrue(ButtonType.POV_S, new ToggleShouldManualHaveLimits());
            // pilot.onTrue(ButtonType.RS, new DrivetrainToggleTransmissionMode()); //
            // SINGLE DRIVER
            // pilot.whileTrue(ButtonType.JOYSTICK_R_X, new FollowTargetJolt()); // SINGLE
            // DRIVER
        }

        else {
            chassis.onTrue(ButtonType.POV_N, new ResetTransportEncoders());
            chassis.onTrue(ButtonType.START, new ResetNavx());
            chassis.onTrue(ButtonType.BACK, new RecalculateWheelDirection());
            chassis.onTrue(ButtonType.LB, new ToggleIsFieldOriented());

            // copilot.onTrue(ButtonType.A, new DrivetrainToggleTransmissionMode()); //
            // COPILOT
            // copilot.whileTrue(ButtonType.RT, new FollowTargetJolt()); // COPILOT
        }

        // pilot.onTrueCombo(new ResetOdometry(0, 0), ButtonType.RT, ButtonType.LT);
        // pit.getJoystickButtonObj(7).onTrue(new ResetTransportEncoders());
        // pit.getJoystickButtonObj(8).onTrue(new ResetNavx());
        // pit.getJoystickButtonObj(9).onTrue(new ResetOdometry(0, 0));
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getDefaultSwerveDriveSpeed() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return -subsystems.getAxis(AxisType.LEFT_Y);
        else {
            return -chassis.getAxis(AxisType.LEFT_Y);
        }
    }

    public double getDefaultSwerveDriveStrafe() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return subsystems.getAxis(AxisType.LEFT_X);
        else
            return -chassis.getAxis(AxisType.LEFT_X);
    }

    public double getDefaultSwerveDriveTurn() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return subsystems.getAxis(AxisType.RIGHT_X);
        else
            return -chassis.getAxis(AxisType.RIGHT_X);
    }

    public double getSwerveDriveAsTankTurn() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return subsystems.getAxis(AxisType.LEFT_X);
        else
            return chassis.getAxis(AxisType.LEFT_X);

    }

    public boolean getSwerveDriveFieldOriented() {
        return true;
    }

    public double getDefaultDriveX() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return subsystems.getAxis(AxisType.LEFT_X) * 0.75 * 0.75;
        else
            return chassis.getAxis(AxisType.LEFT_X) * 0.75 * 0.75;
    }

    public double getDefaultDriveY() {
        if (HighAltitudeConstants.SINGLE_DRIVER)
            return -subsystems.getAxis(AxisType.LEFT_Y) * 0.75;
        else
            return -chassis.getAxis(AxisType.LEFT_Y) * 0.75;
    }

    public double getDefaultDriveTurn() {
        return subsystems.getAxis(AxisType.RIGHT_X);
    }

    public double getDefaultDriveDragonfly() {
        return subsystems.getAxis(AxisType.RIGHT_X);
    }

    public double getWristInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? subsystems.getPovXAxis() * 0.125
                : subsystems.getPovXAxis() * 0.5;
    }

    public double getArmInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual()
                ? -subsystems.getAxis(AxisType.RIGHT_Y) * 0.25
                : -subsystems.getAxis(AxisType.RIGHT_Y) * 1.0;
    }

    public double getExtensorInput() {
        return Robot.getRobotContainer().getShouldExtensorBeSlowerInManual() ? subsystems.getTriggers() * 0.25
                : subsystems.getTriggers() * 0.75;
    }

    public HighAltitudeJoystick getSubsystems() {
        return subsystems;
    }

    public HighAltitudeJoystick getChassis() {
        return chassis;
    }

}