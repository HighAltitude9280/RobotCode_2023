package frc.robot;

import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.gripper.GripperIn;
import frc.robot.commands.gripper.GripperOut;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot, copilot;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
        copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

        pilot.setAxisDeadzone(AxisType.RIGHT_Y, 0.05);

        pilot.onTrue(ButtonType.START, new SetGamePieceMode(GamePieceMode.CONE));
        pilot.onTrue(ButtonType.BACK, new SetGamePieceMode(GamePieceMode.CUBE));

        pilot.whileTrue(ButtonType.LB, new GripperIn());
        pilot.whileTrue(ButtonType.RB, new GripperOut());

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
