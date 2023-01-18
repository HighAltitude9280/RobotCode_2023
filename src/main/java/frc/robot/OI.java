package frc.robot;

import frc.robot.commands.FollowAprilTag;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

public class OI {

    public static OI instance;

    private HighAltitudeJoystick pilot, copilot;

    public void ConfigureButtonBindings() {
        pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
        copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

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
