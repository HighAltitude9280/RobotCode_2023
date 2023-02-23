package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class DriveTrain extends SubsystemBase {
    public enum DrivingMode {
        Tank, Pivot, Mecanum, Swerve
    }

    public enum TransmissionMode {
        torque, speed
    }

    public enum WheelState {
        Lowered, Raised
    }

    HighAltitudeMotorGroup leftMotors, rightMotors, dragonflyMotors;

    DoubleSolenoid transmission;
    TransmissionMode transmissionState;

    DoubleSolenoid dragonflySolenoid;
    WheelState currentDragonflySolenoidState;

    DrivingMode currentDrivingMode = DrivingMode.Tank;

    // If driving with inverted axes;
    boolean reverse = false;
    // Mecanum and Swerve move require the robot to stay in the same angle (unless
    // turning) so hasSetAngle checks if the angle has been set.
    boolean hasSetAngle;
    // The angle the robot will keep during mecanum or swerve drive unless turning.
    double drivingAngle;

    // Odometry object that estimates robots position (only works in tank mode).
    DifferentialDriveOdometry odometry;

    // The position of the encoders in the last frame.
    double lastLeftEncoderPosition = 0;
    double lastRightEncoderPosition = 0;

    // The distance that each side has traveled.
    double leftEncoderDistance = 0;
    double rightEncoderDistance = 0;

    /**
     * Initializes the drivetrain subsystem. DO NOT FORGET TO RESET ODOMETRY
     * IMMEDIATELY AFTER
     * RUNNING THIS CONSTRUCTOR.
     */
    public DriveTrain() {
        if (RobotMap.DRIVETRAIN_TRANSMISSION_IS_AVAILABLE)
            transmission = new DoubleSolenoid(RobotMap.DRIVETRAIN_TRANSMISSION_MODULE_TYPE,
                    RobotMap.DRIVETRAIN_TRANSMISSION_SOLENOID_PORTS[0],
                    RobotMap.DRIVETRAIN_TRANSMISSION_SOLENOID_PORTS[1]);

        if (RobotMap.DRIVETRAIN_DRAGONFLY_SOLENOID_IS_AVAILABLE)
            dragonflySolenoid = new DoubleSolenoid(RobotMap.DRIVETRAIN_DRAGONFLY_SOLENOID_MODULE_TYPE,
                    RobotMap.DRIVETRAIN_DRAGONFLY_SOLENOID_PORTS[0],
                    RobotMap.DRIVETRAIN_DRAGONFLY_SOLENOID_PORTS[1]);

        if (RobotMap.DRIVETRAIN_LEFT_MOTOR_PORTS.length != RobotMap.DRIVETRAIN_RIGHT_MOTOR_PORTS.length)
            DriverStation.reportError("More motors in one side.", true);

        leftMotors = new HighAltitudeMotorGroup(RobotMap.DRIVETRAIN_LEFT_MOTOR_PORTS,
                RobotMap.DRIVETRAIN_LEFT_INVERTED_MOTORS_PORTS, RobotMap.DRIVETRAIN_LEFT_MOTOR_TYPES);

        rightMotors = new HighAltitudeMotorGroup(RobotMap.DRIVETRAIN_RIGHT_MOTOR_PORTS,
                RobotMap.DRIVETRAIN_RIGHT_INVERTED_MOTORS_PORTS, RobotMap.DRIVETRAIN_RIGHT_MOTOR_TYPES);

        leftMotors.setEncoderInverted(RobotMap.DRIVETRAIN_LEFT_ENCODER_IS_INVERTED);
        rightMotors.setEncoderInverted(RobotMap.DRIVETRAIN_RIGHT_ENCODER_IS_INVERTED);

        if (RobotMap.DRIVETRAIN_DRAGONFLY_WHEEL_IS_AVAILABLE) {
            dragonflyMotors = new HighAltitudeMotorGroup(RobotMap.DRIVETRAIN_DRAGONFLY_MOTOR_PORTS,
                    RobotMap.DRIVETRAIN_DRAGONFLY_INVERTED_MOTORS_PORTS, RobotMap.DRIVETRAIN_DRAGONFLY_MOTOR_TYPES);

            dragonflyMotors.setEncoderInverted(RobotMap.DRIVETRAIN_DRAGONFLY_ENCODER_IS_INVERTED);
        }

        setBrakeMode(HighAltitudeConstants.DRIVETRAIN_MOTORS_BRAKING_MODE);
        setTransmissionState(HighAltitudeConstants.DRIVETRAIN_INITIAL_TRANSMISSION_MODE);
        setDragonflySolenoid(HighAltitudeConstants.DRIVETRAIN_INITIAL_DRAGONFLY_STATE);
    }

    /**
     * Drives the robot handling driving modes.
     * 
     * @param x         The turn axis for tank or x-axis for mecanum/swerve drive.
     *                  (Normally left stick x-axis).
     * @param y         The speed axis for tank or y-axis for mecanum/swerve drive.
     *                  (Normally left stick y-axis).
     * @param turn      The turn axis for mecanum/swerve drive. (Normally the right
     *                  stick x-axis).
     * @param dragonfly The axis associated with the dragonfly module manual drive.
     *                  (Normally the right stick x-axis).
     */
    public void defaultDrive(double x, double y, double turn, double dragonfly) {
        double inversion = reverse ? -1 : 1;

        switch (currentDrivingMode) {
            case Tank:

                arcadeDrive(y * inversion, x);
                setDragonflyPower(dragonfly);
                break;

            case Mecanum:

                mecanumDrive(x * inversion, y * inversion, turn);
                break;

            case Swerve:
                swerveDrive(x, y, turn);
                break;

            case Pivot:
                pivotDriving(y * inversion, x);
                break;

            default:
                DriverStation.reportError("Driving mode not recognized, using tank mode", true);
                arcadeDrive(y * inversion, x);
                setDragonflyPower(dragonfly);
                break;
        }
    }

    // ASSISTED DRIVING

    /**
     * Moves the robot pivoting on the left or right side (decides based on the turn
     * parameter).
     * 
     * @param speed From -1 to 1, the value of the joystick asigned to forward
     *              motion.
     * @param turn  From -1 to 1, the value of the joystick asigned to turning.
     */
    public void pivotDriving(double speed, double turn) {
        hasSetAngle = false;

        if (turn < 0)
            tankDrive(-0.1, speed);
        else
            tankDrive(speed, -0.1);
    }

    /**
     * Controls a dragonfly drivetrain as if it were a mecanum drivetrain, that is
     * to say,
     * cartesian motion relative to the robot. No field orientated drive is
     * implemented in
     * this method. Note that this method requires a gyroscope to work.
     * 
     * @param x    The desired movement in the x axis relative to the robot, from -1
     *             to 1.
     * @param y    The desired movement in the y axis (forward), from -1 to 1.
     * @param turn The desired angular velocity of the robot, from -1 to 1.
     */
    public void mecanumDrive(double x, double y, double turn) {
        double currentAngle = Robot.getRobotContainer().getNavx().getYaw();

        if (!hasSetAngle) {
            drivingAngle = currentAngle;
            hasSetAngle = true;
            setDragonflySolenoid(WheelState.Lowered);
        }
        if (Math.abs(turn) > 0.1)
            drivingAngle = currentAngle;

        double deltaAngle = Math.deltaAngle(currentAngle, drivingAngle);

        double proportionalCorrection = deltaAngle * HighAltitudeConstants.DRIVETRAIN_DRAGONFLY_ANGLE_CORRECTION;
        double expectedCorrection = -x * HighAltitudeConstants.DRIVETRAIN_DRAGONFLY_EXPECTED_CORRECTION;
        double correction = proportionalCorrection + expectedCorrection;

        double desiredDirection = Math.atan(Math.abs(y / x));
        double minAngleAtMaxPower = Math.atan(1 / HighAltitudeConstants.DRIVETRAIN_DRAGONFLY_SIDES_CORRECTION);

        double xPower = 0, yPower = 0;

        if (desiredDirection < minAngleAtMaxPower) {
            xPower = x;
            yPower = Math.tan(desiredDirection) * Math.abs(xPower) * (y / Math.abs(y));
        } else {
            yPower = y;
            xPower = Math.abs(y) / Math.tan(desiredDirection) * x / Math.abs(x);
        }

        arcadeDrive(yPower, correction);
        setDragonflyPower(xPower);
    }

    /**
     * Drives a dragonfly drivetrain using field orientated drive like a
     * swerve drivetrain. It moves the drivetrain the specified amount in x, and in
     * y regardless of the
     * robot's orientation, where y is the axis perpendicular to the alliance wall
     * (or the 0° of the gyroscope) and x is the axis perpendicular to y.
     * 
     * @param x    The desired movement in the x axis, from -1 to 1.
     * @param y    The desired movement in the y axis, from -1 to 1.
     * @param turn The desired angular velocity of the robot, from -1 to 1.
     */
    public void swerveDrive(double x, double y, double turn) {
        double absoluteAngle = 0;

        // Prevents division by 0
        if (y != 0) {
            absoluteAngle = Math.toDegrees(Math.atan(x / y));
        } else {
            if (x > 0)
                absoluteAngle = 90;
            else
                absoluteAngle = -90;
        }

        // Handles the third and fouth quadrants
        if (y < 0) {
            absoluteAngle = Math.getOppositeAngle(absoluteAngle);
        }

        double relativeAngle = absoluteAngle - Robot.getRobotContainer().getNavx().getYaw();
        double speed = Math.hypot(x, y);

        driveToAngle(relativeAngle, turn, speed);

    }

    /**
     * Drives a dragonfly drivetrain towards a certain angle relative to the robot,
     * where 0° is forward motion, 90° motion to the right. Note that no field
     * orientated
     * drive is implemented with this method, however, a gyroscope is required for
     * it to work.
     * 
     * @param angle    The angle (in degrees) that the robot will move to, from -180
     *                 to 180
     * @param turn     The desired angular velocity of the robot, from -1 to 1.
     * @param maxPower The maximum power that will me given to the motors, from -1
     *                 to 1.
     */
    public void driveToAngle(double angle, double turn, double maxPower) {
        double x = Math.sin(Math.toRadians(angle)) * maxPower;
        double y = Math.cos(Math.toRadians(angle)) * maxPower;

        mecanumDrive(x, y, turn);
    }

    // MANUAL DRIVING

    /**
     * Default driving mode.
     * 
     * @param speed From -1 to 1, represents the linear speed of the robot.
     * @param turn  From -1 to 1, represents the angular velocity of the robot.
     */
    public void arcadeDrive(double speed, double turn) {
        double leftPower = speed - turn;
        double rightPower = speed + turn;

        tankDrive(leftPower, rightPower);
    }

    /**
     * Drives the robot, controlling each side separately.
     * 
     * @param leftPower  The power (from -1 to 1) that will be asigned to the left
     *                   side.
     * @param rightPower The power (from -1 to 1) that will be asigned to the right
     *                   side.
     */
    public void tankDrive(double leftPower, double rightPower) {
        leftMotors.setAll(leftPower);
        rightMotors.setAll(rightPower);
    }

    /**
     * Assigns the given power to the dragonfly motors.
     * 
     * @param power The power (from -1 to 1) that will be asigned to the dragonfly
     *              motors.
     */
    public void setDragonflyPower(double power) {
        if (RobotMap.DRIVETRAIN_DRAGONFLY_WHEEL_IS_AVAILABLE)
            dragonflyMotors.setAll(power);
    }

    /**
     * Assigns power to each of the motor groups independently.
     * 
     * @param leftPower      The power (from -1 to 1) that will be asigned to the
     *                       left side.
     * @param rightPower     The power (from -1 to 1) that will be asigned to the
     *                       right side.
     * @param dragonflyPower The power (from -1 to 1) that will be asigned to the
     *                       dragonfly motors.
     */
    public void manualDrive(double leftPower, double rightPower, double dragonflyPower) {
        tankDrive(leftPower, rightPower);
        setDragonflyPower(dragonflyPower);
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        manualDrive(0, 0, 0);
    }

    // AUTONOMOUS

    /**
     * Turns the robot (in its own axis) to a given gyroscope target, then stops.
     * This method
     * uses step control to turn the robot. In order to fine adjust the way that the
     * turning is
     * done, arrive offset and braking distance constants located at
     * {@link HighAltitudeConstants}
     * can be modified.
     * 
     * @param target   The desired gyroscope value (in degrees, from -180 to 180).
     * @param maxPower The maximum angular velocity of the robot (from 0 to 1).
     * @return True if the robot has arrived to the target, false otherwise.
     */
    public boolean turn(double target, double maxPower) {
        double deltaAngle = Math.deltaAngle(Robot.getRobotContainer().getNavx().getYaw(), target);

        if (Math.abs(deltaAngle) < HighAltitudeConstants.DRIVETRAIN_AUTO_TURNING_ARRIVE_OFFSET) {
            stop();
            return true;
        }

        maxPower = Math.clamp(Math.abs(target), 0, 1);

        double power = deltaAngle
                / (HighAltitudeConstants.DRIVETRAIN_AUTO_TURNING_BRAKING_DISTANCE * maxPower * maxPower);

        power = Math.clamp(power, 0, 1) * maxPower;

        arcadeDrive(0, power);
        return false;

    }

    /**
     * Drives in a straight line to a certain left encoder traveled distance, uses
     * step control
     * to improve accuracy. In order to fine adjust the way that the movement is
     * done, arrive offset and braking distance constants located at
     * {@link HighAltitudeConstants}
     * can be modified. Use {@link #getLeftEncoderDistance()} to know the inital
     * position and to calculate
     * the target.
     * 
     * @param target   The left encoder distance target (in meters) to which the
     *                 robot will move.
     * @param maxPower The maximum speed of the robot (from 0 to 1).
     * @return True if the robot is on target, false otherwise.
     */
    public boolean moveStraight(double target, double maxPower) {
        double delta = target - leftEncoderDistance;

        if (Math.abs(delta) < HighAltitudeConstants.DRIVETRAIN_AUTO_STRAIGHT_ARRIVE_OFFSET) {
            stop();
            return true;
        }

        double power = delta / (HighAltitudeConstants.DRIVETRAIN_AUTO_STRAIGHT_BRAKING_DISTANCE * maxPower * maxPower);
        power = Math.clamp(power, -1, 1) * maxPower;

        arcadeDrive(power, 0);
        return false;
    }

    /**
     * Drives in a straight line to a certain left encoder traveled distance while
     * keeping a certain
     * angle, uses step control to improve accuracy. In order to fine adjust the way
     * that the movement
     * is done, arrive offset, braking distance, and angle correction constants
     * located at
     * {@link HighAltitudeConstants} can be modified. Use
     * {@link #getLeftEncoderDistance()} to know the
     * inital position and to calculate the target.
     *
     * @param target      The left encoder distance target (in meters) to which the
     *                    robot will move.
     * @param maxPower    The maximum speed of the robot (from 0 to 1).
     * @param angleToKeep The angle to keep while moving in order to assure that the
     *                    movement will be straight
     *
     * @return True if the robot is on target, false otherwise.
     */
    public boolean moveStraight(double target, double maxPower, double angleToKeep) {
        double delta = target - leftEncoderDistance;
        double deltaAngle = Math.deltaAngle(Robot.getRobotContainer().getNavx().getYaw(), angleToKeep);

        if (Math.abs(delta) < HighAltitudeConstants.DRIVETRAIN_AUTO_STRAIGHT_ARRIVE_OFFSET) {
            stop();
            return true;
        }

        double power = delta / (HighAltitudeConstants.DRIVETRAIN_AUTO_STRAIGHT_BRAKING_DISTANCE * maxPower * maxPower);
        power = Math.clamp(power, 0, 1) * maxPower;

        double turnCorrectionPower = deltaAngle * HighAltitudeConstants.DRIVETRAIN_AUTO_STRAIGHT_ANGLE_CORRECTION;

        arcadeDrive(power, turnCorrectionPower);
        return false;
    }

    // GETTERS AND SETTERS

    //// Driving parameters

    // Solenoid

    /**
     * Raises or lowers the dragonfly wheel.
     * 
     * @param state The desired state of the dragonfly module, either raised or
     *              lowered.
     */
    public void setDragonflySolenoid(WheelState state) {
        if (!RobotMap.DRIVETRAIN_DRAGONFLY_SOLENOID_IS_AVAILABLE)
            return;

        if (state == WheelState.Lowered) {
            dragonflySolenoid.set(RobotMap.DRIVETRAIN_DRAGONFLY_LOWERED);
            currentDragonflySolenoidState = WheelState.Lowered;
            return;
        }

        dragonflySolenoid.set(RobotMap.DRIVETRAIN_DRAGONFLY_RAISED);
        currentDragonflySolenoidState = WheelState.Raised;

    }

    /**
     * @return The current state of the dragonfly solenoid (raised or lowered)
     */
    public WheelState getDragonflySolenoidState() {
        return currentDragonflySolenoidState;
    }

    /**
     * Toggles the dragonfly solenoid (lowered/raised)
     */
    public void toggleDragonflySolenoid() {
        if (currentDragonflySolenoidState == WheelState.Raised) {
            setDragonflySolenoid(WheelState.Lowered);
            return;
        }
        setDragonflySolenoid(WheelState.Raised);
    }

    // Driving modes

    /**
     * Changes the current driving mode to the given one.
     * 
     * @param mode Desired driving mode
     */
    public void setDrivingMode(DrivingMode mode) {
        currentDrivingMode = mode;
        hasSetAngle = false;
    }

    /**
     * @return Current driving mode.
     */
    public DrivingMode getCurrentDrivingMode() {
        return currentDrivingMode;
    }

    /**
     * Sets the driving mode to the given mode if the current driving mode is
     * different
     * from the given one, otherwise sets the driving mode to tank.
     * 
     * @param mode The driving mode to toggle.
     */
    public void toggleDrivingMode(DrivingMode mode) {
        if (getCurrentDrivingMode() == mode) {
            setDrivingMode(DrivingMode.Tank);
        } else {
            setDrivingMode(mode);
        }
    }

    // Transmission

    /**
     * Sets the transmission to a given state (torque or speed).
     * 
     * @param state The desired transmission state (torque or speed).
     */
    public void setTransmissionState(TransmissionMode state) {

        if (!RobotMap.DRIVETRAIN_TRANSMISSION_IS_AVAILABLE)
            return;

        transmissionState = state;
        if (state == TransmissionMode.torque)
            transmission.set(RobotMap.DRIVETRAIN_TRANSMISSION_TORQUE);

        else
            transmission.set(RobotMap.DRIVETRAIN_TRANSMISSION_SPEED);

    }

    /**
     * Toggles the transmission state (torque/speed).
     */
    public void toggleTransmission() {
        if (transmissionState == TransmissionMode.speed)
            setTransmissionState(TransmissionMode.torque);

        else
            setTransmissionState(TransmissionMode.speed);
    }

    // Reverse

    /**
     * Toggles the orientation of the robot (reverse/default).
     */
    public void changeOrientation() {
        setOrientation(!getOrientation());
    }

    /**
     * Changes the orientation of the robot, when driving in reverse, the back of
     * the
     * robot will become the front.
     * 
     * @param reverse True to drive in reverse, false to use default.
     */
    public void setOrientation(boolean reverse) {
        this.reverse = reverse;
    }

    /**
     * @return True if driving in reverse.
     */
    public boolean getOrientation() {
        return reverse;
    }

    // Brake mode

    /**
     * Sets the braking mode of the motors.
     * 
     * @param doBrake True for brake, false for coast.
     */
    public void setBrakeMode(boolean doBrake) {
        leftMotors.setBrakeMode(doBrake);
        rightMotors.setBrakeMode(doBrake);
        if (RobotMap.DRIVETRAIN_DRAGONFLY_WHEEL_IS_AVAILABLE)
            dragonflyMotors.setBrakeMode(doBrake);
    }

    //// Encoders and odometry

    /**
     * This method resets odometry and sets the robot position to (0,0).
     */
    public void resetOdometry() {
        resetOdometry(0, 0);
    }

    /**
     * Resets the robot odometry, sets the new position to the given x and y (in
     * meters) and the
     * current robot rotation (reported by the Navx). Note that this method also
     * resets the encoders.
     * </p>
     * 
     * <strong> Do not reset encoders or odometry manually, use only this method to
     * reset either.</strong>
     * 
     * @param x The desired new x position in meters.
     * @param y The desired new x position in meters.
     */
    public void resetOdometry(double x, double y) {
        Rotation2d currentRotation = new Rotation2d(Robot.getRobotContainer().getNavx().getYaw());

        odometry = new DifferentialDriveOdometry(currentRotation, x, y);
        resetEncoders();
    }

    /**
     * DO NOT USE THIS METHOD TO RESET ENCODERS, USE {@link #resetOdometry()}
     * INSTEAD.
     * Encoders should never be reset without also resetting odometry, since the
     * odometry
     * will misscalculate robot position.
     * </p>
     * This method resets all three drivetrain encoders.
     */
    private void resetEncoders() {
        leftEncoderDistance = 0;
        rightEncoderDistance = 0;

        rightMotors.resetEncoder();
        leftMotors.resetEncoder();
        if (RobotMap.DRIVETRAIN_DRAGONFLY_WHEEL_IS_AVAILABLE)
            dragonflyMotors.resetEncoder();
    }

    /**
     * @return The left encoder position in meters.
     */
    public double getLeftEncoderDistance() {
        return leftEncoderDistance;
    }

    /**
     * @return The right encoder position in meters.
     */
    public double getRightEncoderDistance() {
        return rightEncoderDistance;
    }

    /**
     * @return The differential drive odometry object.
     */
    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    /**
     * DO NOT FORGET TO CALL THIS METHOD ON {@link #periodic()}
     * Updates odometry and encoder positions.
     */
    public void updateOdometry() {
        double deltaLeft = lastLeftEncoderPosition - leftMotors.getEncoderPosition();
        double deltaRight = lastRightEncoderPosition - rightMotors.getEncoderPosition();

        if (transmissionState == TransmissionMode.torque) {

            leftEncoderDistance -= deltaLeft * HighAltitudeConstants.DRIVETRAIN_METERS_PER_PULSE_TORQUE;
            rightEncoderDistance += deltaRight * HighAltitudeConstants.DRIVETRAIN_METERS_PER_PULSE_TORQUE;

        } else {

            leftEncoderDistance -= deltaLeft * HighAltitudeConstants.DRIVETRAIN_METERS_PER_PULSE_SPEED;
            rightEncoderDistance += deltaRight * HighAltitudeConstants.DRIVETRAIN_METERS_PER_PULSE_SPEED;

        }

        lastLeftEncoderPosition = leftMotors.getEncoderPosition();
        lastRightEncoderPosition = rightMotors.getEncoderPosition();

        if (odometry != null) {
            odometry.update(new Rotation2d(Math.toRadians(Robot.getRobotContainer().getNavx().getYaw())),
                    leftEncoderDistance, rightEncoderDistance);

        } else {
            resetOdometry();
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
