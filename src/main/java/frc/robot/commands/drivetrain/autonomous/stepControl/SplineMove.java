package frc.robot.commands.drivetrain.autonomous.stepControl;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.resources.math.Math;
import frc.robot.resources.math.splines.CubicSpline;

public class SplineMove extends CommandBase {

    CubicSpline spline;
    double maxSpeed;
    boolean speedReduction, vertical, inverted, reverse;

    /**
     * Creates a new autonomous command in which the robot will follow the given
     * trajectory.
     *
     * @param spline         The spline that defines the trajectory that this
     *                       command will follow.
     * @param maxSpeed       The maximum speed that will be given to the robot at
     *                       any given time.
     * @param speedReduction True if the robot needs to stop at the end of the
     *                       command. False is useful when concatenating paths.
     * @param vertical       The default axis settings are: x-axis is paralell to
     *                       the guardrails, y-axis is paralel to the driver
     *                       stations. True if these settings need to be inverted.
     * @param inverted       By default, the trajectory is folllowed from left to
     *                       right. True if this setting needs to be inverted.
     * @param reverse        By default, the robot will follow the trajectory going
     *                       forward. True if the robot needs to follow the path
     *                       backwards.
     */
    public SplineMove(CubicSpline spline, double maxSpeed, boolean speedReduction, boolean vertical,
            boolean inverted,
            boolean reverse) {

        addRequirements(Robot.getRobotContainer().getDriveTrain());

        this.spline = spline;
        this.maxSpeed = maxSpeed;
        this.speedReduction = speedReduction;
        this.vertical = vertical;
        this.inverted = inverted;
        this.reverse = reverse;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        System.out.println("Endend spline move");
        Robot.getRobotContainer().getDriveTrain().stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        DifferentialDriveOdometry odometry = Robot.getRobotContainer().getDriveTrain().getOdometry();

        // The error in the Y axis (or x if vertical)
        double error = 0;
        double currentSlope = 0;
        double targetAngle = 0;

        if (vertical) {
            currentSlope = spline.fPrime(odometry.getPoseMeters().getY());
            error = odometry.getPoseMeters().getX() - spline.f(odometry.getPoseMeters().getY());
        } else {
            currentSlope = spline.fPrime(odometry.getPoseMeters().getX());
            error = odometry.getPoseMeters().getY() - spline.f(odometry.getPoseMeters().getX());
        }

        targetAngle = Math.angleFromSlope(currentSlope, vertical);

        if ((vertical || reverse || inverted) && !(vertical && reverse && inverted)) {
            targetAngle -= error * HighAltitudeConstants.SPLINE_DRIVE_ERROR_CORRECTION;
        } else {
            targetAngle += error * HighAltitudeConstants.SPLINE_DRIVE_ERROR_CORRECTION;
        }

        if ((inverted || reverse) && !(inverted && reverse)) {
            targetAngle = Math.getOppositeAngle(targetAngle);
        }

        double finalXPosition = spline.getFinalXPosition();

        if (inverted) {
            finalXPosition = spline.getInitialXPosition();
        }

        double finalYPosition = spline.f(finalXPosition);

        if (vertical) {
            double bubble = finalXPosition;
            finalXPosition = finalYPosition;
            finalYPosition = bubble;
        }

        double distanceToTarget = Math.distance(odometry.getPoseMeters().getX(), finalXPosition,
                odometry.getPoseMeters().getY(), finalYPosition);

        if (reverse)
            maxSpeed = -Math.abs(maxSpeed);

        double power = maxSpeed;

        if (speedReduction) {
            double speedReductionCorrection = Math
                    .clamp(distanceToTarget
                            / (HighAltitudeConstants.SPLINE_SPEED_REDUCTION_BRAKING_DISTANCE * maxSpeed), 0, 1);

            power = speedReductionCorrection * maxSpeed;
        }

        double deltaAngle = Math.deltaAngle(Robot.getRobotContainer().getNavx().getYaw(), targetAngle);
        double turnCorrectionPower = deltaAngle * HighAltitudeConstants.DRIVETRAIN_SPLINE_ANGLE_CORRECTION;

        Robot.getRobotContainer().getDriveTrain().arcadeDrive(power, turnCorrectionPower);

        return distanceToTarget < HighAltitudeConstants.DRIVETRAIN_SPLINE_ARRIVE_OFFSET;
    }

}
