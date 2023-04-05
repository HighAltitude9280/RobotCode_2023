// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.pieceHandlers.gripper.GripperIn;
import frc.robot.commands.pieceHandlers.gripper.GripperOut;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.arm.DriveArmToPosition;
import frc.robot.commands.transport.compound.NewTransportGoTo;

/** Add your docs here. */
public class SwerveAutos {
        public static Command exampleAuto;

        public static Command abcAuto;
        public static Command abAuto;
        public static Command abChargingAuto;
        public static Command ihgAuto;
        public static Command ihAuto;
        public static Command ihChargingAuto;
        public static Command baAuto;
        public static Command acAuto;

        public static void generateAutos() {
                /////// Example auto
                List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup("ExamplePath",
                                new PathConstraints(1.5, 1.0));

                ////// PATHS

                List<PathPlannerTrajectory> abcPath = PathPlanner.loadPathGroup("ABC",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> abPath = PathPlanner.loadPathGroup("AB",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> abChargingPath = PathPlanner.loadPathGroup("AB Charging",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> ihgPath = PathPlanner.loadPathGroup("IHG",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> ihPath = PathPlanner.loadPathGroup("IH",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> ihChargingPath = PathPlanner.loadPathGroup("IH Charging",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> baPath = PathPlanner.loadPathGroup("BA",
                                new PathConstraints(1.5, 1.0));

                List<PathPlannerTrajectory> acPath = PathPlanner.loadPathGroup("AC",
                                new PathConstraints(1.5, 1.0));

                HashMap<String, Command> eventMap = new HashMap<>();
                eventMap.put("arrive", new PrintCommand("Ahyes placing game pieces"));

                // Cone
                eventMap.put("SetConeMode", new SetGamePieceMode(GamePieceMode.CONE));
                eventMap.put("ConeTop", new NewTransportGoTo(TransportTarget.TOP_ROW_BACK));
                eventMap.put("ConeMid", new PrintCommand("Cone Mid"));
                eventMap.put("ConeIntake", new NewTransportGoTo(TransportTarget.INTAKE));
                // Cube
                eventMap.put("SetCubeMode", new SetGamePieceMode(GamePieceMode.CUBE));
                eventMap.put("CubeTop", new NewTransportGoTo(TransportTarget.TOP_ROW_BACK));
                eventMap.put("CubeMid", new PrintCommand("Cube Mid"));
                eventMap.put("CubeIntake", new NewTransportGoTo(TransportTarget.INTAKE));
                // Gripper
                eventMap.put("GripperIn", new GripperIn().withTimeout(0.75));
                eventMap.put("GripperOut", new GripperOut().withTimeout(0.75));
                eventMap.put("GripperOff", new PrintCommand("Gripper Off"));
                // Charging
                eventMap.put("Charging", new PrintCommand("Balancing"));

                SwerveAutoBuilder autoBuilder = autoBuilder(
                                eventMap,
                                new PIDConstants(6.25, 0.0, 0.0), // translation constants
                                new PIDConstants(0.5, 0.0, 0.0), // rotation constants
                                true);

                // exampleAuto = autoBuilder.fullAuto(examplePath);
                //
                // abcAuto = autoBuilder.fullAuto(abcPath);
                // abAuto = autoBuilder.fullAuto(abPath);
                // abChargingAuto = autoBuilder.fullAuto(abChargingPath);
                //
                // ihgAuto = autoBuilder.fullAuto(ihgPath);
                // ihAuto = autoBuilder.fullAuto(ihPath);
                // ihChargingAuto = autoBuilder.fullAuto(ihChargingPath);
                //
                baAuto = autoBuilder.fullAuto(baPath);
                acAuto = autoBuilder.fullAuto(acPath);
        }

        /**
         * Returns the object of Type SwerveAutoBuilder only taking in the four
         * parameters that don't depend on Suppliers and consumers that are constant and
         * depend on Robot.getRobotContainer().getSwerveDriveTrain() in order to avoid
         * repetition.
         * 
         * @param translConstants
         * @param rotationConstants
         * @param useAllianceColor
         * @return
         */

        private static SwerveAutoBuilder autoBuilder(
                        Map<String, Command> eventMap,
                        PIDConstants translConstants,
                        PIDConstants rotationConstants,
                        boolean useAllianceColor) {
                return new SwerveAutoBuilder(
                                Robot.getRobotContainer().getSwerveDriveTrain()::getPose,
                                Robot.getRobotContainer().getSwerveDriveTrain()::resetPose,
                                HighAltitudeConstants.SWERVE_KINEMATICS,
                                translConstants, // translation constants
                                rotationConstants, // rotation constants
                                Robot.getRobotContainer().getSwerveDriveTrain()::setModuleStates,
                                eventMap,
                                useAllianceColor,
                                Robot.getRobotContainer().getSwerveDriveTrain());
        }
}
