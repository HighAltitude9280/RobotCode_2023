// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2;

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
import frc.robot.commands.autonomousV2.superSimpleAutos.LeavePiece;
import frc.robot.commands.pieceHandlers.gripper.GripperHold;
import frc.robot.commands.pieceHandlers.gripper.GripperInDontHold;
import frc.robot.commands.pieceHandlers.gripper.GripperOut;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.NewTransportGoTo;

/** Add your docs here. */
public class SwerveAutos2 {

        public static Command testStraightAuto;
        public static Command twoPieceTopeAuto;
        public static Command twoPieceFreeAuto;
        public static Command cubeMidFwdAuto;
        public static Command coneMidFwdAuto;
        public static Command mobilityAuto;

        public static void generateAutos() {

                List<PathPlannerTrajectory> testStraight = PathPlanner.loadPathGroup("TestStraight",
                                new PathConstraints(1.0, 0.875));
                List<PathPlannerTrajectory> twoPieceTope = PathPlanner.loadPathGroup("TwoPieceTope",
                                new PathConstraints(1.0, 1.5));
                List<PathPlannerTrajectory> twoPieceFree = PathPlanner.loadPathGroup("TwoPieceFree",
                                new PathConstraints(1.0, 1.25));
                List<PathPlannerTrajectory> cubeMidFwd = PathPlanner.loadPathGroup("CubeMidAndDrive",
                                new PathConstraints(1.0, 0.875));
                List<PathPlannerTrajectory> coneMidFwd = PathPlanner.loadPathGroup("ConeMidAndDrive",
                                new PathConstraints(1.0, 0.875));
                List<PathPlannerTrajectory> mobility = PathPlanner.loadPathGroup("MobilityBonus",
                                new PathConstraints(1.5, 1.25));

                HashMap<String, Command> eventMap = new HashMap<>();
                eventMap.put("arrive", new PrintCommand("Ahyes placing game pieces"));

                // Cone
                eventMap.put("SetConeMode", new SetGamePieceMode(GamePieceMode.CONE));
                // Cube
                eventMap.put("SetCubeMode", new SetGamePieceMode(GamePieceMode.CUBE));

                // Transport Go To
                eventMap.put("GoTopBack", new NewTransportGoTo(TransportTarget.TOP_ROW_BACK));
                eventMap.put("GoMiddleBack", new NewTransportGoTo(TransportTarget.MIDDLE_ROW_BACK));
                eventMap.put("GoMiddleFront", new NewTransportGoTo(TransportTarget.MIDDLE_ROW_FRONT));
                eventMap.put("GoIntake", new NewTransportGoTo(TransportTarget.INTAKE));
                eventMap.put("GoRest", new NewTransportGoTo(TransportTarget.RESTING));

                // Gripper
                eventMap.put("GripperIn", new GripperInDontHold().withTimeout(3.0));
                eventMap.put("GripperHold", new GripperHold());
                eventMap.put("GripperOut", new GripperOut().withTimeout(0.75));
                eventMap.put("GripperOff", new PrintCommand("Gripper Off"));

                // Full piece auto
                eventMap.put("LeaveCubeMid", new LeavePiece(GamePieceMode.CUBE, TransportTarget.MIDDLE_ROW_BACK));
                eventMap.put("LeaveConeMid", new LeavePiece(GamePieceMode.CONE, TransportTarget.MIDDLE_ROW_BACK));

                testStraightAuto = getFullAuto(
                                testStraight,
                                eventMap,
                                new PIDConstants(1.0, 0, 0),
                                new PIDConstants(0.1, 0, 0),
                                true);

                twoPieceTopeAuto = getFullAuto(
                                twoPieceTope,
                                eventMap,
                                new PIDConstants(1.125, 0, 0),
                                new PIDConstants(0.625, 0, 0),
                                true);
                twoPieceFreeAuto = getFullAuto(
                                twoPieceFree,
                                eventMap,
                                new PIDConstants(1.75, 0, 0),
                                new PIDConstants(0.8, 0, 0.08),
                                true);

                cubeMidFwdAuto = getFullAuto(
                                cubeMidFwd,
                                eventMap,
                                new PIDConstants(0.0, 0.0, 0.0),
                                new PIDConstants(0.0, 0.0, 0.0),
                                true);
                coneMidFwdAuto = getFullAuto(
                                coneMidFwd,
                                eventMap,
                                new PIDConstants(0.0, 0.0, 0.0),
                                new PIDConstants(0.0, 0.0, 0.0),
                                true);
                mobilityAuto = getFullAuto(
                                mobility,
                                eventMap,
                                new PIDConstants(0.0, 0.0, 0.0),
                                new PIDConstants(0.5, 0.0, 0.0),
                                true);
        }

        private static Command getFullAuto(List<PathPlannerTrajectory> trajectory,
                        Map<String, Command> eventMap,
                        PIDConstants translationConstants,
                        PIDConstants rotationConstants,
                        boolean useAllianceColor) {
                SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                                Robot.getRobotContainer().getSwerveDriveTrain()::getPose,
                                Robot.getRobotContainer().getSwerveDriveTrain()::resetPose,
                                HighAltitudeConstants.SWERVE_KINEMATICS,
                                translationConstants, // translation constants
                                rotationConstants, // rotation constants
                                Robot.getRobotContainer().getSwerveDriveTrain()::setModuleStates,
                                eventMap,
                                useAllianceColor,
                                Robot.getRobotContainer().getSwerveDriveTrain());
                return swerveAutoBuilder.fullAuto(trajectory);
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
         * 
         * 
         *         private static SwerveAutoBuilder autoBuilder(
         *         Map<String, Command> eventMap,
         *         PIDConstants translConstants,
         *         PIDConstants rotationConstants,
         *         boolean useAllianceColor) {
         *         return new SwerveAutoBuilder(
         *         Robot.getRobotContainer().getSwerveDriveTrain()::getPose,
         *         Robot.getRobotContainer().getSwerveDriveTrain()::resetPose,
         *         HighAltitudeConstants.SWERVE_KINEMATICS,
         *         translConstants, // translation constants
         *         rotationConstants, // rotation constants
         *         Robot.getRobotContainer().getSwerveDriveTrain()::setModuleStates,
         *         eventMap,
         *         useAllianceColor,
         *         Robot.getRobotContainer().getSwerveDriveTrain());
         *         }
         */
}
