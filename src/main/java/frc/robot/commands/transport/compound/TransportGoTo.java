// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.transport.arm.DriveArmToPosition;
import frc.robot.commands.transport.extensor.DriveExtensorToPosition;
import frc.robot.commands.transport.wrist.DriveWristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TransportGoTo extends SequentialCommandGroup {
  GamePieceMode currentGamePieceMode;
  double extensorTarget, armTarget, wristTarget;
  double extensorMaxPower, armMaxPower, wristMaxPower;

  TransportTarget target;

  public enum TransportTarget {
    TOP_ROW(HighAltitudeConstants.EXTENSOR_TOP_ROW_METERS_CONE, HighAltitudeConstants.ARM_TOP_ROW_DEGREES_CONE,
        HighAltitudeConstants.WRIST_TOP_ROW_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_TOP_ROW_METERS_CUBE,
        HighAltitudeConstants.ARM_TOP_ROW_DEGREES_CUBE, HighAltitudeConstants.WRIST_TOP_ROW_DEGREES_CUBE),

    MIDDLE_ROW(HighAltitudeConstants.EXTENSOR_MIDDLE_ROW_METERS_CONE, HighAltitudeConstants.ARM_MIDDLE_ROW_DEGREES_CONE,
        HighAltitudeConstants.WRIST_MIDDLE_ROW_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_MIDDLE_ROW_METERS_CUBE,
        HighAltitudeConstants.ARM_MIDDLE_ROW_DEGREES_CUBE, HighAltitudeConstants.WRIST_MIDDLE_ROW_DEGREES_CUBE),

    BOTTOM_ROW(HighAltitudeConstants.EXTENSOR_BOTTOM_ROW_METERS_CONE, HighAltitudeConstants.ARM_BOTTOM_ROW_DEGREES_CONE,
        HighAltitudeConstants.WRIST_BOTTOM_ROW_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_BOTTOM_ROW_METERS_CUBE,
        HighAltitudeConstants.ARM_BOTTOM_ROW_DEGREES_CUBE, HighAltitudeConstants.WRIST_BOTTOM_ROW_DEGREES_CUBE),

    INTAKE(HighAltitudeConstants.EXTENSOR_INTAKE_METERS_CONE, HighAltitudeConstants.ARM_INTAKE_DEGREES_CONE,
        HighAltitudeConstants.WRIST_INTAKE_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_INTAKE_METERS_CUBE,
        HighAltitudeConstants.ARM_INTAKE_DEGREES_CUBE, HighAltitudeConstants.WRIST_INTAKE_DEGREES_CUBE),

    FEEDER(HighAltitudeConstants.EXTENSOR_FEEDER_METERS_CONE, HighAltitudeConstants.ARM_FEEDER_DEGREES_CONE,
        HighAltitudeConstants.WRIST_FEEDER_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_FEEDER_METERS_CUBE,
        HighAltitudeConstants.ARM_FEEDER_DEGREES_CUBE, HighAltitudeConstants.WRIST_FEEDER_DEGREES_CUBE);

    private double extensorTargetCone, armTargetCone, wristTargetCone;
    private double extensorTargetCube, armTargetCube, wristTargetCube;

    private TransportTarget(double eCone, double aCone, double wCone,
        double eCube, double aCube, double wCube) {
      extensorTargetCone = eCone;
      armTargetCone = aCone;
      wristTargetCone = wCone;
      extensorTargetCube = eCube;
      armTargetCube = aCube;
      wristTargetCube = wCube;
    }

    public double getExtensorTargetCone() {
      return extensorTargetCone;
    }

    public double getArmTargetCone() {
      return armTargetCone;
    }

    public double getWristTargetCone() {
      return wristTargetCone;
    }

    public double getExtensorTargetCube() {
      return extensorTargetCube;
    }

    public double getArmTargetCube() {
      return armTargetCube;
    }

    public double getWristTargetCube() {
      return wristTargetCube;
    }
  }

  /** Creates a new TransportGoTo. */
  public TransportGoTo(TransportTarget target) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    wristMaxPower = HighAltitudeConstants.WRIST_AUTO_MAX_POWER;
    armMaxPower = HighAltitudeConstants.ARM_AUTO_MAX_POWER;
    extensorMaxPower = HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER;

    switch (currentGamePieceMode) {
      case CONE:
        wristTarget = target.getWristTargetCone();
        armTarget = target.getArmTargetCone();
        extensorTarget = target.getExtensorTargetCone();
        break;
      case CUBE:
        wristTarget = target.getWristTargetCube();
        armTarget = target.getArmTargetCube();
        extensorTarget = target.getExtensorTargetCube();
        break;
      case MANUAL:
        return;
      default:
        return;
    }

    addCommands(
        new DriveExtensorToPosition(extensorTarget, extensorMaxPower),
        new DriveArmToPosition(armTarget, armMaxPower),
        new DriveWristToPosition(wristTarget, wristMaxPower));
  }
}
