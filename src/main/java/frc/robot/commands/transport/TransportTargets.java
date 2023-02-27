package frc.robot.commands.transport;

import frc.robot.HighAltitudeConstants;

public class TransportTargets {
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

    private TransportTarget(double extensorTargetCone, double armTargetCone, double wristTargetCone,
        double extensorTargetCube, double armTargetCube, double wristTargetCube) {
      this.extensorTargetCone = extensorTargetCone;
      this.armTargetCone = armTargetCone;
      this.wristTargetCone = wristTargetCone;
      this.extensorTargetCube = extensorTargetCube;
      this.armTargetCube = armTargetCube;
      this.wristTargetCube = wristTargetCube;
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
}