package frc.robot.commands.transport;

import frc.robot.HighAltitudeConstants;

public class TransportTargets {
  public enum TransportTarget {
    TOP_ROW(HighAltitudeConstants.EXTENSOR_TOP_ROW_METERS_CONE, HighAltitudeConstants.ARM_TOP_ROW_DEGREES_CONE,
        HighAltitudeConstants.WRIST_TOP_ROW_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_TOP_ROW_METERS_CUBE,
        HighAltitudeConstants.ARM_TOP_ROW_DEGREES_CUBE, HighAltitudeConstants.WRIST_TOP_ROW_DEGREES_CUBE,
        HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER, HighAltitudeConstants.ARM_AUTO_MAX_POWER,
        HighAltitudeConstants.WRIST_AUTO_MAX_POWER),

    MIDDLE_ROW(HighAltitudeConstants.EXTENSOR_MIDDLE_ROW_METERS_CONE, HighAltitudeConstants.ARM_MIDDLE_ROW_DEGREES_CONE,
        HighAltitudeConstants.WRIST_MIDDLE_ROW_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_MIDDLE_ROW_METERS_CUBE,
        HighAltitudeConstants.ARM_MIDDLE_ROW_DEGREES_CUBE, HighAltitudeConstants.WRIST_MIDDLE_ROW_DEGREES_CUBE,
        HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER, HighAltitudeConstants.ARM_AUTO_MAX_POWER,
        HighAltitudeConstants.WRIST_AUTO_MAX_POWER),
    /*
     * BOTTOM_ROW(HighAltitudeConstants.EXTENSOR_BOTTOM_ROW_METERS_CONE,
     * HighAltitudeConstants.ARM_BOTTOM_ROW_DEGREES_CONE,
     * HighAltitudeConstants.WRIST_BOTTOM_ROW_DEGREES_CONE,
     * HighAltitudeConstants.EXTENSOR_BOTTOM_ROW_METERS_CUBE,
     * HighAltitudeConstants.ARM_BOTTOM_ROW_DEGREES_CUBE,
     * HighAltitudeConstants.WRIST_BOTTOM_ROW_DEGREES_CUBE,
     * HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER,
     * HighAltitudeConstants.ARM_AUTO_MAX_POWER,
     * HighAltitudeConstants.WRIST_AUTO_MAX_POWER),
     */
    RESTING(HighAltitudeConstants.EXTENSOR_REST_METERS_CONE, HighAltitudeConstants.ARM_REST_DEGREES_CONE,
        HighAltitudeConstants.WRIST_REST_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_REST_METERS_CUBE,
        HighAltitudeConstants.ARM_REST_DEGREES_CUBE, HighAltitudeConstants.WRIST_REST_DEGREES_CUBE,
        HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER, HighAltitudeConstants.ARM_AUTO_MAX_POWER,
        HighAltitudeConstants.WRIST_AUTO_MAX_POWER),
    /*
     * INTAKE(HighAltitudeConstants.EXTENSOR_INTAKE_METERS_CONE,
     * HighAltitudeConstants.ARM_INTAKE_DEGREES_CONE,
     * HighAltitudeConstants.WRIST_INTAKE_DEGREES_CONE,
     * HighAltitudeConstants.EXTENSOR_INTAKE_METERS_CUBE,
     * HighAltitudeConstants.ARM_INTAKE_DEGREES_CUBE,
     * HighAltitudeConstants.WRIST_INTAKE_DEGREES_CUBE,
     * HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER,
     * HighAltitudeConstants.ARM_AUTO_MAX_POWER,
     * HighAltitudeConstants.WRIST_AUTO_MAX_POWER),
     */

    FEEDER(HighAltitudeConstants.EXTENSOR_FEEDER_METERS_CONE, HighAltitudeConstants.ARM_FEEDER_DEGREES_CONE,
        HighAltitudeConstants.WRIST_FEEDER_DEGREES_CONE, HighAltitudeConstants.EXTENSOR_FEEDER_METERS_CUBE,
        HighAltitudeConstants.ARM_FEEDER_DEGREES_CUBE, HighAltitudeConstants.WRIST_FEEDER_DEGREES_CUBE,
        HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER, HighAltitudeConstants.ARM_AUTO_MAX_POWER,
        HighAltitudeConstants.WRIST_AUTO_MAX_POWER);

    private double extensorTargetCone, armTargetCone, wristTargetCone;
    private double extensorTargetCube, armTargetCube, wristTargetCube;
    private double extensorMaxPower, armMaxPower, wristMaxPower;

    private TransportTarget(double extensorTargetCone, double armTargetCone, double wristTargetCone,
        double extensorTargetCube, double armTargetCube, double wristTargetCube, double extensorMaxPower,
        double armMaxPower, double wristMaxPower) {
      this.extensorTargetCone = extensorTargetCone;
      this.armTargetCone = armTargetCone;
      this.wristTargetCone = wristTargetCone;
      this.extensorTargetCube = extensorTargetCube;
      this.armTargetCube = armTargetCube;
      this.wristTargetCube = wristTargetCube;
      this.extensorMaxPower = extensorMaxPower;
      this.armMaxPower = armMaxPower;
      this.wristMaxPower = wristMaxPower;
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

    public double getExtensorMaxPower() {
      return extensorMaxPower;
    }

    public double getArmMaxPower() {
      return armMaxPower;
    }

    public double getWristMaxPower() {
      return wristMaxPower;
    }
  }
}