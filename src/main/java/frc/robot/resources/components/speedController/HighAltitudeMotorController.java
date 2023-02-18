/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot.resources.components.speedController;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.*;
import frc.robot.Robot;

/**
 * Creates new HighAltitudeMotorController and allows a generalized
 * use of speed controllers. To see the currently supported
 * motor controllers you can check {@link TypeOfMotor}
 */
public class HighAltitudeMotorController {

    public enum TypeOfMotor {

        TALON_SRX, PWM_TALON_SRX, VICTOR, SPARK, CAN_SPARK_BRUSHLESS, CAN_SPARK_BRUSHED, JAGUAR, VICTOR_SPX,
        PWM_VICTOR_SPX, TALON_FX

    }

    private BaseMotorController phoenixMotor;

    private MotorController frcMotor;

    private TypeOfMotor motorToUse;

    private int port;

    public TypeOfMotor getType() {
        return motorToUse;
    }

    boolean inverted;

    public HighAltitudeMotorController(int port, TypeOfMotor m) {

        this.port = port;

        motorToUse = m;

        switch (motorToUse) {

            case TALON_SRX:

                phoenixMotor = new TalonSRX(port);

                phoenixMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

                break;

            case PWM_TALON_SRX:

                frcMotor = new PWMTalonSRX(port);

                break;

            case VICTOR:

                frcMotor = new Victor(port);

                break;

            case SPARK:

                frcMotor = new Spark(port);

                break;

            case JAGUAR:

                frcMotor = new Jaguar(port);

                break;

            case VICTOR_SPX:

                phoenixMotor = new VictorSPX(port);

                break;

            case PWM_VICTOR_SPX:

                frcMotor = new PWMVictorSPX(port);

                break;

            case CAN_SPARK_BRUSHLESS:

                frcMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

                getCANSparkMax().restoreFactoryDefaults();

                break;

            case CAN_SPARK_BRUSHED:

                frcMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushed);

                break;

            case TALON_FX:

                phoenixMotor = new TalonFX(port);

                phoenixMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

                break;

            default:

                DriverStation.reportError("Error configuring " + motorToUse + " with port " + port + ".", true);

        }

    }

    public void set(double speed) {

        if (phoenixMotor != null)
            phoenixMotor.set(ControlMode.PercentOutput, speed);
        if (frcMotor != null)
            frcMotor.set(speed);
        if (phoenixMotor == null && frcMotor == null)
            DriverStation.reportError("That type of motor doesn't exist!", true);

    }

    public double getEncPosition() {

        switch (motorToUse) {
            case TALON_SRX:
                return phoenixMotor.getSelectedSensorPosition(0);
            case TALON_FX:
                return phoenixMotor.getSelectedSensorPosition(0);
            case CAN_SPARK_BRUSHLESS:
                return ((CANSparkMax) frcMotor).getEncoder().getPosition();

            default:
                DriverStation.reportWarning("Encoder for " + motorToUse + " not found, returning 0!", true);
                return 0;
        }

    }

    public void stopMotor() {

        if (frcMotor != null)
            frcMotor.stopMotor();
        if (phoenixMotor != null)
            phoenixMotor.set(ControlMode.PercentOutput, 0);

    }

    public double getOutput() {

        if (phoenixMotor != null)
            return phoenixMotor.getMotorOutputPercent();
        if (frcMotor != null)
            return frcMotor.get();
        else
            DriverStation.reportError("Null motor", true);
        return 0;

    }

    public int getPort() {
        return port;
    }

    public void setEncoderPosition(int value) {

        if (motorToUse == TypeOfMotor.TALON_SRX || motorToUse == TypeOfMotor.TALON_FX) {
            ErrorCode a = phoenixMotor.setSelectedSensorPosition(value);
            ErrorCode b = phoenixMotor.setSelectedSensorPosition(value, 0, 0);

            Robot.debugPrint(a.toString());
            Robot.debugPrint(b.toString());
            return;
        }
        if (motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS) {
            ((CANSparkMax) frcMotor).getEncoder().setPosition(0);
            return;
        }
        DriverStation.reportWarning("Not a talonSRX, sensor position not updated", false);

    }

    public WPI_TalonSRX getTalon() {
        if (motorToUse == TypeOfMotor.TALON_SRX || motorToUse == TypeOfMotor.TALON_FX)
            return (WPI_TalonSRX) phoenixMotor;
        return null;

    }

    public CANSparkMax getCANSparkMax() {
        if (motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS || motorToUse == TypeOfMotor.CAN_SPARK_BRUSHED) {
            return (CANSparkMax) frcMotor;
        }
        return null;
    }

    /*
     * WARNING: this will only work with TALON SRX, FX and SPARK MAX
     *
     */
    public void setBrakeMode(boolean doBrake) {

        if (phoenixMotor != null)
            phoenixMotor.setNeutralMode(doBrake ? NeutralMode.Brake : NeutralMode.Coast);
        if (frcMotor != null)
            ((CANSparkMax) frcMotor).setIdleMode(doBrake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

    }

    public void setInverted(boolean i) {
        inverted = i;
        if (phoenixMotor != null)
            phoenixMotor.setInverted(i);
        if (frcMotor != null)
            frcMotor.setInverted(i);
    }

    public boolean isInverted() {
        return inverted;
    }

}