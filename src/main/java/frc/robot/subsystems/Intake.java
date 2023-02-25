package frc.robot.subsystems;

import com.alumiboti5590.util.properties.RobotProperty;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.math.BigDecimal;
import java.util.HashMap;

public class Intake extends SubsystemBase {

  // Matches to a specific speed of the intake based on our testing
  public enum VelocityMode {
    CONE_INTAKE,
    CUBE_INTAKE,
    REVERSE,
    ZERO
  }

  private WPI_TalonSRX intakeMotor;

  // Velocity mode is used by setIntakeVelocity to use tested, appropriate
  // values when setting the motor velocity controls.
  private HashMap<VelocityMode, Double> velocityModeMap;

  public Intake() {
    this.intakeMotor = new WPI_TalonSRX(RobotProperty.INTAKE_MOTOR_ID.getInteger());
    this.intakeMotor.setNeutralMode(NeutralMode.Brake);
    this.intakeMotor.setInverted(RobotProperty.INTAKE_MOTOR_INVERT.getBoolean());

    this.configureClosedLoopEncoder();

    // Velocity map configurations below
    velocityModeMap = new HashMap<VelocityMode, Double>();
    velocityModeMap.put(
        VelocityMode.CONE_INTAKE,
        determineRPMVelocityTarget(Constants.Intake.VelocityRPM.CONE_INTAKE));
    velocityModeMap.put(
        VelocityMode.CUBE_INTAKE,
        determineRPMVelocityTarget(Constants.Intake.VelocityRPM.CUBE_INTAKE));
    velocityModeMap.put(
        VelocityMode.REVERSE, determineRPMVelocityTarget(Constants.Intake.VelocityRPM.REVERSE));
    velocityModeMap.put(VelocityMode.ZERO, BigDecimal.ZERO.doubleValue());
  }

  /**
   * Directly sets the motor speed based on a percentage of [-1.0, 1.0] where positive numbers are
   * intake and negative numbers are exhausting
   */
  public void setIntakeSpeed(double speed) {
    this.intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /** Set the desired motor velocity using one of the approved & verified VelocityMode values. */
  public void setIntakeVelocity(VelocityMode mode) {
    double desiredVelocity = this.velocityModeMap.get(mode);
    this.intakeMotor.set(ControlMode.Velocity, desiredVelocity);
  }

  /** Move all of the encoder configuration into a nice, clean class */
  private void configureClosedLoopEncoder() {
    this.intakeMotor.configSelectedFeedbackSensor(
        Constants.Intake.Encoder.ENCODER_TYPE,
        Constants.Intake.Encoder.SLOT_ID,
        Constants.Intake.Encoder.TIMEOUT_MS);

    this.intakeMotor.setSensorPhase(Constants.Intake.Encoder.INVERT_PHASE);

    // Config the peak and nominal outputs
    this.intakeMotor.configNominalOutputForward(0, Constants.Intake.Encoder.TIMEOUT_MS);
    this.intakeMotor.configNominalOutputReverse(0, Constants.Intake.Encoder.TIMEOUT_MS);
    this.intakeMotor.configPeakOutputForward(1, Constants.Intake.Encoder.TIMEOUT_MS);
    this.intakeMotor.configPeakOutputReverse(-1, Constants.Intake.Encoder.TIMEOUT_MS);

    // Config the Velocity closed loop gains in slot0
    this.intakeMotor.config_kF(
        Constants.Intake.Encoder.SLOT_ID,
        Constants.Intake.PID_GAINS.kF,
        Constants.Intake.Encoder.TIMEOUT_MS);
    this.intakeMotor.config_kP(
        Constants.Intake.Encoder.SLOT_ID,
        Constants.Intake.PID_GAINS.kP,
        Constants.Intake.Encoder.TIMEOUT_MS);
    this.intakeMotor.config_kI(
        Constants.Intake.Encoder.SLOT_ID,
        Constants.Intake.PID_GAINS.kI,
        Constants.Intake.Encoder.TIMEOUT_MS);
    this.intakeMotor.config_kD(
        Constants.Intake.Encoder.SLOT_ID,
        Constants.Intake.PID_GAINS.kD,
        Constants.Intake.Encoder.TIMEOUT_MS);
  }

  /**
   * Convert <rpmTarget> RPM to units / 100ms. 4096 Units/Rev * 500 RPM / 600 100ms/min in either
   * direction: velocity setpoint is in units/100ms
   */
  private double determineRPMVelocityTarget(double rpmTarget) {
    double targetVelocity_UnitsPer100ms = rpmTarget * Constants.Intake.Encoder.TICKS_PER_REV / 600;
    return targetVelocity_UnitsPer100ms;
  }
}
