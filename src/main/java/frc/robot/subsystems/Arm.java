package frc.robot.subsystems;

import com.alumiboti5590.util.properties.RobotProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private static enum ControlMode {
    PERCENTAGE, // No encoder enabled, brushed motors
    POSITION // Encoder-enabled, brushless motors
  }

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private ControlMode controlMode = ControlMode.PERCENTAGE;

  public Arm() {
    MotorType motorType =
        RobotProperty.ARM_MOTOR_BRUSHLESS.getBoolean() ? MotorType.kBrushless : MotorType.kBrushed;
    this.armMotor = new CANSparkMax(RobotProperty.ARM_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.armMotor.setInverted(RobotProperty.ARM_MOTOR_INVERT.getBoolean());
    this.armMotor.setIdleMode(IdleMode.kBrake);

    if (motorType == MotorType.kBrushless) {
      this.controlMode = ControlMode.POSITION;
      this.armEncoder = this.armMotor.getEncoder();
      this.armEncoder.setPositionConversionFactor(Constants.Arm.ENCODER_CONVERSION_FACTOR);
    }
  }

  /** Used by the robot to determine controller mappings */
  public boolean isPositionControlled() {
    return this.controlMode == ControlMode.POSITION;
  }
}
