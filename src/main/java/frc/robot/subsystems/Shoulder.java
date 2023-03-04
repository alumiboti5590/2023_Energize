package frc.robot.subsystems;

import com.alumiboti5590.util.properties.RobotProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {

  public enum ShoulderPosition {
    ZERO(Constants.Shoulder.MIN_POSITION),
    MAX(Constants.Shoulder.MAX_POSITION);

    private final double position;

    private ShoulderPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return this.position;
    }
  }

  private CANSparkMax shoulderMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder shoulderEncoder;

  double goalPosition = 0;

  public Shoulder() {
    this.shoulderMotor =
        new CANSparkMax(RobotProperty.SHOULDER_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.shoulderMotor.setInverted(RobotProperty.ARM_MOTOR_INVERT.getBoolean());
    this.shoulderMotor.setIdleMode(IdleMode.kBrake);

    this.shoulderEncoder = this.shoulderMotor.getEncoder();
    this.shoulderEncoder.setPositionConversionFactor(Constants.Shoulder.ENCODER_CONVERSION_FACTOR);
    this.shoulderEncoder.setPosition(this.goalPosition);

    this.pidController = this.shoulderMotor.getPIDController();
  }

  /**
   * Allows for a modifier value to control the shoulder height in small increments A positive value
   * will raise the arm, a negative value will lower the arm, and a zero value will not affect the
   * arm at all.
   */
  public void openLoopControl(double modifier) {
    if (modifier > 0) {
      this.raiseShoulder();
    } else if (modifier < 0) {
      this.lowerShoulder();
    }
  }

  /** Raises the shoulder by a small amount, determined by the set conversion factor */
  public void raiseShoulder() {
    this.goalPosition++;
  }

  /** Lowers the shoulder by a small amount, determined by the set conversion factor */
  public void lowerShoulder() {
    this.goalPosition--;
  }

  /**
   * Sets the shoulder to a predetermined position
   *
   * @param goaShoulderPosition
   */
  public void setGoalPosition(ShoulderPosition goaShoulderPosition) {
    this.goalPosition = goaShoulderPosition.getPosition();
  }

  /** Runs every 20ms and sets the smart motion parameters needed for the shoulder */
  @Override
  public void periodic() {
    // Bound our goal position to something realistic
    this.goalPosition = this.ensurePositionInRange(this.goalPosition);

    // Provide some forward feed as we go up to ensure the motor can move the weight
    if (this.shoulderEncoder.getPosition() < this.goalPosition) {
      this.pidController.setFF(0);
    } else {
      this.pidController.setFF(Constants.Shoulder.FORWARD_FEED_UPWARDS);
    }

    this.pidController.setReference(this.goalPosition, ControlType.kSmartMotion);
    this.updateSmartDashboard();
  }

  /** Ensure the position provided is not outside of the shoulder's physical boundaries */
  private double ensurePositionInRange(double desiredPosition) {
    return Math.min(
        Math.max(Constants.Shoulder.MIN_POSITION, desiredPosition),
        Constants.Shoulder.MAX_POSITION);
  }

  /** Keep the SmartDashboard updated */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Shoulder Position", this.shoulderEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder Goal", this.goalPosition);
  }
}
