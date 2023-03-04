package frc.robot.subsystems;

import com.alumiboti5590.util.properties.RobotProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {

  /** Predetermined shoulder positions */
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

  /** Either Open (percentage) or Closed (position-based) */
  private enum ControlMode {
    OPEN,
    CLOSED;
  }

  private CANSparkMax shoulderMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder shoulderEncoder;

  private double goalPosition = 0;

  private SendableChooser<ControlMode> controlModeChooser;

  public Shoulder() {
    this.shoulderMotor =
        new CANSparkMax(RobotProperty.SHOULDER_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.shoulderMotor.setInverted(RobotProperty.ARM_MOTOR_INVERT.getBoolean());
    this.shoulderMotor.setIdleMode(IdleMode.kBrake);

    this.shoulderEncoder = this.shoulderMotor.getEncoder();
    this.shoulderEncoder.setInverted(RobotProperty.SHOULDER_ENCODER_INVERT.getBoolean());
    this.shoulderEncoder.setPositionConversionFactor(Constants.Shoulder.ENCODER_CONVERSION_FACTOR);
    this.shoulderEncoder.setPosition(this.goalPosition);

    this.pidController = this.shoulderMotor.getPIDController();

    controlModeChooser = new SendableChooser<ControlMode>();
    controlModeChooser.setDefaultOption("Closed Position Mode", ControlMode.CLOSED);
    controlModeChooser.addOption("Open Mode", ControlMode.OPEN);
  }

  /** Parse input from a controller and handle the different control modes */
  public void controllerAction(double desiredInput) {
    if (controlModeChooser.getSelected() == ControlMode.OPEN) {
      this.percentageControl(desiredInput);
    } else {
      this.adjustGoalPosition(desiredInput);
    }
  }

  /**
   * Allows for a modifier value to control the shoulder height in small increments A positive value
   * will raise the arm, a negative value will lower the arm, and a zero value will not affect the
   * arm at all.
   */
  public void adjustGoalPosition(double modifier) {
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

  /** Control the arm via percentage speed of [-1, 1] */
  public void percentageControl(double desiredInput) {
    // When going down, we dont really need power
    if (desiredInput < 0) {
      desiredInput = Math.max(desiredInput, Constants.Shoulder.OPEN_PERCENTAGE_DOWN_MAX);
    }
    this.shoulderMotor.set(desiredInput);
  }

  public void smartMotionPeriodic() {
    // Bound our goal position to something realistic
    this.goalPosition = this.ensurePositionInRange(this.goalPosition);

    // Provide some forward feed as we go up to ensure the motor can move the weight
    if (this.shoulderEncoder.getPosition() < this.goalPosition) {
      this.pidController.setFF(0);
    } else {
      this.pidController.setFF(Constants.Shoulder.FORWARD_FEED_UPWARDS);
    }

    this.pidController.setReference(this.goalPosition, ControlType.kSmartMotion);
  }

  /** Runs every 20ms and sets the smart motion parameters needed for the shoulder */
  @Override
  public void periodic() {
    if (controlModeChooser.getSelected() == ControlMode.CLOSED) {
      this.smartMotionPeriodic();
    }
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
