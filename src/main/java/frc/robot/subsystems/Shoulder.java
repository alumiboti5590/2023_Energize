package frc.robot.subsystems;

import java.util.HashMap;

import com.alumiboti5590.util.pid.Gains;
import com.alumiboti5590.util.properties.RobotProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {

  public interface ShoulderCanMove {
    public boolean op();
  }

  public interface ShoulderFeedForwardMultiplier {
    public double op();
  }

  /** Predetermined shoulder positions */
  public enum ShoulderPosition {
    ZERO(Constants.Shoulder.MIN_POSITION),
    MAX(Constants.Shoulder.MAX_POSITION),
    HALFWAY(Constants.Shoulder.HALFWAY),
    SAFE_MAX(Constants.Shoulder.SAFE_MAX);

    private final double position;

    private ShoulderPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return this.position;
    }
  }

  public enum Direction {
    UPWARD,
    DOWNWARD,
    HOLD,
  }

  /** Either Open (percentage) or Closed (position-based) */
  private enum ControlMode {
    ZEROING,
    ZEROED,
    OPEN,
    CLOSED;
  }

  public enum BrakeMode {
    COAST,
    BRAKE;
  }

  private DoubleSolenoid.Value COAST_MODE = DoubleSolenoid.Value.kForward,
      BRAKE_MODE = DoubleSolenoid.Value.kReverse;
  private Direction direction = Direction.HOLD;

  private CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private DigitalInput lowerLimitSwitch;

  private DoubleSolenoid mechanicalBrake;

  private double goalPosition = 0,
      lastGoalPosition = 0,
      currentPosition = 0,
      minPosition = 0,
      maxPosition = 0,
      minPercentage = 0,
      maxPercentage = 0,
      feedForwardMult = 1;

  ShoulderCanMove canMoveCheck;
  ShoulderFeedForwardMultiplier feedForwardMultCheck;

  private SendableChooser<ControlMode> controlModeChooser;
  private ControlMode desiredMode, currentMode = ControlMode.ZEROING;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Shoulder(
      ShoulderCanMove canMoveCheck, ShoulderFeedForwardMultiplier feedForwardMultCheck) {
    this.motor =
        new CANSparkMax(RobotProperty.SHOULDER_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.motor.setInverted(RobotProperty.SHOULDER_MOTOR_INVERT.getBoolean());
    this.motor.setIdleMode(IdleMode.kBrake);

    this.encoder = this.motor.getEncoder();
    this.encoder.setPositionConversionFactor(Constants.Shoulder.ENCODER_CONVERSION_FACTOR);
    this.encoder.setPosition(this.goalPosition);

    this.lowerLimitSwitch = new DigitalInput(RobotProperty.SHOULDER_LIMIT_SWITCH_DIO.getInteger());

    this.mechanicalBrake = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      RobotProperty.SHOULDER_BRAKE_SOLENOID_A.getInteger(),
      RobotProperty.SHOULDER_BRAKE_SOLENOID_B.getInteger());
    
    // Invert if open & close is backwards
    if (RobotProperty.SHOULDER_BRAKE_INVERT.getBoolean()) {
      BRAKE_MODE = DoubleSolenoid.Value.kReverse;
      COAST_MODE = DoubleSolenoid.Value.kReverse;
    }

    this.pidController = this.motor.getPIDController();

    controlModeChooser = new SendableChooser<ControlMode>();
    controlModeChooser.setDefaultOption("Position Mode", ControlMode.CLOSED);
    controlModeChooser.addOption("Open Mode", ControlMode.OPEN);

    minPosition = Constants.Shoulder.MIN_POSITION;
    maxPosition = Constants.Shoulder.MAX_POSITION;
    minPercentage = Constants.Shoulder.PERCENTAGE_MIN;
    maxPercentage = Constants.Shoulder.PERCENTAGE_MAX;

    // PID coefficients
    Gains pidGains = Constants.Shoulder.PID;

    // set PID coefficients
    pidController.setP(pidGains.kP);
    pidController.setI(pidGains.kI);
    pidController.setD(pidGains.kD);
    pidController.setIZone(pidGains.kIzone);
    pidController.setFF(pidGains.kF);
    pidController.setOutputRange(
        Constants.Shoulder.PERCENTAGE_MIN, Constants.Shoulder.PERCENTAGE_MAX);

    pidController.setSmartMotionMaxVelocity(700, 0);
    pidController.setSmartMotionMinOutputVelocity(0, 0);
    pidController.setSmartMotionMaxAccel(350, 0);
    pidController.setSmartMotionAllowedClosedLoopError(.5, 0);

    this.canMoveCheck = canMoveCheck;

    this.feedForwardMultCheck = feedForwardMultCheck;
  }

  /** Parse input from a controller and handle the different control modes */
  public void controllerAction(double desiredInput) {
    desiredMode = controlModeChooser.getSelected();
    if (currentMode == ControlMode.ZEROED && desiredMode != currentMode) {
      currentMode = desiredMode;
    }

    switch (currentMode) {
      case OPEN:
        this.percentageControl(desiredInput);
        break;
      case CLOSED:
        this.adjustGoalPosition(desiredInput);
        break;
      case ZEROING:
        this.performZeroing();
        break;
      default:
        this.motor.set(0);
    }
  }

  public void startZeroingMode() {
    this.currentMode = ControlMode.ZEROING;
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
    this.goalPosition += .05;
  }

  /** Lowers the shoulder by a small amount, determined by the set conversion factor */
  public void lowerShoulder() {
    this.goalPosition -= .05;
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
    this.mechanicalBrake.set(COAST_MODE);
  
    // When going down, we dont really need power
    if (desiredInput < 0) {
      desiredInput = MathUtil.clamp(desiredInput, minPercentage, maxPercentage);
    }
    if (isFullyDown() && desiredInput < 0) {
      desiredInput = 0;
      this.resetEncoder();
    }
    this.motor.set(desiredInput);
  }

  public void performZeroing() {
    this.motor.set(-.3);
    if (isFullyDown()) {
      this.zero();
      this.motor.set(0);
      currentMode = ControlMode.ZEROED;
    }
  }

  public void smartMotionPeriodic() {
    if (this.goalPosition < minPosition && this.isFullyDown()) {
      this.goalPosition = minPosition;
      this.encoder.setPosition(minPosition);
    }

    double newFeedForwardMult = this.feedForwardMultCheck.op();
    double newFeedForward = newFeedForwardMult * Constants.Shoulder.PID.kF;
    if (newFeedForwardMult != feedForwardMult) {
      this.pidController.setFF(newFeedForward);
    }

    // Bound our goal position to something realistic
    this.goalPosition = this.ensurePositionInRange(this.goalPosition);

    if (goalPosition != this.lastGoalPosition) {
      this.direction = goalPosition > lastGoalPosition ? Direction.UPWARD : Direction.DOWNWARD;
    }

    double offBy = Math.abs(this.currentPosition - this.goalPosition);
    double errorTolerance = .5; // [0, ~14]
    boolean aboveGoal = currentPosition > goalPosition;

    if (direction == Direction.UPWARD && aboveGoal && offBy <= errorTolerance) {
      direction = Direction.HOLD;
    }

    if (direction == Direction.DOWNWARD && !aboveGoal && offBy <= errorTolerance) {
      direction = Direction.HOLD;
    }

    // Either HOLD or let the robot move
    if (currentMode == ControlMode.ZEROING) {
      this.mechanicalBrake.set(COAST_MODE);
    }
    if (direction == Direction.HOLD) {
      this.mechanicalBrake.set(BRAKE_MODE);
      this.motor.set(newFeedForward);
    } else {
      this.mechanicalBrake.set(COAST_MODE);

      boolean canUpdateRef = this.canMoveCheck != null && this.canMoveCheck.op();
      if (goalPosition != lastGoalPosition && canUpdateRef) {
        this.pidController.setReference(this.goalPosition, ControlType.kSmartMotion);
      }
    }

    this.lastGoalPosition = goalPosition;
  }

  public void setBrakeMode(boolean braking) {
    this.mechanicalBrake.set(braking ? BRAKE_MODE : COAST_MODE);
  }

  /** Runs every 20ms and sets the smart motion parameters needed for the shoulder */
  @Override
  public void periodic() {
    this.currentPosition = this.encoder.getPosition();

    if (controlModeChooser.getSelected() == ControlMode.CLOSED) {
      this.smartMotionPeriodic();
    }
    this.updateSmartDashboard();
  }

  /** Ensure the position provided is not outside of the shoulder's physical boundaries */
  private double ensurePositionInRange(double desiredPosition) {
    return Math.min(desiredPosition, maxPosition);
  }

  public void resetEncoder() {
    this.encoder.setPosition(0);
  }

  public void zero() {
    this.encoder.setPosition(0);
    this.goalPosition = 0;
  }

  public double percentageRaised() {
    return this.currentPosition / maxPosition;
  }

  public boolean isFullyDown() {
    return this.lowerLimitSwitch.get();
  }

  /** Keep the SmartDashboard updated */
  public void updateSmartDashboard() {
    HashMap<Direction, String> directions = new HashMap<Direction, String>();
    directions.put(Direction.UPWARD, "Upwards");
    directions.put(Direction.DOWNWARD, "Downwards");
    directions.put(Direction.HOLD, "Hold");

    SmartDashboard.putNumber("Shoulder Position", this.encoder.getPosition());
    SmartDashboard.putNumber("Shoulder Goal", this.goalPosition);
    SmartDashboard.putNumber("Shoulder Power", this.motor.getAppliedOutput());
    SmartDashboard.putString("Shoulder Direction", directions.get(direction));
    SmartDashboard.putBoolean("Shoulder Fully Down", this.lowerLimitSwitch.get());
    SmartDashboard.putData("Shoulder Mode", controlModeChooser);
  }
}
