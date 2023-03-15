package frc.robot.subsystems;

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
import java.util.HashMap;

public class Shoulder extends SubsystemBase {

  public interface ArmExtendedCheck {
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
    SAFE_MAX(Constants.Shoulder.SAFE_MAX),

    COLLECT(1),

    LOW_CUBE(4.3),
    MED_CUBE(9),
    HIGH_CUBE(10.5),

    LOW_CONE(7.75),
    MED_CONE(10.2),
    HIGH_CONE(12.5);

    private final double position;

    private ShoulderPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return this.position;
    }
  }

  public enum Direction {
    ACTIVE_UPWARDS,
    UPWARD,
    INITIATE_DOWNWARD,
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
      feedForwardMult = 1,
      initiateDownwardCounts = 0;

  ArmExtendedCheck armExtendedCheck;
  ShoulderFeedForwardMultiplier feedForwardMultCheck;

  private SendableChooser<ControlMode> controlModeChooser;
  private ControlMode desiredMode, currentMode = ControlMode.ZEROING;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Shoulder(
      ArmExtendedCheck armExtendedCheck, ShoulderFeedForwardMultiplier feedForwardMultCheck) {
    this.motor =
        new CANSparkMax(RobotProperty.SHOULDER_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.motor.setInverted(RobotProperty.SHOULDER_MOTOR_INVERT.getBoolean());
    this.motor.setIdleMode(IdleMode.kBrake);

    this.encoder = this.motor.getEncoder();
    this.encoder.setPositionConversionFactor(Constants.Shoulder.ENCODER_CONVERSION_FACTOR);
    this.encoder.setPosition(this.goalPosition);

    this.lowerLimitSwitch = new DigitalInput(RobotProperty.SHOULDER_LIMIT_SWITCH_DIO.getInteger());

    this.mechanicalBrake =
        new DoubleSolenoid(
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

    this.armExtendedCheck = armExtendedCheck;

    this.feedForwardMultCheck = feedForwardMultCheck;
  }

  public void setMode(ControlMode mode) {
    this.currentMode = mode;
  }

  /** Parse input from a controller and handle the different control modes */
  public void controllerAction(double desiredInput) {
    desiredMode = controlModeChooser.getSelected();
    if (currentMode == ControlMode.ZEROED && desiredMode != currentMode) {
      currentMode = desiredMode;
    }

    switch (currentMode) {
      case OPEN:
        this.percentageControl(desiredInput > 0 ? desiredInput : .00001);
        if (desiredInput == 0) {
          this.currentMode = ControlMode.CLOSED;
          this.direction = Direction.HOLD;
          this.goalPosition = currentPosition;
        }
        break;
      case CLOSED:
        this.adjustGoalPosition(desiredInput);
        if (desiredInput > 0) {
          this.currentMode = ControlMode.OPEN;
        }
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

    if (currentPosition >= maxPosition) {
      desiredInput = 0;
    }

    if (isFullyDown() && desiredInput < 0) {
      desiredInput = 0;
      this.resetEncoder();
    }
    this.motor.set(desiredInput);
    this.goalPosition = this.encoder.getPosition();
  }

  /** When ZEROING, drive the shoulder down until the limit switch is set */
  public void performZeroing() {
    this.motor.set(Constants.Shoulder.ZEROING_SPEED);
    if (isFullyDown()) {
      this.zero();
      this.motor.set(0);
      currentMode = ControlMode.ZEROED;
    }
  }

  /**
   * This is called on every periodic iteration where Position Mode is enabled. It is responsible
   * for setting the motor PID controller position and determining _what_ the motors / controller
   * actually should be doing.
   */
  public void smartMotionPeriodic() {
    // If we are against the limit switch, then reset the position to
    // be at the minimum so we don't break the robot
    if (this.goalPosition < minPosition && this.isFullyDown()) {
      this.goalPosition = minPosition;
      this.encoder.setPosition(minPosition);
    }

    // The forward feed is applied to ensure the motor has enough power
    // to lift the arm when close to the goal.
    double newFeedForwardMult = this.feedForwardMultCheck.op();
    double newFeedForward = newFeedForwardMult * Constants.Shoulder.PID.kF;
    if (newFeedForwardMult != feedForwardMult) {
      this.pidController.setFF(newFeedForward);
    }

    // Bound our goal position to something realistic
    this.goalPosition = this.ensurePositionInRange(this.goalPosition);

    // If the goal position has changed since last iteration, we
    // are trying to go somewhere new
    if (goalPosition != this.lastGoalPosition) {
      // We are trying to go upwards
      if (goalPosition > lastGoalPosition) {
        this.direction = Direction.UPWARD;
        // We are trying to go down, but we need to drive the motor
        // upwards a bit to disengage the mechanical brake
      } else if (this.direction != Direction.DOWNWARD) {
        this.direction = Direction.INITIATE_DOWNWARD;
        this.initiateDownwardCounts = 0;
      }
    }

    double localGoalPosition = this.goalPosition;

    // If we are going upwards, just add a bit to "overdrive" the motor to get
    // there and allow the mechanical brake to lock
    if (this.direction == Direction.UPWARD) {
      localGoalPosition += Constants.Shoulder.UPWARDS_ADDITIONAL_GOAL;
    }

    // Determine our error + error tolerance so we know how close we are to
    // switching into the HOLD mode, which engages the mechanical brake
    double offBy = Math.abs(this.currentPosition - localGoalPosition);
    double errorTolerance; // [0, ~14]
    if (direction == Direction.UPWARD) {
      errorTolerance = Constants.Shoulder.UPWARD_ERROR_TOLERANCE;
    } else if (direction == Direction.DOWNWARD) {
      errorTolerance = Constants.Shoulder.DOWNWARD_ERROR_TOLERANCE;
    }

    boolean aboveGoal = currentPosition > goalPosition;

    // If we are moving upward and are above the goal, then we can engage the
    // mechanical brake and give the motor a rest
    if ((direction == Direction.UPWARD) && aboveGoal) {
      direction = Direction.HOLD;
    }

    // When ZEROING, just let the PID controller follow the current arm position
    // so that we can drive it via a set speed until we hit the limit switch
    if (currentMode == ControlMode.ZEROING) {
      this.mechanicalBrake.set(COAST_MODE);
      this.pidController.setReference(currentPosition, ControlType.kSmartMotion);
    }

    boolean armExtended = this.armExtendedCheck != null && this.armExtendedCheck.op();

    switch (direction) {
      case HOLD:
        // When HOLDing, set the mechanical brake and add some feed-forward
        // to allow the motor to ensure the arm doesn't move.
        this.mechanicalBrake.set(BRAKE_MODE);
        this.motor.set(newFeedForward);
        break;
      case INITIATE_DOWNWARD:
        // When we _start_ going downwards, drive the motor up a short amount
        // and allow the mechanical brake to disengage properly before driving down
        this.mechanicalBrake.set(COAST_MODE);
        this.motor.set(percentageRaised() > .5 ? .6 : .4);
        if (initiateDownwardCounts > 5) {
          this.motor.set(0);
          this.pidController.setReference(goalPosition, ControlType.kSmartMotion);
          this.direction = Direction.DOWNWARD;
        }
        initiateDownwardCounts++;
        break;
      default:
        // When going UP or DOWN, just update the arm position and let it cruise
        this.mechanicalBrake.set(COAST_MODE);

        if (localGoalPosition != lastGoalPosition && armExtended) {
          this.pidController.setReference(localGoalPosition, ControlType.kSmartMotion);
        }
        break;
    }

    // Update the last goal position so we have it to reference
    // on the next iteration
    this.lastGoalPosition = this.goalPosition;
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
    directions.put(Direction.INITIATE_DOWNWARD, "Init Downward");
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
