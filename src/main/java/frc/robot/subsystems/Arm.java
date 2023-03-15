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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  /** Either Open (percentage) or Closed (position-based) */
  private enum ControlMode {
    ZEROING,
    ZEROED,
    OPEN,
    CLOSED;
  }

  private CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private DigitalInput lowerLimitSwitch;

  private double goalPosition = 0,
      lastGoalPosition = 0,
      currentPosition = 0,
      minPosition = 0,
      maxPosition = 0,
      minPercentage = 0,
      maxPercentage;

  private SendableChooser<ControlMode> controlModeChooser;
  private ControlMode desiredMode, currentMode = ControlMode.ZEROING;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Arm() {
    this.motor = new CANSparkMax(RobotProperty.ARM_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.motor.setInverted(RobotProperty.ARM_MOTOR_INVERT.getBoolean());
    this.motor.setIdleMode(IdleMode.kBrake);
    this.lowerLimitSwitch = new DigitalInput(RobotProperty.ARM_LIMIT_SWITCH_DIO.getInteger());

    this.encoder = this.motor.getEncoder();
    this.encoder.setPositionConversionFactor(Constants.Arm.ENCODER_CONVERSION_FACTOR);
    this.encoder.setPosition(this.goalPosition);

    this.pidController = this.motor.getPIDController();

    controlModeChooser = new SendableChooser<ControlMode>();
    controlModeChooser.setDefaultOption("Position Mode", ControlMode.CLOSED);
    controlModeChooser.addOption("Open Mode", ControlMode.OPEN);

    minPosition = Constants.Arm.MIN_POSITION;
    maxPosition = Constants.Arm.MAX_POSITION;
    minPercentage = Constants.Arm.PERCENTAGE_MIN;
    maxPercentage = Constants.Arm.PERCENTAGE_MAX;

    // PID coefficients
    Gains pidGains = Constants.Arm.PID;

    // set PID coefficients
    pidController.setP(pidGains.kP);
    pidController.setI(pidGains.kI);
    pidController.setD(pidGains.kD);
    pidController.setIZone(pidGains.kIzone);
    pidController.setFF(pidGains.kF);
    pidController.setOutputRange(Constants.Arm.PERCENTAGE_MIN, Constants.Arm.PERCENTAGE_MAX);

    pidController.setSmartMotionMaxVelocity(700, 0);
    pidController.setSmartMotionMinOutputVelocity(0, 0);
    pidController.setSmartMotionMaxAccel(400, 0);
    pidController.setSmartMotionAllowedClosedLoopError(.1, 0);
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
        if (desiredInput == 0) {
          this.currentMode = ControlMode.CLOSED;
          this.goalPosition = currentPosition;
        }
        break;
      case CLOSED:
        this.adjustGoalPosition(desiredInput);
        if (desiredInput != 0) {
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
      this.extendArm();
    } else if (modifier < 0) {
      this.retractArm();
    }
  }

  /** Raises the shoulder by a small amount, determined by the set conversion factor */
  public void extendArm() {
    this.goalPosition += .15;
  }

  /** Lowers the shoulder by a small amount, determined by the set conversion factor */
  public void retractArm() {
    this.goalPosition -= .15;
  }

  /** Control the arm via percentage speed of [-1, 1] */
  public void percentageControl(double desiredInput) {
    // When going down, we dont really need power
    if (desiredInput < 0) {
      desiredInput = MathUtil.clamp(desiredInput, minPercentage, maxPercentage);
    }

    if (currentPosition >= maxPosition) {
      desiredInput = 0;
    }

    if (isFullyRetracted() && desiredInput < 0) {
      desiredInput = 0;
      this.resetEncoder();
    }
    this.motor.set(desiredInput);
  }

  public void performZeroing() {
    this.motor.set(-.3);
    if (isFullyRetracted()) {
      this.zero();
      this.motor.set(0);
      currentMode = ControlMode.ZEROED;
    }
  }

  public void smartMotionPeriodic() {
    if (this.goalPosition <= minPosition && this.isFullyRetracted()) {
      this.goalPosition = minPosition;
      this.encoder.setPosition(minPosition);
    }

    // Bound our goal position to something realistic
    this.goalPosition = this.ensurePositionInRange(this.goalPosition);

    if (goalPosition != lastGoalPosition) {
      this.pidController.setReference(this.goalPosition, ControlType.kSmartMotion);
    }

    if (goalPosition == minPosition) {
      this.motor.set(isFullyRetracted() ? -.04 : -.1);
    }

    this.lastGoalPosition = goalPosition;
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
    return MathUtil.clamp(desiredPosition, minPosition, maxPosition);
  }

  public void resetEncoder() {
    this.encoder.setPosition(minPosition);
  }

  public void zero() {
    this.encoder.setPosition(minPosition);
    this.goalPosition = minPosition;
  }

  public double percentageExtended() {
    return this.currentPosition / maxPosition;
  }

  public boolean isFullyRetracted() {
    return this.lowerLimitSwitch.get();
  }

  /**
   * Prevent the shoulder from moving if the arm is extended more than the percentage in the
   * function.
   */
  public boolean armExtendedToUnsafeMoveDistance() {
    return this.percentageExtended() < .30;
  }

  /**
   * Determines how much dynamic feed forward multiplier to add to the shoulder. When the arm is
   * extended, we require much more "push" to keep it raised.
   */
  public double getShoulderFeedForwardMultiplier() {
    double armPercent = this.percentageExtended();
    if (armPercent < .3) {
      return 1;
    } else if (armPercent < .6) {
      return 1.5;
    } else if (armPercent < .8) {
      return 2;
    } else {
      return 2.5;
    }
  }

  /** Keep the SmartDashboard updated */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Position", currentPosition);
    SmartDashboard.putNumber("Arm Goal", this.goalPosition);
    SmartDashboard.putNumber("Arm Power", this.motor.getAppliedOutput());
    SmartDashboard.putBoolean("Arm Fully Down", this.lowerLimitSwitch.get());
    SmartDashboard.putData("Arm Mode", controlModeChooser);
  }
}
