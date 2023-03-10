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

public class Shoulder extends SubsystemBase {

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
      minPosition = 0,
      maxPosition = 0,
      minPercentage = 0,
      maxPercentage;

  private SendableChooser<ControlMode> controlModeChooser;
  private ControlMode desiredMode, currentMode = ControlMode.ZEROING;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Shoulder() {
    this.motor =
        new CANSparkMax(RobotProperty.SHOULDER_MOTOR_ID.getInteger(), MotorType.kBrushless);
    this.motor.setInverted(RobotProperty.SHOULDER_MOTOR_INVERT.getBoolean());
    this.motor.setIdleMode(IdleMode.kBrake);

    this.encoder = this.motor.getEncoder();
    this.encoder.setPositionConversionFactor(Constants.Shoulder.ENCODER_CONVERSION_FACTOR);
    this.encoder.setPosition(this.goalPosition);

    this.lowerLimitSwitch = new DigitalInput(RobotProperty.SHOULDER_LIMIT_SWITCH_DIO.getInteger());

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

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Sh P Gain", pidController.getP());
    SmartDashboard.putNumber("Sh I Gain", pidController.getI());
    SmartDashboard.putNumber("Sh D Gain", pidController.getD());
    SmartDashboard.putNumber("Sh I Zone", pidController.getIZone());
    SmartDashboard.putNumber("Sh Feed Forward", pidController.getFF());
    SmartDashboard.putNumber("Sh Max Output", pidController.getOutputMin());
    SmartDashboard.putNumber("Sh Min Output", pidController.getOutputMax());
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
    SmartDashboard.putNumber("Shoulder Open Desired", desiredInput);
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
    // pidController.setP(SmartDashboard.getNumber("Sh P Gain", pidController.getP()));
    // pidController.setI(SmartDashboard.getNumber("Sh I Gain", pidController.getI()));
    // pidController.setD(SmartDashboard.getNumber("Sh D Gain", pidController.getD()));
    // pidController.setIZone(SmartDashboard.getNumber("Sh I Zone", pidController.getIZone()));
    // pidController.setFF(SmartDashboard.getNumber("Sh Feed Forward", pidController.getFF()));

    if (this.goalPosition < minPosition && this.isFullyDown()) {
      this.goalPosition = minPosition;
      this.encoder.setPosition(minPosition);
    }

    // Bound our goal position to something realistic
    this.goalPosition = this.ensurePositionInRange(this.goalPosition);

    this.pidController.setReference(this.goalPosition, ControlType.kPosition);
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
    return Math.min(desiredPosition, maxPosition);
  }

  public void resetEncoder() {
    this.encoder.setPosition(0);
  }

  public void zero() {
    this.encoder.setPosition(0);
    this.goalPosition = 0;
  }

  public boolean isFullyDown() {
    return this.lowerLimitSwitch.get();
  }

  /** Keep the SmartDashboard updated */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Shoulder Position", this.encoder.getPosition());
    SmartDashboard.putNumber("Shoulder Goal", this.goalPosition);
    SmartDashboard.putNumber("Shoulder Power", this.motor.getAppliedOutput());
    SmartDashboard.putBoolean("Shoulder Fully Down", this.lowerLimitSwitch.get());
    SmartDashboard.putData("Shoulder Mode", controlModeChooser);
  }
}
