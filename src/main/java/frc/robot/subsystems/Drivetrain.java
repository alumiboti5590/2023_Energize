// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.alumiboti5590.util.Tuple;
import com.alumiboti5590.util.filters.IInputFilter;
import com.alumiboti5590.util.filters.PolynomialInputFilter;
import com.alumiboti5590.util.properties.RobotProperty;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controllers.IDriverController;

public class Drivetrain extends SubsystemBase {

  public enum DriveType {
    ARCADE,
    CURVATURE,
    TANK_DRIVE,
  }

  private SendableChooser<DriveType> driveTypeChooser;

  private IInputFilter inputFilter1, inputFilter2;

  // Basic Drivetrain things
  private WPI_TalonSRX leftLeader, leftFollower, rightLeader, rightFollower;
  private Encoder leftEncoder, rightEncoder;
  private DifferentialDrive diffDrive;

  private AHRS navX;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Set up the motor controllers
    leftLeader = new WPI_TalonSRX(RobotProperty.DRIVETRAIN_LEFT_LEADER_ID.getInteger());
    leftFollower = new WPI_TalonSRX(RobotProperty.DRIVETRAIN_LEFT_FOLLOWER_ID.getInteger());
    rightLeader = new WPI_TalonSRX(RobotProperty.DRIVETRAIN_RIGHT_LEADER_ID.getInteger());
    rightFollower = new WPI_TalonSRX(RobotProperty.DRIVETRAIN_RIGHT_FOLLOWER_ID.getInteger());

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Set up coast mode
    leftLeader.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    // Configure Encoder settings to have proper ratios and distance controls
    leftEncoder =
        new Encoder(
            RobotProperty.DRIVETRAIN_ENCODER_LEFT_CHAN_A.getInteger(),
            RobotProperty.DRIVETRAIN_ENCODER_LEFT_CHAN_B.getInteger());
    rightEncoder =
        new Encoder(
            RobotProperty.DRIVETRAIN_ENCODER_RIGHT_CHAN_A.getInteger(),
            RobotProperty.DRIVETRAIN_ENCODER_RIGHT_CHAN_B.getInteger());

    setEncoderDistancePerPulse(Constants.Drivetrain.metersPerEncoderPulse());

    // Set up AHRS
    navX = new AHRS(SPI.Port.kMXP);

    // set current limit to avoid brownouts and spikes
    setCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

    // Set motor inversions if necessary
    leftLeader.setInverted(RobotProperty.DRIVETRAIN_LEFT_LEADER_INVERT.getBoolean());
    rightLeader.setInverted(RobotProperty.DRIVETRAIN_RIGHT_LEADER_INVERT.getBoolean());

    leftFollower.setInverted(RobotProperty.DRIVETRAIN_LEFT_FOLLOWER_INVERT.getBoolean());
    rightFollower.setInverted(RobotProperty.DRIVETRAIN_RIGHT_FOLLOWER_INVERT.getBoolean());

    leftEncoder.setReverseDirection(RobotProperty.DRIVETRAIN_ENCODER_LEFT_INVERT.getBoolean());
    rightEncoder.setReverseDirection(RobotProperty.DRIVETRAIN_ENCODER_RIGHT_INVERT.getBoolean());

    // Set ramp rate to make smoother acceleration and avoid electrical spikes
    this.setRampRate(Constants.Drivetrain.RAMP_RATE_SECONDS);

    // set up DiffDrive
    diffDrive = new DifferentialDrive(leftLeader, rightLeader);
    diffDrive.setSafetyEnabled(false);

    // Set up any input filters necessary
    // Default to squared
    inputFilter1 = new PolynomialInputFilter(2);
    inputFilter2 = new PolynomialInputFilter(2);

    // Clamp max speed if necessary

    // Reset odometry
    resetOdometry();
    navX.reset();

    // Set up SmartDashboard drive type changes
    driveTypeChooser = new SendableChooser<>();
    driveTypeChooser.setDefaultOption("Curvature", DriveType.CURVATURE);
    driveTypeChooser.addOption("Arcade", DriveType.ARCADE);
    driveTypeChooser.addOption("Tank Drive", DriveType.TANK_DRIVE);
    SmartDashboard.putData("Drive Type", driveTypeChooser);
  }

  public void setRampRate(double rate) {
    this.leftLeader.configOpenloopRamp(rate);
    this.leftLeader.configOpenloopRamp(rate);
    this.rightLeader.configOpenloopRamp(rate);
    this.rightLeader.configOpenloopRamp(rate);
  }

  public void setCurrentLimit(int currentInAmps) {
    leftLeader.configPeakCurrentLimit(currentInAmps);
    rightLeader.configPeakCurrentLimit(currentInAmps);
    leftFollower.configPeakCurrentLimit(currentInAmps);
    rightFollower.configPeakCurrentLimit(currentInAmps);
  }

  public void setEncoderDistancePerPulse(double distancePerPulse) {
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);
  }

  private void resetEncoders() {
    this.leftEncoder.reset();
    this.rightEncoder.reset();
  }

  public double getLeftDistanceFeet() {
    return this.leftEncoder.getDistance();
  }

  public double getRightDistanceFeet() {
    return this.rightEncoder.getDistance();
  }

  public double getLeftVelocityFPS() {
    return this.leftEncoder.getRate();
  }

  public double getRightVelocityFPS() {
    return this.rightEncoder.getRate();
  }

  public void resetOdometry() {
    resetEncoders();
  }

  public double getHeadingDegrees() {
    return navX.getYaw();
  }

  public double getPitchDegrees() {
    return navX.getPitch();
  }

  public void resetYaw() {
    navX.reset();
  }

  // Drive methods
  public void controllerDrive(IDriverController controller, boolean disableController) {
    DriveType driveType = driveTypeChooser.getSelected();
    Tuple<Double, Double> inputs;

    if (disableController) {
      return;
    }

    // There's a few different ways the drive controller sticks can be used
    // to drive the robot, so this fetches the right type of inputs (first + second)
    // from the left + right sticks from the appropriate axis values.
    switch (driveType) {
      case ARCADE:
      case CURVATURE:
        inputs = controller.getArcadeOrCurvatureDriveValues();
        break;
      default:
        inputs = controller.getTankDriveValues();
        break;
    }

    // We disallow max speed by default (without turbo button) by having
    // a decimal multiplier that reduces the speed to a normally-allowed max
    double multiplier = RobotProperty.STANDARD_DRIVE_SPEED.getDouble();
    if (controller.getTurboButton().getAsBoolean()) {
      multiplier = RobotProperty.TURBO_DRIVE_SPEED.getDouble();
    }

    double steeringInput = inputs.second * multiplier;
    if (RobotProperty.DRIVETRAIN_INVERT_STEERING.getBoolean()) {
      steeringInput *= -1;
    }

    // Call the different method appropriate for the desired drive type
    // with the collected controller inputs. See the DriveType enum or
    // the individually called method documentation for more info
    switch (driveType) {
      case ARCADE:
        this.arcadeDrive(inputs.first * multiplier, steeringInput);
        break;
      case CURVATURE:
        boolean isQuickTurn = controller.getCurvatureDriveQuickTurn();
        this.curvatureDrive(inputs.first * multiplier, steeringInput, isQuickTurn);
        break;
      case TANK_DRIVE:
        this.tankDrive(inputs.first, inputs.second);
        break;
    }
  }

  public void arcadeDrive(double speed, double rotation) {
    arcadeDrive(speed, rotation, false, false);
  }

  public void arcadeDrive(
      double speed, double rotation, boolean skipSpeedFilter, boolean skipRotationFilter) {
    if (!skipSpeedFilter) {
      speed = this.inputFilter1.get(speed);
    }

    if (!skipRotationFilter) {
      rotation = this.inputFilter2.get(rotation);
    }

    diffDrive.arcadeDrive(speed, rotation, false);
  }

  public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
    curvatureDrive(speed, rotation, isQuickTurn, false, false);
  }

  public void curvatureDrive(
      double speed,
      double rotation,
      boolean isQuickTurn,
      boolean skipSpeedFilter,
      boolean skipRotationFilter) {
    if (!skipSpeedFilter) {
      speed = this.inputFilter1.get(speed);
    }

    if (!skipRotationFilter) {
      rotation = this.inputFilter2.get(rotation);
    }

    diffDrive.curvatureDrive(speed, rotation, isQuickTurn);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false, false);
  }

  public void tankDrive(
      double leftSpeed, double rightSpeed, boolean skipLeftFilter, boolean skipRightFilter) {
    if (!skipLeftFilter) {
      leftSpeed = this.inputFilter1.get(leftSpeed);
    }

    if (!skipRightFilter) {
      rightSpeed = this.inputFilter2.get(rightSpeed);
    }

    diffDrive.tankDrive(leftSpeed, rightSpeed, false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L Drive", getLeftDistanceFeet());
    SmartDashboard.putNumber("R Drive", getRightDistanceFeet());
    SmartDashboard.putNumber("Heading", getHeadingDegrees());
    SmartDashboard.putNumber("Pitch", getPitchDegrees());
  }
}
