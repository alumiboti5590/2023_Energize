// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.alumiboti5590.util.Tuple;
import com.alumiboti5590.util.filters.IInputFilter;
import com.alumiboti5590.util.filters.PolynomialInputFilter;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controllers.IDriverController;

public class Drivetrain extends SubsystemBase {

  public enum DriveType {
    CURVATURE,
    ARCADE
  }

  private SendableChooser<DriveType> driveTypeChooser;

  private IInputFilter inputFilter1, inputFilter2;

  // Basic Drivetrain things
  private CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower;
  private RelativeEncoder leftEncoder, rightEncoder;
  private DifferentialDrive diffDrive;

  // Odometry and kinematics
  private AHRS navX;
  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  private final Field2d field2d = new Field2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Set up the motor controllers
    leftLeader = new CANSparkMax(Constants.Drivetrain.LEFT_LEADER_CAN_ID, MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.Drivetrain.LEFT_FOLLOWER_CAN_ID, MotorType.kBrushless);
    rightLeader = new CANSparkMax(Constants.Drivetrain.RIGHT_LEADER_CAN_ID, MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.Drivetrain.RIGHT_FOLLOWER_CAN_ID, MotorType.kBrushless);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Set up coast mode
    leftLeader.setIdleMode(IdleMode.kCoast);
    rightLeader.setIdleMode(IdleMode.kCoast);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftEncoder.setPositionConversionFactor(
        Constants.Drivetrain.DISTANCE_PER_WHEEL_REVOLUTION_FEET * Constants.Drivetrain.DRIVETRAIN_GEAR_REDUCTION);
    rightEncoder.setPositionConversionFactor(
        Constants.Drivetrain.DISTANCE_PER_WHEEL_REVOLUTION_FEET * Constants.Drivetrain.DRIVETRAIN_GEAR_REDUCTION);

    leftEncoder.setVelocityConversionFactor(Constants.Drivetrain.DISTANCE_PER_WHEEL_REVOLUTION_FEET
        * Constants.Drivetrain.DRIVETRAIN_GEAR_REDUCTION / 60.0);
    rightEncoder.setVelocityConversionFactor(Constants.Drivetrain.DISTANCE_PER_WHEEL_REVOLUTION_FEET
        * Constants.Drivetrain.DRIVETRAIN_GEAR_REDUCTION / 60.0);

    leftLeader.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

    // Set up AHRS
    navX = new AHRS(SPI.Port.kMXP);

    // TODO set current limit

    // Set motor inversions if necessary
    rightLeader.setInverted(true);

    // Set ramp rate if necessary
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

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.Drivetrain.TRACKWIDTH_INCHES));
    // TODO: Use a starting point to determine placement on the field for the start
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeadingDegrees()), 0, 0);

    // Set up SmartDashboard Things
    SmartDashboard.putData("Robot Position", field2d);

    driveTypeChooser = new SendableChooser<>();
    driveTypeChooser.setDefaultOption("Curvature", DriveType.CURVATURE);
    driveTypeChooser.addOption("Arcade", DriveType.ARCADE);
    SmartDashboard.putData("Drive Type", driveTypeChooser);
  }

  public void setRampRate(double rate) {
    this.leftLeader.setClosedLoopRampRate(rate);
    this.leftLeader.setOpenLoopRampRate(rate);
    this.rightLeader.setClosedLoopRampRate(rate);
    this.rightLeader.setOpenLoopRampRate(rate);
  }

  private void resetEncoders() {
    this.leftEncoder.setPosition(0.0);
    this.rightEncoder.setPosition(0.0);
  }

  public double getLeftDistanceFeet() {
    return this.leftEncoder.getPosition();
  }

  public double getRightDistanceFeet() {
    return this.rightEncoder.getPosition();
  }

  public double getLeftVelocityFPS() {
    return this.leftEncoder.getVelocity();
  }

  public double getRightVelocityFPS() {
    return this.rightEncoder.getVelocity();
  }

  public void resetOdometry() {
    resetEncoders();
  }

  public double getHeadingDegrees() {
    return navX.getYaw();
  }

  public void resetYaw() {
    navX.reset();
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public void resetPoseMeters(Rotation2d newRotation, Pose2d newPose) {
    resetOdometry();
    odometry.resetPosition(newRotation, this.leftEncoder.getPosition(), this.rightEncoder.getPosition(), newPose);
  }

  public void resetPoseMeters() {
    resetPoseMeters(Rotation2d.fromDegrees(0.0), new Pose2d());
  }

  // Drive methods
  public void controllerDrive(IDriverController controller) {
    DriveType type = driveTypeChooser.getSelected();
    Tuple<Double, Double> inputs;

    inputs = controller.getArcadeOrCurvatureDriveValues();
    double multiplier = controller.getTurboButton().getAsBoolean() ? 1.0
        : Constants.Drivetrain.STANDARD_DRIVE_SPEED_SCALAR;

    if (type == DriveType.ARCADE) {
      this.arcadeDrive(inputs.first * multiplier, inputs.second * multiplier);
    } else if (type == DriveType.CURVATURE) {
      this.curvatureDrive(inputs.first * multiplier, inputs.second * multiplier,
          controller.getCurvatureDriveQuickTurn());
    }
  }

  public void arcadeDrive(double speed, double rotation) {
    arcadeDrive(speed, rotation, false, false);
  }

  public void arcadeDrive(double speed, double rotation, boolean skipSpeedFilter, boolean skipRotationFilter) {
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

  public void curvatureDrive(double speed, double rotation, boolean isQuickTurn, boolean skipSpeedFilter,
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

  public void tankDrive(double leftSpeed, double rightSpeed, boolean skipLeftFilter, boolean skipRightFilter) {
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
    odometry.update(Rotation2d.fromDegrees(getHeadingDegrees()), Units.feetToMeters(getLeftDistanceFeet()),
        Units.feetToMeters(getRightDistanceFeet()));
    field2d.setRobotPose(getPoseMeters());

    SmartDashboard.putNumber("Left Distance Feet", getLeftDistanceFeet());
    SmartDashboard.putNumber("Right Distance Feet", getRightDistanceFeet());
    SmartDashboard.putNumber("Heading", getHeadingDegrees());
  }
}