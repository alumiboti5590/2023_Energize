// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.auto.JoltIntakeDown;
import frc.robot.commands.auto.LeaveCommunity;
import frc.robot.commands.auto.ScoreLowAndLeaveCommunity;
import frc.robot.commands.auto.ScoreMedConeAndLeaveCommunity;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.controllers.IDriverController;
import frc.robot.controllers.IOperatorController;
import frc.robot.controllers.XboxDriverController;
import frc.robot.controllers.XboxOperatorController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Grabber.GrabMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shoulder.ShoulderPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Controller interfaces that allow us to control the robot
  // through different means
  private IDriverController driverController;
  private IOperatorController operatorController;

  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain;
  private Arm arm;
  private Grabber grabber;
  private Intake intake;
  private Shoulder shoulder;

  // Autonomous Command selector
  // This allows us to pick from a drop down of different autonomous commands
  private SendableChooser<Command> autoChooser;

  boolean isAuto = false;

  public RobotContainer() {
    // 1. Configure subsystems that are used by the robot
    configureSubsystems();

    // 2. Configure the button bindings and operator controls
    configureBindings();

    // 3. Configure the default command (probably just to drive the robot)
    configureDefaultCommands();

    // 4. Configure the autonomous command to run based on what is selected
    // in the SendableChooser
    configureAutoCommands();

    this.shoulder.setBrakeMode(false);
  }

  /** Initialize subsystems across the robot */
  private void configureSubsystems() {
    this.drivetrain = new Drivetrain();
    this.arm = new Arm();
    this.grabber = new Grabber();
    this.intake = new Intake();

    this.shoulder =
        new Shoulder(arm::armExtendedToUnsafeMoveDistance, arm::getShoulderFeedForwardMultiplier);
  }

  /** Configure trigger & axis bindings between the robot and the controllers */
  private void configureBindings() {
    driverController =
        new XboxDriverController(
            Constants.Controller.DRIVER_CONTROLLER_PORT,
            Constants.Controller.XBOX_CONTROLLER_DEADBAND);
    operatorController =
        new XboxOperatorController(
            Constants.Controller.OPERATOR_CONTROLLER_PORT,
            Constants.Controller.XBOX_CONTROLLER_DEADBAND);

    // --------------------------
    // Driver Controller Bindings
    // --------------------------

    driverController
        .getStraightDrive()
        .whileTrue(
            new StraightDrive(drivetrain, () -> driverController.getTankDriveValues().first));

    // ----------------------------
    // Operator Controller Bindings
    // ----------------------------

    // Grabber Controls
    // ~~~~~~~~~~~~~~~~
    operatorController
        .getGrabOpen()
        .whileTrue(run(() -> this.grabber.setGrabMode(GrabMode.OPEN), this.grabber));
    operatorController
        .getGrabClose()
        .whileTrue(run(() -> this.grabber.setGrabMode(GrabMode.CLOSE), this.grabber));

    // Shoulder Controls
    // ~~~~~~~~~~~~~~~~~

    operatorController
        .getShoulderZero()
        .and(operatorController::getGoalModifier)
        .whileTrue(
            run(
                () -> {
                  this.shoulder.setGoalPosition(ShoulderPosition.ZERO);
                  this.grabber.setGrabMode(GrabMode.CLOSE);
                },
                this.shoulder,
                this.grabber));

    operatorController
        .getShoulderZero()
        .whileTrue(
            run(
                () -> {
                  this.shoulder.setGoalPosition(ShoulderPosition.ZERO);
                  this.grabber.setGrabMode(GrabMode.OPEN);
                },
                this.shoulder,
                this.grabber));

    // CUBEs
    // -----

    operatorController
        .getLowGoal()
        .whileTrue(
            run(() -> this.shoulder.setGoalPosition(ShoulderPosition.LOW_CUBE), this.shoulder));
    operatorController
        .getMedGoal()
        .whileTrue(
            run(() -> this.shoulder.setGoalPosition(ShoulderPosition.MED_CUBE), this.shoulder));
    operatorController
        .getHighGoal()
        .whileTrue(
            run(() -> this.shoulder.setGoalPosition(ShoulderPosition.HIGH_CUBE), this.shoulder));

    // CONEs
    // -----

    operatorController
        .getLowGoal()
        .and(operatorController::getGoalModifier)
        .whileTrue(
            run(() -> this.shoulder.setGoalPosition(ShoulderPosition.LOW_CONE), this.shoulder));
    operatorController
        .getMedGoal()
        .and(operatorController::getGoalModifier)
        .whileTrue(
            run(() -> this.shoulder.setGoalPosition(ShoulderPosition.LOW_CONE), this.shoulder));
    operatorController
        .getHighGoal()
        .and(operatorController::getGoalModifier)
        .whileTrue(
            run(() -> this.shoulder.setGoalPosition(ShoulderPosition.LOW_CONE), this.shoulder));

    operatorController
        .getShoulderZeroMode()
        .whileTrue(run(() -> this.shoulder.startZeroingMode(), this.shoulder));

    operatorController
        .getArmZeroMode()
        .whileTrue(
            run(
                () -> {
                  this.grabber.setGrabMode(GrabMode.CLOSE);
                  this.arm.startZeroingMode();
                },
                this.arm));
  }

  /**
   * Configure the default command(s) for the robot. At the present time, the only default command
   * is for the drivetrain to continuously be allowing the robot to move around the field.
   */
  private void configureDefaultCommands() {
    // Allows for dynamically controlling the drivetrain with the drive controller
    // using the two sticks in either Tank Drive or Arcade drive mode
    setDefaultCommand(drivetrain, () -> drivetrain.controllerDrive(driverController, isAuto));

    // Runs the intake at the percentage given by the controller
    setDefaultCommand(intake, () -> intake.setIntakeSpeed(operatorController.getIntakeSpeed()));

    // Allows the operator controller to modify and adjust the arm position in small increments
    setDefaultCommand(arm, () -> arm.controllerAction(operatorController.getArmModifier()));

    // Allows the operator controller to modify and adjust the arm position in small increments
    setDefaultCommand(
        shoulder,
        () ->
            shoulder.controllerAction(
                operatorController.getShoulderModifier(),
                operatorController.getShoulderSafetyOverride()));
  }

  /**
   * Configure a SendableChooser with a list of potential autonomous actions we want to perform One
   * of these options will be selected by the drive team before initiating autonomous, where the
   * robot will then select & follow through with it.
   */
  private void configureAutoCommands() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", run(() -> drivetrain.tankDrive(0, 0), drivetrain));
    autoChooser.addOption("Jolt Intake", new JoltIntakeDown(drivetrain));
    autoChooser.addOption("Leave Community", new LeaveCommunity(drivetrain));
    autoChooser.addOption(
        "Score Low & Leave Community", new ScoreLowAndLeaveCommunity(drivetrain, intake));
    autoChooser.addOption(
        "Score Med Cone & Leave Community",
        new ScoreMedConeAndLeaveCommunity(drivetrain, shoulder, arm, grabber));

    // Place on the dashboard
    SmartDashboard.putData("Auto Command", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getTestCommand() {
    return null;
  }

  private RunCommand run(Runnable runnable, Subsystem... subsystem) {
    return new RunCommand(runnable, subsystem);
  }

  private void setDefaultCommand(Subsystem subsystem, Runnable runnable) {
    subsystem.setDefaultCommand(run(runnable, subsystem));
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
  }
}
