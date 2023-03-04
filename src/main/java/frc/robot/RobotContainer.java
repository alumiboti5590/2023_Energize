// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.controllers.IDriverController;
import frc.robot.controllers.IOperatorController;
import frc.robot.controllers.XboxDriverController;
import frc.robot.controllers.XboxOperatorController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Grabber.GrabMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;

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
  private Grabber grabber;
  private Intake intake;
  private Shoulder shoulder;

  // Autonomous Command selector
  // This allows us to pick from a drop down of different autonomous commands
  private SendableChooser<Command> autoChooser;

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
  }

  /** Initialize subsystems across the robot */
  private void configureSubsystems() {
    this.drivetrain = new Drivetrain();
    // this.arm = new Arm();
    this.intake = new Intake();
    this.shoulder = new Shoulder();
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

    operatorController
        .getGrabOpen()
        .whileTrue(new RunCommand(() -> this.grabber.setGrabMode(GrabMode.OPEN), this.grabber));
    operatorController
        .getGrabClose()
        .whileTrue(new RunCommand(() -> this.grabber.setGrabMode(GrabMode.CLOSE), this.grabber));
  }

  /**
   * Configure the default command(s) for the robot. At the present time, the only default command
   * is for the drivetrain to continuously be allowing the robot to move around the field.
   */
  private void configureDefaultCommands() {
    // Allows for dynamically controlling the drivetrain with the drive controller
    this.drivetrain.setDefaultCommand(
        new RunCommand(() -> drivetrain.controllerDrive(driverController), drivetrain));

    // Runs the intake at the percentage given by the controller
    this.intake.setDefaultCommand(
        new RunCommand(() -> intake.setIntakeSpeed(operatorController.getIntakeSpeed()), intake));

    // Allows the operator controller to modify and adjust the arm position in small increments
    this.shoulder.setDefaultCommand(
        new RunCommand(
            () -> shoulder.controllerAction(operatorController.getShoulderModifier()), shoulder));
  }

  /**
   * Configure a SendableChooser with a list of potential autonomous actions we want to perform One
   * of these options will be selected by the drive team before initiating autonomous, where the
   * robot will then select & follow through with it.
   */
  private void configureAutoCommands() {
    autoChooser = new SendableChooser<>();
    // TODO: add choices here once we know what autonomous actions we want to
    // perform
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
