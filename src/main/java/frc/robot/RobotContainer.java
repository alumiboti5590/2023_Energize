// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.controllers.IDriverController;
import frc.robot.controllers.IOperatorController;
import frc.robot.controllers.XboxDriverController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

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
  private Intake intake;

  // Autonomous Command selector
  // This allows us to pick from a drop down of different autonomous commands
  private SendableChooser<Command> autoChooser;

  private SendableChooser<Double> intakeSpeedChooser;

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

    // TODO: delete this and update it with controller input instead
    intakeSpeedChooser = new SendableChooser<>();
    intakeSpeedChooser.setDefaultOption("0", 0.0);
    intakeSpeedChooser.addOption("25%", .25);
    intakeSpeedChooser.addOption("50%", .5);
    intakeSpeedChooser.addOption("75%", .75);
    intakeSpeedChooser.addOption("100%", 1.0);
    intakeSpeedChooser.addOption("-25%", -.25);
    intakeSpeedChooser.addOption("-50%", -.5);
    intakeSpeedChooser.addOption("-75%", -.75);
    intakeSpeedChooser.addOption("-100%", -1.0);

    SmartDashboard.putData("Intake Speed", intakeSpeedChooser);
    // TODO: </end> delete this and update it with controller input instead
  }

  /** Initialize subsystems across the robot */
  private void configureSubsystems() {
    this.drivetrain = new Drivetrain();
    this.intake = new Intake();
  }

  /** Configure trigger & axis bindings between the robot and the controllers */
  private void configureBindings() {
    driverController = new XboxDriverController(0, Constants.XBOX_CONTROLLER_DEADBAND);
  }

  /**
   * Configure the default command(s) for the robot. At the present time, the only default command
   * is for the drivetrain to continuously be allowing the robot to move around the field.
   */
  private void configureDefaultCommands() {
    RunCommand drivetrainCommand =
        new RunCommand(() -> this.drivetrain.controllerDrive(driverController), this.drivetrain);
    this.drivetrain.setDefaultCommand(drivetrainCommand);

    // TODO: this needs to be updated to use controller commands instead, but this is fine
    //       for testing until we know what we are really doing
    RunCommand intakeCommand =
        new RunCommand(
            () -> this.intake.setIntakeSpeed(-intakeSpeedChooser.getSelected()), this.intake);
    this.intake.setDefaultCommand(intakeCommand);
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
