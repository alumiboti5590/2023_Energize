package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * IOperatorController defines the interface commands needed by the secondary (operator) controller.
 * This is responsible for any additional subsystems that are too difficult for the drivetrain
 * operator to manage.
 */
public interface IOperatorController {

  // TODO: replace this with a real button that is required by the robot
  public Trigger getExampleButton();
}
