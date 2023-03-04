package frc.robot.controllers;

/**
 * IOperatorController defines the interface commands needed by the secondary (operator) controller.
 * This is responsible for any additional subsystems that are too difficult for the drivetrain
 * operator to manage.
 */
public interface IOperatorController {

  public double getIntakeSpeed();

  public double getShoulderModifier();
}
