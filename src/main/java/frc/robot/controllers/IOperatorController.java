package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * IOperatorController defines the interface commands needed by the secondary (operator) controller.
 * This is responsible for any additional subsystems that are too difficult for the drivetrain
 * operator to manage.
 */
public interface IOperatorController {

  public Trigger getGrabOpen();

  public Trigger getGrabClose();

  public double getIntakeSpeed();

  public double getShoulderModifier();

  public double getArmModifier();

  public Trigger getShoulderZero();

  public Trigger getShoulderHalfway();

  public Trigger getShoulderMax();

  public Trigger getArmZeroMode();

  public Trigger getShoulderZeroMode();
}
