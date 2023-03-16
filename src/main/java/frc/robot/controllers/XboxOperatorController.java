package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.hardware.CustomXBoxController;

public class XboxOperatorController extends CustomXBoxController implements IOperatorController {

  public XboxOperatorController(int port, double deadzone) {
    super(port, deadzone);
  }

  @Override
  public double getIntakeSpeed() {
    if (this.getLeftTriggerAxis() > 0) {
      return this.getLeftTriggerAxis();
    }
    if (this.getRightTriggerAxis() > 0) {
      return -this.getRightTriggerAxis();
    }
    return 0;
    // A positive (towards user) is input, which kinda makes sense
    // return this.handleDeadband(this.getLeftY(), this.deadzone);
  }

  @Override
  public double getShoulderModifier() {
    // Negate so that pushing stick away from you is +, meaning raise the arm
    return -this.handleDeadband(this.getRightY(), this.deadzone);
  }

  @Override
  public double getArmModifier() {
    return -this.handleDeadband(this.getLeftY(), this.deadzone) / 5;
  }

  @Override
  public Trigger getGrabOpen() {
    return new Trigger(this::getLeftBumper);
  }

  @Override
  public Trigger getGrabClose() {
    return new Trigger(this::getRightBumper);
  }

  @Override
  public Trigger getShoulderZero() {
    return new Trigger(this::getAButton);
  }

  @Override
  public Trigger getArmZeroMode() {
    return new Trigger(this::getStartButton);
  }

  @Override
  public Trigger getShoulderZeroMode() {
    return new Trigger(this::getBackButton);
  }

  @Override
  public Trigger getLowGoal() {
    return new Trigger(this::getXButton);
  }

  @Override
  public Trigger getMedGoal() {
    return new Trigger(this::getYButton);
  }

  @Override
  public Trigger getHighGoal() {
    return new Trigger(this::getBButton);
  }

  @Override
  public boolean getGoalModifier() {
    return this.getPOV() == 270;
  }
}
