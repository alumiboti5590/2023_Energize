package frc.robot.controllers;

import frc.robot.controllers.hardware.CustomXBoxController;

public class XboxOperatorController extends CustomXBoxController implements IOperatorController {

  public XboxOperatorController(int port, double deadzone) {
    super(port, deadzone);
  }

  @Override
  public double getIntakeSpeed() {
    // A positive (towards user) is input, which kinda makes sense
    return this.handleDeadband(this.getLeftY(), this.deadzone);
  }

  @Override
  public double getShoulderModifier() {
    // Negate so that pushing stick away from you is +, meaning raise the arm
    return -this.handleDeadband(this.getRightY(), this.deadzone);
  }
}
