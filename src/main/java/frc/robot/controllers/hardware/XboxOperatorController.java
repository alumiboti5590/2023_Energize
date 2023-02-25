package frc.robot.controllers.hardware;

import frc.robot.controllers.IOperatorController;

public class XboxOperatorController extends CustomXBoxController implements IOperatorController {

  public XboxOperatorController(int port, double deadzone) {
    super(port, deadzone);
  }

  @Override
  public double getIntakeSpeed() {
    return this.getLeftY(); // A positive (towards user) is input, which kinda makes sense
  }
}
