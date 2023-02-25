package frc.robot.controllers.hardware;

import frc.robot.controllers.IOperatorController;

public class XboxOperatorController extends CustomXBoxController implements IOperatorController {

  public XboxOperatorController(int port, double deadzone) {
    super(port, deadzone);
  }

  @Override
  public double getIntakePercentage() {
    return 0; // TODO: fix me
  }
}
