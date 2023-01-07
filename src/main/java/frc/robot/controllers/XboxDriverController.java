package frc.robot.controllers;

import com.alumiboti5590.util.Tuple;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.hardware.CustomXBoxController;

public class XboxDriverController extends CustomXBoxController implements IDriverController {

  public XboxDriverController(int port, double deadzone) {
    super(port, deadzone);
  }

  @Override
  public Tuple<Double, Double> getArcadeOrCurvatureDriveValues() {
    double speed =
        this.handleDeadband(this.getRawAxis(XboxController.Axis.kLeftY.value), this.deadzone);
    double rotation =
        this.handleDeadband(this.getRawAxis(XboxController.Axis.kRightX.value), this.deadzone);

    // Negate the speed since "pushing the stick forward" is actually a negative
    // value
    return new Tuple<Double, Double>(-speed, rotation);
  }

  @Override
  public Tuple<Double, Double> getTankDriveValues() {
    double leftSpeed =
        this.handleDeadband(this.getRawAxis(XboxController.Axis.kLeftY.value), this.deadzone);
    double rightSpeed =
        this.handleDeadband(this.getRawAxis(XboxController.Axis.kRightY.value), this.deadzone);

    // Negate the speed since "pushing the stick forward" is actually a negative
    // value
    return new Tuple<Double, Double>(-leftSpeed, -rightSpeed);
  }

  @Override
  public boolean getCurvatureDriveQuickTurn() {
    return this.rightBumperTrigger.getAsBoolean();
  }

  @Override
  public Trigger getTurboButton() {
    return this.leftBumperTrigger;
  }
}
