package frc.robot.controllers.hardware;

import com.alumiboti5590.util.ControllerUtility;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An extension of the XboxController class that comes from WPI, but it includes some extra buttons
 * and axes as Triggers and helpful utilities
 */
public class CustomXBoxController extends XboxController {

  protected Trigger leftBumperTrigger, rightBumperTrigger;

  // Deadzone is the amount that an axis needs to be pushed before its registered
  protected double deadzone = 0;

  public CustomXBoxController(int port) {
    this(port, 0);
  }

  public CustomXBoxController(int port, double deadzone) {
    super(port);
    this.deadzone = deadzone;
    leftBumperTrigger = new Trigger(this::getLeftBumper);
    rightBumperTrigger = new Trigger(this::getRightBumper);
  }

  protected double handleDeadband(double val, double deadband) {
    return ControllerUtility.handleDeadband(val, deadband);
  }

  public double getDeadzone() {
    return this.deadzone;
  }
}
