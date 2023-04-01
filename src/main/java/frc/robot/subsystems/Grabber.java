/* 2023 Written by Alumiboti FRC 5590 */
package frc.robot.subsystems;

import com.alumiboti5590.util.properties.RobotProperty;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

    public enum GrabMode {
        OPEN,
        CLOSE;
    }

    DoubleSolenoid grabSolenoid;

    private DoubleSolenoid.Value GRAB_OPEN = DoubleSolenoid.Value.kForward, GRAB_CLOSE = DoubleSolenoid.Value.kReverse;

    public Grabber() {
        this.grabSolenoid = new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                RobotProperty.GRABBER_SOLENOID_A.getInteger(),
                RobotProperty.GRABBER_SOLENOID_B.getInteger());

        // Invert if open & close is backwards
        if (RobotProperty.GRABBER_INVERT.getBoolean()) {
            GRAB_OPEN = DoubleSolenoid.Value.kReverse;
            GRAB_CLOSE = DoubleSolenoid.Value.kReverse;
        }
    }

    public void setGrabMode(GrabMode grabMode) {
        this.grabSolenoid.set(grabMode == GrabMode.OPEN ? GRAB_OPEN : GRAB_CLOSE);
    }
}
