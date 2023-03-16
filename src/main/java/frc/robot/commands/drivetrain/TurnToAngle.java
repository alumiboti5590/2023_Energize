package frc.robot.commands.drivetrain;

import com.alumiboti5590.util.math.AngleClamp;
import com.alumiboti5590.util.pid.Gains;
import com.alumiboti5590.util.properties.RobotProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

  public interface SpeedGetter {
    public double op();
  }

  private Drivetrain drivetrain;
  private PIDController straightDrivePID;
  private SpeedGetter speedGetter;
  private double desiredAngle;

  public TurnToAngle(Drivetrain drivetrain, SpeedGetter speedGetter, double desiredAngle) {
    this.drivetrain = drivetrain;
    this.speedGetter = speedGetter;
    this.desiredAngle = AngleClamp.navXClamp(desiredAngle);

    // Turn to angle drive uses a PID to determine the 'best' turn rotation
    // needed to get to the desired heading, and this initializes the PID
    Gains drivePID = Constants.Drivetrain.STRAIGHT_PID;
    straightDrivePID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    straightDrivePID.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    this.drivetrain.resetOdometry();
  }

  @Override
  public void execute() {
    // double speed = this.speedGetter.op();

    // Determine a value from the PID controller between the current heading (-180, 180)
    // and the desired heading.
    double rawSteering =
        straightDrivePID.calculate(drivetrain.getHeadingDegrees(), this.desiredAngle);

    // The robot gets jittery if we allow it to turn TOO fast, so we clamp the max value
    double absMaxRotation = Constants.Drivetrain.STRAIGHT_MAX_TURN_ROTATION;
    double clampedSteering = MathUtil.clamp(rawSteering, -absMaxRotation, absMaxRotation);

    // Based on how we mount the NavX / RoboRio, we might need to invert the value
    if (RobotProperty.DRIVETRAIN_INVERT_STEER_PID.getBoolean()) {
      clampedSteering *= -1;
    }
    this.drivetrain.curvatureDrive(0, clampedSteering, true);
  }

  @Override
  public boolean isFinished() {
    double avgDistance = (drivetrain.getLeftDistanceFeet() + drivetrain.getRightDistanceFeet()) / 2;
    double errTolerance = 1;
    double offBy = Math.abs(avgDistance - desiredAngle);
    return desiredAngle != Double.POSITIVE_INFINITY && offBy <= errTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.tankDrive(0, 0);
  }
}
