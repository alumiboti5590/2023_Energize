package frc.robot.commands.drivetrain;

import com.alumiboti5590.util.pid.Gains;
import com.alumiboti5590.util.properties.RobotProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class StraightDrive extends CommandBase {

  public interface SpeedGetter {
    public double op();
  }

  private Drivetrain drivetrain;
  private PIDController straightDrivePID, distanceDrivePID;
  private SpeedGetter speedGetter;
  private double desiredDistance = Double.POSITIVE_INFINITY, desiredHeading;
  private int withinToleranceCounter = 0;

  public StraightDrive(Drivetrain drivetrain, SpeedGetter speedGetter) {
    this.drivetrain = drivetrain;
    this.speedGetter = speedGetter;

    // Straight drive uses a PID to determine the 'best' turn rotation
    // needed to get to the desired heading, and this initializes the PID
    Gains drivePID = Constants.Drivetrain.STRAIGHT_PID;
    straightDrivePID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    straightDrivePID.enableContinuousInput(-180, 180);

    distanceDrivePID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    distanceDrivePID.enableContinuousInput(-.4, .4);
  }

  public StraightDrive(Drivetrain drivetrain, SpeedGetter speedGetter, double desiredDistance) {
    this(drivetrain, speedGetter);
    this.desiredDistance = desiredDistance;
  }

  @Override
  public void initialize() {
    this.drivetrain.resetOdometry();
    this.desiredHeading = this.drivetrain.getHeadingDegrees();
  }

  @Override
  public void execute() {
    double speed = 0;
    if (this.speedGetter != null) {
      speed = this.speedGetter.op();
    } else {
      speed = distanceDrivePID.calculate(this.getDistance(), desiredDistance);
    }

    // Determine a value from the PID controller between the current heading (-180, 180)
    // and the desired heading.
    double rawSteering =
        straightDrivePID.calculate(drivetrain.getHeadingDegrees(), this.desiredHeading);

    // The robot gets jittery if we allow it to turn TOO fast, so we clamp the max value
    double absMaxRotation = Constants.Drivetrain.STRAIGHT_MAX_TURN_ROTATION;
    double clampedSteering = MathUtil.clamp(rawSteering, -absMaxRotation, absMaxRotation);

    // Based on how we mount the NavX / RoboRio, we might need to invert the value
    if (RobotProperty.DRIVETRAIN_INVERT_STEER_PID.getBoolean()) {
      clampedSteering *= -1;
    }
    this.drivetrain.arcadeDrive(speed, clampedSteering);
  }

  @Override
  public boolean isFinished() {
    if (desiredDistance == Double.POSITIVE_INFINITY) {
      return false;
    }
    double avgDistance = this.getDistance();
    double errTolerance = 1;
    double offBy = Math.abs(avgDistance - desiredDistance);
    if (offBy <= errTolerance) {
      withinToleranceCounter++;
    } else {
      withinToleranceCounter = 0;
    }

    return withinToleranceCounter >= 10;
  }

  public double getDistance() {
    return drivetrain.getLeftDistanceFeet() == 0
        ? drivetrain.getRightDistanceFeet()
        : drivetrain.getLeftDistanceFeet();
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.tankDrive(0, 0);
  }
}
