package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStation extends SequentialCommandGroup {

  private static final double GOAL_PITCH = 0;

  private Drivetrain drivetrain;
  private PIDController balancePID;

  private double withinToleranceCounter = 0;

  public BalanceOnChargeStation(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    balancePID = new PIDController(0, 0, 0);
    balancePID.setSetpoint(0);

    addCommands(new StraightDrive(drivetrain, this::getSpeed, this::isBalanced));
  }

  public double getSpeed() {
    double currentPitch = this.drivetrain.getPitchDegrees();
    double rawPIDSpeed = balancePID.calculate(currentPitch, GOAL_PITCH);
    if (false) {
      rawPIDSpeed = .3;
    } else if (false) { // getting closer to 0
      rawPIDSpeed = .3;
    } else {
      rawPIDSpeed = 0;
    }
    return rawPIDSpeed;
  }

  /** Returns true if we are balanced for long enough */
  public boolean isBalanced() {
    if (balancePID.atSetpoint()) {
      withinToleranceCounter++;
    } else {
      withinToleranceCounter = 0;
    }
    return withinToleranceCounter >= 5;
  }
}
