package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.subsystems.Drivetrain;

public class LeaveCommunity extends SequentialCommandGroup {

  private static final double SPEED = .3;
  private static final double TIMEOUT = 2;

  public LeaveCommunity(Drivetrain drivetrain) {
    addCommands(new StraightDrive(drivetrain, () -> SPEED).withTimeout(TIMEOUT));
  }
}
