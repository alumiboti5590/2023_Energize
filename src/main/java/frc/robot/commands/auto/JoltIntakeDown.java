package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.subsystems.Drivetrain;

public class JoltIntakeDown extends SequentialCommandGroup {

  private double JOLT_SPEED = -.3;

  public JoltIntakeDown(Drivetrain drivetrain) {
    addCommands(new StraightDrive(drivetrain, () -> JOLT_SPEED).withTimeout(.2));
  }
}
