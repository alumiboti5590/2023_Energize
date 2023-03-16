package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class ScoreLowAndLeaveCommunity extends SequentialCommandGroup {

  public ScoreLowAndLeaveCommunity(Drivetrain drivetrain, Intake intake) {
    addCommands(
        new JoltIntakeDown(drivetrain)
            .andThen(new StraightDrive(drivetrain, () -> .2).withTimeout(.2))
            .andThen(new RunCommand(() -> intake.setIntakeSpeed(.5), intake).withTimeout(.5))
            .andThen(new StraightDrive(drivetrain, () -> -.4).withTimeout(4)));
  }
}
