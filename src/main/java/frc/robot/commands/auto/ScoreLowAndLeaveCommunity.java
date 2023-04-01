/* 2023 Written by Alumiboti FRC 5590 */
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class ScoreLowAndLeaveCommunity extends SequentialCommandGroup {

    public ScoreLowAndLeaveCommunity(Drivetrain drivetrain, Intake intake) {
        addCommands(new RunCommand(() -> intake.setIntakeSpeed(-.2), intake)
                .withTimeout(1)
                .andThen(new StraightDrive(drivetrain, () -> -.4).withTimeout(4.3)));
    }
}
