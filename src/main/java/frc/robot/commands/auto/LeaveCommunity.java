/* 2023 Written by Alumiboti FRC 5590 */
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.subsystems.Drivetrain;

public class LeaveCommunity extends SequentialCommandGroup {

    private static final double SPEED = .5;
    private static final double TIMEOUT = 3;

    public LeaveCommunity(Drivetrain drivetrain) {
        addCommands(new JoltIntakeDown(drivetrain)
                .andThen(new StraightDrive(drivetrain, () -> SPEED).withTimeout(TIMEOUT)));
    }
}
