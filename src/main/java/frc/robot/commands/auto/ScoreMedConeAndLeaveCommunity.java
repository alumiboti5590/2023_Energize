/* 2023 Written by Alumiboti FRC 5590 */
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Grabber.GrabMode;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shoulder.ShoulderPosition;

public class ScoreMedConeAndLeaveCommunity extends SequentialCommandGroup {

    public ScoreMedConeAndLeaveCommunity(Drivetrain drivetrain, Shoulder shoulder, Arm arm, Grabber grabber) {
        addCommands(new JoltIntakeDown(drivetrain)
                .andThen(new RunCommand(() -> shoulder.setGoalPosition(ShoulderPosition.HALFWAY), shoulder))
                .andThen(new RunCommand(() -> arm.setGoalPosition(10), arm))
                .andThen(new RunCommand(() -> grabber.setGrabMode(GrabMode.OPEN), grabber)));
    }
}
