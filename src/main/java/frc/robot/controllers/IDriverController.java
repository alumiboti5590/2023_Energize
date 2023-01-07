package frc.robot.controllers;

import com.alumiboti5590.util.Tuple;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * IDriverController defines the interface commands needed by the primary (driver) controller. This
 * is responsible for moving the robot, and essential operations that occur on the primary
 * controller
 */
public interface IDriverController {

  /**
   * Return values for arcade/curvature drive - Speed + Rotation First element in tuple is speed,
   * Second element is rotation
   */
  public Tuple<Double, Double> getArcadeOrCurvatureDriveValues();

  /**
   * Return values for tank drive - Left + Right Speed First element in tuple is Left speed, Second
   * element is Right speed
   */
  public Tuple<Double, Double> getTankDriveValues();

  public boolean getCurvatureDriveQuickTurn();

  // Define buttons
  public Trigger getTurboButton();
}
