// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.alumiboti5590.util.DistanceUtility;
import com.alumiboti5590.util.pid.Gains;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // All variables associated with the Controller subsystems
  public static final class Controller {
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
    public static final double XBOX_CONTROLLER_DEADBAND = 0.2;
  }

  // All variables associated with the Drivetrain subsystem
  public static final class Drivetrain {
    // How fast does the motor try to get up to the desired speed. [0, 1.0]
    // A higher value means slower, but more controlled acceleration, whereas
    // a smaller value means faster but more chaotic acceleration.
    public static final double RAMP_RATE_SECONDS = 0.3;

    // Limit the motors to 50A to prevent brownouts, trips, or too much
    // power being drawn when it really isn't needed.
    // This is a nice safety feature that is a tried & tested value.
    public static final int CURRENT_LIMIT = 50;

    // Hardware/Mechanical Constants
    public static final double GEAR_BOX_RATIO = 7.31, //  1.45; 10.71; 12.75;
        WHEEL_DIAMETER_INCHES = 6,
        ENCODER_COUNTS_PER_ROTATION = 96;

    public static double metersPerEncoderPulse() {
      double wheelsDiameterAsCm = DistanceUtility.inchesToCentimeters(WHEEL_DIAMETER_INCHES);
      double beforeGearBoxReduction = 100 / (wheelsDiameterAsCm * Math.PI); // 100 == 100cm to 1 m
      double afterGearBoxReduction = beforeGearBoxReduction / GEAR_BOX_RATIO;
      return afterGearBoxReduction / 100;
    }

    public static final Gains STRAIGHT_PID = new Gains(.05, 0, .1, 0, 0, 0);
    public static final double STRAIGHT_MAX_TURN_ROTATION = .3;
  }

  // All variables associated with the Intake subsystem
  public static final class Intake {
    public static final class Encoder {
      public static final FeedbackDevice ENCODER_TYPE = FeedbackDevice.QuadEncoder;

      // Which PID slot to pull gains from. Starting 2018, you can choose from
      // 0,1,2 or 3. Only the first two (0,1) are visible in web-based configuration.
      public static final int SLOT_ID = 0;

      // Set to zero to skip waiting for confirmation, set to nonzero to wait
      // and report to DS if action fails.
      public static final int TIMEOUT_MS = 30;

      public static final int TICKS_PER_REV = 8192;
      public static final boolean INVERT_PHASE = false; // positive should be motor positive
    }

    // PID Gains may have to be adjusted based on the responsiveness of control loop.
    // kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100%
    // output
    public static final Gains PID_GAINS = new Gains(0.35, 0.001, 200, 1023.0 / 7200.0, 300, 1.00);

    // See the following URL to determine these values
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop/src/main/java/frc/robot/Robot.java#L132-L139
    public static final class VelocityRPM {
      public static final double CONE_INTAKE = 0.0, CUBE_INTAKE = 0.0, REVERSE = -0.0;
    }
  }

  public static final class Arm {
    // TODO: starting with a high number to ensure we dont drive the thing through itself
    public static final double ENCODER_CONVERSION_FACTOR = 1.0;

    public static final double MIN_POSITION = 0;
    public static final double MAX_POSITION = 22.25;

    public static final double PERCENTAGE_MAX = .3;
    public static final double PERCENTAGE_MIN = -.3;

    public static final Gains PID = new Gains(5e-5, 1e-6, 0, 0, 0, 0);
  }

  public static final class Shoulder {
    public static final double ENCODER_CONVERSION_FACTOR = 1.0;

    public static final double PERCENTAGE_MAX = .5;
    public static final double PERCENTAGE_MIN = -.2;

    public static final double FORWARD_FEED_UPWARDS = .2,
        ZEROING_SPEED = -.3,
        UPWARDS_ADDITIONAL_GOAL = .5,
        UPWARD_ERROR_TOLERANCE = .2,
        DOWNWARD_ERROR_TOLERANCE = .1;

    public static final double MIN_POSITION = 0;
    public static final double HALFWAY = 3.5;
    public static final double SAFE_MAX = 13;
    public static final double MAX_POSITION = 13.4;

    // public static final Gains PID = new Gains(2, 0.00020, 20, 0, 0, 0);
    public static final Gains PID = new Gains(5e-5, 1e-6, 0.0, 0.000356, 0, 0);
  }
}
