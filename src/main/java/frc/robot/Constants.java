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
  public static final double XBOX_CONTROLLER_DEADBAND = 0.1;

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
    public static final double GEAR_BOX_RATIO = 7.31; //  1.45; 10.71; 12.75;
    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double ENCODER_COUNTS_PER_ROTATION = 96;

    public static double metersPerEncoderPulse() {
      double wheelsDiameterAsCm = DistanceUtility.inchesToCentimeters(WHEEL_DIAMETER_INCHES);
      double beforeGearBoxReduction = 100 / (wheelsDiameterAsCm * Math.PI); // 100 == 100cm to 1 m
      double afterGearBoxReduction = beforeGearBoxReduction / GEAR_BOX_RATIO;
      return afterGearBoxReduction;
    }

    // Used to control the scaled maximum speed that the robot should have
    // under regular circumstances. This can be overriden to full by using
    // the "Turbo" button
    public static final double STANDARD_DRIVE_SPEED_SCALAR = 0.7;

    // Used to control the maximum speed that the robot should have when
    // the "Turbo" button is activated.
    public static final double TURBO_DRIVE_SPEED_SCALAR = 1.0;

    // Motor CAN ports
    public static final class Motors {
      public static final int LEFT_LEADER_ID = 1;
      public static final int LEFT_FOLLOWER_ID = 5;

      public static final int RIGHT_LEADER_ID = 12;
      public static final int RIGHT_FOLLOWER_ID = 15;
    }

    // Encoder DIO ports
    public static final class Encoders {
      public static final int LEFT_CHANNEL_A = 5;
      public static final int LEFT_CHANNEL_B = 6;
      public static final int RIGHT_CHANNEL_A = 7;
      public static final int RIGHT_CHANNEL_B = 8;
    }

    public static final boolean INVERT_STEERING = true;

    public static final boolean LEFT_LEADER_INVERT = false;
    public static final boolean RIGHT_LEADER_INVERT = true;

    public static final boolean LEFT_FOLLOWER_INVERT = false;
    public static final boolean RIGHT_FOLLOWER_INVERT = true;
  }

  // All variables associated with the Intake subsystem
  public static final class Intake {
    public static final class Motors {
      public static final int INTAKE_ID = 10;
    }

    public static final boolean INTAKE_MOTOR_INVERT = false;

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
    public static final Gains PID_GAINS = new Gains(0.25, 0.001, 20, 1023.0 / 7200.0, 300, 1.00);

    // See the following URL to determine these values
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop/src/main/java/frc/robot/Robot.java#L132-L139
    public static final class VelocityRPM {
      public static final double CONE_INTAKE = 0.0;
      public static final double CUBE_INTAKE = 0.0;
      public static final double REVERSE = -0.0;
    }
  }
}
