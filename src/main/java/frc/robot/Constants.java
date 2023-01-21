// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.alumiboti5590.util.DistanceUtility;

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
}
