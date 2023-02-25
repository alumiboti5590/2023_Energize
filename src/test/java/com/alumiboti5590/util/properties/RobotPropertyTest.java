package com.alumiboti5590.util.properties;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class RobotPropertyTest {

  private static final String FULL = "unit_testing/full", MISSING = "unit_testing/missing";

  @Test
  void valueWithInt() {
    RobotProperties.UNSAFE_setSingleton(FULL);
    int expected =
        Integer.parseInt(RobotProperties.getInstance().getProperty("drivetrain_left_follower_id"));
    int actual = RobotProperty.DRIVETRAIN_LEFT_FOLLOWER_ID.getInteger();
    assertEquals(expected, actual);
  }

  @Test
  void valueWithDouble() {
    RobotProperties.UNSAFE_setSingleton(FULL);
    double expected =
        Double.parseDouble(RobotProperties.getInstance().getProperty("standard_drive_speed"));
    double actual = RobotProperty.STANDARD_DRIVE_SPEED.getDouble();
    assertEquals(expected, actual);
  }

  @Test
  void valueWithBoolean() {
    RobotProperties.UNSAFE_setSingleton(FULL);
    boolean expected =
        Boolean.parseBoolean(RobotProperties.getInstance().getProperty("intake_motor_invert"));
    boolean actual = RobotProperty.INTAKE_MOTOR_INVERT.getBoolean();
    assertEquals(expected, actual);
  }

  @Test
  void invalidTypeCast() {
    RobotProperties.UNSAFE_setSingleton(FULL);
    assertThrows(Exception.class, () -> RobotProperty.INTAKE_MOTOR_INVERT.getInteger());
  }

  @Test
  void missingProperties() {
    assertThrows(Exception.class, () -> RobotProperties.UNSAFE_setSingleton(MISSING));
  }
}
