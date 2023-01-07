package com.alumiboti5590.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class ControllerUtilityTest {

  @Test
  void valueWithinDeadband() {
    double value = .1;
    double deadband = .2;
    double expectedOutput = 0;

    double actual = ControllerUtility.handleDeadband(value, deadband);
    assertEquals(expectedOutput, actual);
  }

  @Test
  void valueOutsideDeadband() {
    double value = .4;
    double deadband = .2;
    double expectedOutput = .4;

    double actual = ControllerUtility.handleDeadband(value, deadband);
    assertEquals(expectedOutput, actual);
  }
}
