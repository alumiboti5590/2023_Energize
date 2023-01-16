package com.alumiboti5590.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class DistanceUtilityTest {

  @Test
  void validInchesToCentimeters() {
    double inches = 4;
    double expected = 10.16;
    double actual = DistanceUtility.inchesToCentimeters(inches);

    assertEquals(expected, actual, .01);
  }

  @Test
  void validCentimetersToInches() {
    double centimeters = 10;
    double expected = 3.94;
    double actual = DistanceUtility.centimetersToInches(centimeters);

    assertEquals(expected, actual, .01);
  }

  @Test
  void validCentimetersToMeters() {
    double centimeters = 230;
    double expected = 2.30;
    double actual = DistanceUtility.centimetersToMeters(centimeters);

    assertEquals(expected, actual, .001);
  }
}
