package com.alumiboti5590.util;

public class DistanceUtility {

  private static double inchToCmRatio = 2.54;

  public static double inchesToCentimeters(double inches) {
    return inches * inchToCmRatio;
  }

  public static double centimetersToInches(double centimeters) {
    return centimeters / inchToCmRatio;
  }

  public static double centimetersToMeters(double centimeters) {
    return centimeters / 100;
  }

  public static double metersToCentimeters(double centimeters) {
    return centimeters * 100;
  }
}
