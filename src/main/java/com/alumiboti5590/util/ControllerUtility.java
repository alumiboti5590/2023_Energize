package com.alumiboti5590.util;

public class ControllerUtility {

  public static double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
}
