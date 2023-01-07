package com.alumiboti5590.util.filters;

public class LinearInputFilter implements IInputFilter {
  private double scale;

  public LinearInputFilter(double scaleFactor) {
    this.scale = scaleFactor;
  }

  @Override
  public double get(double input) {
    return input * this.scale;
  }
}
