package com.alumiboti5590.util.filters;

public class PolynomialInputFilter implements IInputFilter {
  private int order;

  public PolynomialInputFilter(int order) {
    this.order = order;
  }

  @Override
  public double get(double input) {
    boolean isNegative = (input < 0.0);

    // Careful with exponents. Only consider positive values and
    // mirror accordingly so that graph is symmetrical around the
    // y = -x line

    if (isNegative) {
      input = -input;
    }

    double result = Math.pow(input, order);

    if (isNegative) {
      result = -result;
    }
    return result;
  }
  
}