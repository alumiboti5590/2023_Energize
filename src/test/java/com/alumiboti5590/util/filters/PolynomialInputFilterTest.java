package com.alumiboti5590.util.filters;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class PolynomialInputFilterTest {

  @Test
  void filtersPositiveNumbers() {
    double value = .2;
    int scaleFactor = 2;
    double expectedFilterValue = .04;

    PolynomialInputFilter filter = new PolynomialInputFilter(scaleFactor);
    double actualFilterValue = filter.get(value);
    assertEquals(
        expectedFilterValue,
        actualFilterValue,
        .001,
        String.format("%2f %2f", expectedFilterValue, actualFilterValue));
  }

  @Test
  void filtersNegativeNumbers() {
    double value = -.2;
    int scaleFactor = 2;
    double expectedFilterValue = -0.04;

    PolynomialInputFilter filter = new PolynomialInputFilter(scaleFactor);
    double actualFilterValue = filter.get(value);
    assertEquals(
        expectedFilterValue,
        actualFilterValue,
        .001,
        String.format("%2f %2f", expectedFilterValue, actualFilterValue));
  }
}
