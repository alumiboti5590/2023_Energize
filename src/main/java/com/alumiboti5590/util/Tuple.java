package com.alumiboti5590.util;

/**
 * Helper class that defines a Tuple (two numbers)
 *   These should just exist in Java, but they don't
 */
public class Tuple<X, Y> {
  public final X first;
  public final Y second;

  public Tuple(X x, Y y) {
    this.first = x;
    this.second = y;
  }
}