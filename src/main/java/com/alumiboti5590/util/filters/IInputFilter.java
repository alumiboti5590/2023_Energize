/* 2023 Written by Alumiboti FRC 5590 */
package com.alumiboti5590.util.filters;

public interface IInputFilter {
    /**
     * Return a filtered value given some input
     *
     * @param input
     * @return Filtered value
     */
    public double get(double input);
}
