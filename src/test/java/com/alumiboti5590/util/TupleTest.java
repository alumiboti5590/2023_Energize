/* 2023 Written by Alumiboti FRC 5590 */
package com.alumiboti5590.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class TupleTest {

    @Test
    void keepsValuesOrdered() {
        double firstNum = 55;
        double secondNum = 90;
        Tuple<Double, Double> exampleTuple = new Tuple<Double, Double>(firstNum, secondNum);
        assertEquals(firstNum, exampleTuple.first);
        assertEquals(secondNum, exampleTuple.second);
    }

    @Test
    void respectsEqualityWhenEqual() {
        double firstNum = 55;
        double secondNum = 90;
        Tuple<Double, Double> exampleTuple = new Tuple<Double, Double>(firstNum, secondNum);
        Tuple<Double, Double> otherButEqualTuple = new Tuple<Double, Double>(firstNum, secondNum);
        assertTrue(exampleTuple.equals(otherButEqualTuple), "expected two tuples to match with same values");
    }

    @Test
    void respectsEqualityWhenUnequal() {
        double firstNum = 55;
        double secondNum = 90;
        Tuple<Double, Double> exampleTuple = new Tuple<Double, Double>(firstNum, secondNum);
        Tuple<Double, Double> otherButEqualTuple = new Tuple<Double, Double>(secondNum, firstNum);
        assertFalse(exampleTuple.equals(otherButEqualTuple), "expected two tuples to not match with different values");
    }
}
