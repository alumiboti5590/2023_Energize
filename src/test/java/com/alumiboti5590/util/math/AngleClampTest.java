/* 2023 Written by Alumiboti FRC 5590 */
package com.alumiboti5590.util.math;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class AngleClampTest {

    @Test
    void normalPositiveValue() {
        double value = 117;
        double expectedOutput = 117;

        double actual = AngleClamp.navXClamp(value);
        assertEquals(expectedOutput, actual);
    }

    @Test
    void normalNegativeValue() {
        double value = -117;
        double expectedOutput = -117;

        double actual = AngleClamp.navXClamp(value);
        assertEquals(expectedOutput, actual);
    }

    @Test
    void oneRotationPositiveValue() {
        double value = 477;
        double expectedOutput = 117;

        double actual = AngleClamp.navXClamp(value);
        assertEquals(expectedOutput, actual);
    }

    @Test
    void oneRotationNegativeValue() {
        double value = -477;
        double expectedOutput = -117;

        double actual = AngleClamp.navXClamp(value);
        assertEquals(expectedOutput, actual);
    }

    @Test
    void halfRotationPositiveValue() {
        double value = 297;
        double expectedOutput = -63;

        double actual = AngleClamp.navXClamp(value);
        assertEquals(expectedOutput, actual);
    }

    @Test
    void halfRotationNegativeValue() {
        double value = -297;
        double expectedOutput = 63;

        double actual = AngleClamp.navXClamp(value);
        assertEquals(expectedOutput, actual);
    }
}
