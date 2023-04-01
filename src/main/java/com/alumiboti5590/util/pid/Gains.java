/* 2023 Written by Alumiboti FRC 5590 */
package com.alumiboti5590.util.pid;

// Directly from
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop/src/main/java/frc/robot/Gains.java
public class Gains {

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final int kIzone;
    public final double kPeakOutput;

    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kPeakOutput = _kPeakOutput;
    }
}
