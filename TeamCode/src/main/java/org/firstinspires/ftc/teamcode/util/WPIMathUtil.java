package org.firstinspires.ftc.teamcode.util;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Math utility functions.
 */
public final class WPIMathUtil {
    private WPIMathUtil() {
        throw new AssertionError("utility class");
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * 026   * Returns value clamped between low and high boundaries.
     * 027   *
     * 028   * @param value Value to clamp.
     * 029   * @param low The lower boundary to which to clamp value.
     * 030   * @param high The higher boundary to which to clamp value.
     * 031   * @return The clamped value.
     * 032
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * 038   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * 039   * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     * 040   *
     * 041   * @param value Value to clip.
     * 042   * @param deadband Range around zero.
     * 043   * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * 044   * @return The value after the deadband is applied.
     * 045
     */
    public static double applyDeadband(double value, double deadband, double maxMagnitude) {
        if (Math.abs(value) > deadband) {
            if (maxMagnitude / deadband > 1.0e12) {
                // If max magnitude is sufficiently large, the implementation encounters
                // roundoff error.  Implementing the limiting behavior directly avoids
                // the problem.
                return value > 0.0 ? value - deadband : value + deadband;
            }
            if (value > 0.0) {
                // Map deadband to 0 and map max to max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                // x₁ = deadband
                // y₁ = 0
                // x₂ = max
                // y₂ = max
                //
                // y = (max - 0)/(max - deadband) (x - deadband) + 0
                // y = max/(max - deadband) (x - deadband)
                // y = max (x - deadband)/(max - deadband)
                return maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
            } else {
                // Map -deadband to 0 and map -max to -max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                // x₁ = -deadband
                // y₁ = 0
                // x₂ = -max
                // y₂ = -max
                //
                // y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                // y = max/(max - deadband) (x + deadband)
                // y = max (x + deadband)/(max - deadband)
                return maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * 095   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * 096   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     * 097   *
     * 098   * @param value Value to clip.
     * 099   * @param deadband Range around zero.
     * 100   * @return The value after the deadband is applied.
     * 101
     */
    public static double applyDeadband(double value, double deadband) {
        return applyDeadband(value, deadband, 1);
    }

    /**
     * 107   * Returns modulus of input.
     * 108   *
     * 109   * @param input Input value to wrap.
     * 110   * @param minimumInput The minimum value expected from the input.
     * 111   * @param maximumInput The maximum value expected from the input.
     * 112   * @return The wrapped value.
     * 113
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Wraps an angle to the range -π to π radians.
     *
     * @param angleRadians Angle to wrap in radians.
     * @return The wrapped angle.
     */
    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -Math.PI, Math.PI);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * WPIMathUtil.clamp(t, 0, 1);
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue   Upper part of interpolation range.
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    public static double inverseInterpolate(double startValue, double endValue, double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q - startValue;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(double expected, double actual, double tolerance) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        return Math.abs(expected - actual) < tolerance;
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance. Supports
     * continuous input for cases like absolute encoders.
     *
     * <p>Continuous input means that the min and max value are considered to be the same point, and
     * tolerances can be checked across them. A common example would be for absolute encoders: calling
     * isNear(2, 359, 5, 0, 360) returns true because 359 is 1 away from 360 (which is treated as the
     * same as 0) and 2 is 2 away from 0, adding up to an error of 3 degrees, which is within the
     * given tolerance of 5.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @param min       Smallest value before wrapping around to the largest value
     * @param max       Largest value before wrapping around to the smallest value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(
            double expected, double actual, double tolerance, double min, double max) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        // Max error is exactly halfway between the min and max
        double errorBound = (max - min) / 2.0;
        double error = WPIMathUtil.inputModulus(expected - actual, -errorBound, errorBound);
        return Math.abs(error) < tolerance;
    }
}
























