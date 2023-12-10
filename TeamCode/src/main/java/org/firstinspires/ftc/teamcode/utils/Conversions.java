package org.firstinspires.ftc.teamcode.utils;

public class Conversions {
    public static int inchesToTicks(double inches, double WHEEL_RADIUS, double GEAR_RATIO, double TICKS_PER_REV) {
        return (int)Math.round((TICKS_PER_REV * inches) / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO));
    }

    public static double ticksToInches(double ticks, double WHEEL_RADIUS, double GEAR_RATIO, double TICKS_PER_REV) {
        return (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO * ticks) / TICKS_PER_REV;
    }
}