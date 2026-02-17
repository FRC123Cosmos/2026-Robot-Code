package frc.robot.utils;

public class MathUtils {
    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        // The Arduino map function uses integer arithmetic, which truncates results
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
