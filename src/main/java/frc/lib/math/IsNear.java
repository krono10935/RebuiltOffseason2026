package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class IsNear {
    /**
     * Check if the distance between two numbers is within tolerance.
     * @param x The first number.
     * @param y The second number.
     * @param tolerance The tolerance.
     * @return Whether the distance between the numbers is in tolerance.
     */
    public static boolean isNear(double x, double y, double tolerance){
        return Math.abs(x - y) <= tolerance;
    }

    /**
     * Check if the distance between two angles is within tolerance.
     * @param x The first angle.
     * @param y The second angle.
     * @param tolerance The tolerance.
     * @return Whether the distance between the angles is within tolerance.
     */
    public static boolean isNear(Rotation2d x, Rotation2d y, Rotation2d tolerance){
        return isNear(x.getDegrees(), y.getDegrees(), tolerance.getDegrees());
    }
}
