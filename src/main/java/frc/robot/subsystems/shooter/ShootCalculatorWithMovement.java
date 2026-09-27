package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootCalculatorWithMovement  {

    public record ShootCalculatorWithMovementParams(
        double timeOfFlight,
        Pose2d lookaheadPose,
        double lookaheadShooterToTargetDistance) {
    }

    /**
     * 
     * @param shooterPosition The position of the shooter on the field
     * @param timeOfFlightMap Time of flight map distance from hub : estimated time of flight
     * @param shooterFieldRelativeSpeeds Shooter speeds field relative
     * @param hub Translation2d of the hub position
     * @return The regressed shot calculator with movement params
     */
    public static ShootCalculatorWithMovementParams regressFuturePositionParams(Pose2d shooterPosition,
     InterpolatingDoubleTreeMap timeOfFlightMap, Translation2d shooterFieldRelativeSpeeds, Translation2d hub){
        // Regress to the optimal shooting calculation for distance to shoot
        // We assume that this is to calculate the v0(m/s) that the robot gives the ball in fieldRelative terms
        double timeOfFlight = 0;
        Pose2d lookaheadPose = shooterPosition;
        double lookaheadShooterToTargetDistance = shooterPosition.getTranslation().getDistance(hub);
        for (int i = 0; i < 20; i++){
            timeOfFlight = timeOfFlightMap.get(lookaheadShooterToTargetDistance);
            double offsetX = shooterFieldRelativeSpeeds.getX() * timeOfFlight;
            double offsetY = shooterFieldRelativeSpeeds.getY() * timeOfFlight;

            lookaheadPose = 
                 new Pose2d(
                    shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    shooterPosition.getRotation());

            lookaheadShooterToTargetDistance = hub.getDistance(lookaheadPose.getTranslation());
        }

        return new ShootCalculatorWithMovementParams(timeOfFlight, lookaheadPose, lookaheadShooterToTargetDistance);

    }
}
