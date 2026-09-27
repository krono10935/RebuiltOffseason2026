package frc.robot.subsystems.shooter;

import java.util.Random;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShootCalculatorWithMovement.ShootCalculatorWithMovementParams;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.lib.field.AllianceFlipUtil;

public class ShotCalculator {
    private static ShotCalculator instance;

    private static Rotation2d robotAngleOffset = Rotation2d.kZero;
    private static Rotation2d hoodOffset = Rotation2d.kZero;
    private static double flyWheelOffset = 0;

    public void addRobotAngleOffset(Rotation2d offset){
        robotAngleOffset = robotAngleOffset.plus(offset);
    }

    public void addHoodAngleOffset(Rotation2d offset){

        hoodOffset = hoodOffset.plus(offset);
    }

    public void addflyWheelOffset(double offset){
        flyWheelOffset += offset;
    }

    public void resetOffsets(){
        robotAngleOffset = Rotation2d.kZero;
        hoodOffset = Rotation2d.kZero;
        flyWheelOffset = 0;
    }

    public enum ValidityState{
        VALID("Valid", AlertType.kInfo),
        OUT_OF_RANGE("Out of shooting range", AlertType.kWarning),
        TOO_MUCH_OMEGA_SPEED("Stop spinning", AlertType.kWarning),
        HUB_INACTIVE("Womp womp the hub is inactive", AlertType.kWarning),
        SHOULD_NOT_BE_MOVING("Stop moving", AlertType.kWarning);

        private final Alert alert;

        ValidityState(String message, AlertType alertType){
            alert = new Alert(message, alertType);
        }

        public void toggleAlert(boolean activate){
            alert.set(activate);
        }
    }


    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
        ValidityState validityState,
        Rotation2d robotAngle,
        Rotation2d hoodAngle,
        double flywheelSpeed,
        Rotation2d robotAngleOffset,
        Rotation2d hoodAngleOffset,
        double flyWheelOffset) {
            public ShootingParameters(ValidityState validityState,
                Rotation2d robotAngle,
                Rotation2d hoodAngle,
                double flywheelSpeed){
                
                this(
                    validityState,
                    robotAngle.plus(ShotCalculator.robotAngleOffset),
                    hoodAngle.plus(ShotCalculator.hoodOffset),
                    flywheelSpeed + ShotCalculator.flyWheelOffset,
                    ShotCalculator.robotAngleOffset,
                    ShotCalculator.hoodOffset,
                    ShotCalculator.flyWheelOffset
                );
        }
        }
        
    private ShootingParameters latestParameters = null;

    private static final double minDistance;
    private static final double maxDistance;
    private static final double phaseDelay;

    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = 
        new InterpolatingDoubleTreeMap();
    
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();

    /**
     *
     * @param distance
     * @param angleDegrees
     * @param shotSpeed
     * @param timeOfFlight
     */
    private static void putToMaps(double distance, double angleDegrees, double shotSpeed, double timeOfFlight){
        distance = distance - FlywheelConstants.ROBOT_TO_FLYWHEEL.getX();
        shotHoodAngleMap.put(distance, Rotation2d.fromDegrees(angleDegrees));
        shotFlywheelSpeedMap.put(distance,shotSpeed);
        timeOfFlightMap.put(distance,timeOfFlight);
    }
    static {
        minDistance = 1; // TODO: find the real value
        maxDistance = 5.60; // TODO: find the real value
        phaseDelay = 0.015; // TODO: find the real value

        putToMaps(1.065, 3, 15.0, 0.9);
        putToMaps(1.196, 4, 15.5, 1.0 );
        putToMaps(1.307, 5,15.65, 1.12 );
        putToMaps(1.401, 5,15.5, 1.06 );
        putToMaps(1.510, 6,15.5, 1.09 );
        putToMaps(1.610, 7, 15.5 ,1 );
        putToMaps(1.707, 8, 15.5, 1.03 );
        putToMaps(1.813,9, 15.5, 1);
        putToMaps(1.910, 10 , 16.2 , 1.08);
        putToMaps(2.011, 11, 16.3 , 1.06);
        putToMaps(2.101, 12, 16.4,0.94);
        putToMaps(2.194, 13, 16.4,0.95 );
        putToMaps(2.310 , 14, 16.4, 0.92);
        putToMaps(2.4, 15, 16.4, 0.95);
        putToMaps(2.5, 16, 16.4, 0.94);
        putToMaps(2.6, 17, 17.1,1 );
        putToMaps(2.7, 18, 17.2, 0.9);
        putToMaps(2.8, 19, 17.5, 1.02 );
        putToMaps(2.91, 18, 18, 0.95);
        putToMaps(3,19, 18, 1.02 );
        putToMaps(3.1, 21, 18, 1.01);
        putToMaps(3.2, 22, 18.1, 1.07);
        putToMaps(3.3, 23, 18.1, 1);
        putToMaps(3.4, 25, 18.1, 0.9);
        putToMaps(3.5, 26, 18.1,0.97 );
        putToMaps(3.6, 27,18.1, 1.04);
        putToMaps(3.7 , 28, 18.1, 1.07);
        putToMaps(4.0 , 28, 18.2, 1.2);
        putToMaps(4.22 , 28, 18.55, 1.2);
        putToMaps(4.4 , 29, 23, 1.27);
        putToMaps(4.6 , 29, 25, 1.45);
        putToMaps(4.8 , 29, 27, 1.5);

    }

    /**
     * burst a ton of calculations on the ShotCalculator's getParameters function for 10 seconds in order for subsequent runs to be faster
     * @return the command for warming up the shot calculator
     */
    public Command warmUpShotCalculator(){
        Random rnd = new Random();
        return new RunCommand(() -> getParameters(new Pose2d(
            new Translation2d(rnd.nextDouble(0, 16), rnd.nextDouble(0, 8))
            , Rotation2d.fromDegrees(rnd.nextDouble(0, 360))
            ),

            new ChassisSpeeds(rnd.nextDouble(-6, 6), rnd.nextDouble(-4, 4),
                rnd.nextDouble(-10*2*Math.PI, 10*2*Math.PI))))
        .withTimeout(10).ignoringDisable(true);
    }

    /**
     * wrapper for ShotCalculator::getParameters when you don't are standing still
     * @param estimatedPose the robot's estimated position
     * @return ShootingParameters based on the estimated position and the robot not moving (speed = 0)
     */
    public ShootingParameters getStaticParameters(Pose2d estimatedPose){
        return getParameters(estimatedPose, new ChassisSpeeds());
    }

    /**
     * gets shooting parameters based on the robot's estimated position and velocity
     * @param estimatedPose Estimated robot pose
     * @param robotRelativeVelocity Robot-relative velocity
     * @return ShootingParameters based on these parameters
     */
    public ShootingParameters getParameters(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity){
        if (latestParameters != null){
            return latestParameters;
        }
        
        Pose2d shooterPosition = shooterPoseAtShooting(estimatedPose, robotRelativeVelocity);

        Logger.recordOutput("ShotCalculator/Shooter position", shooterPosition);
        
        Translation2d hub = 
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
     
        Translation2d shooterFieldRelativeSpeeds = 
            getShooterFieldRelativeSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation()),
            shooterPosition);
        if (ShooterConstants.SHOOT_WITH_MOVEMENT){

            return calculateShootingParametersWithMovement(shooterPosition, shooterFieldRelativeSpeeds, hub, robotRelativeVelocity);

        } else {

            return calculateShootingParametersWithoutMovement(shooterPosition, shooterFieldRelativeSpeeds, hub, robotRelativeVelocity);
        }

    }

    /**
     * @param robotRelativeVelocity The robot relative velocity
     * @param shooterFieldRelativeSpeeds field relative X and Y speed of the robot (and therefore the shooter)
     * @param lookaheadShooterToTargetDistance The shoot with movement params
     * @return the validity state of the calculation
     */
    @SuppressWarnings("unused")
    private ValidityState findValidityState(ChassisSpeeds robotRelativeVelocity, Translation2d shooterFieldRelativeSpeeds,
     double lookaheadShooterToTargetDistance){

        ValidityState state = ValidityState.VALID;

        // TODO: figure out how to implement this now (not different from the season just Dan will enjoy like I did)
        // if (!Constants.HubTiming.isActiveWithMargin(
        //     DriverStation.getMatchTime() - timeOfFlightMap.get(lookaheadShooterToTargetDistance),
        //     0,
        //     1
        // )){
        //     state = ValidityState.HUB_INACTIVE;
        // }

        if (Math.abs(robotRelativeVelocity.omegaRadiansPerSecond) > ShooterConstants.ANGULAR_DEADBAND_DEGREES_PER_SECOND){
                state = ValidityState.TOO_MUCH_OMEGA_SPEED;
        }

        if (!ShooterConstants.SHOOT_WITH_MOVEMENT && Math.abs(shooterFieldRelativeSpeeds.getNorm()) > ShooterConstants.LINEAR_SPEED_DEADBAND_MPS){
            state = ValidityState.SHOULD_NOT_BE_MOVING;
        }

        if (lookaheadShooterToTargetDistance <= minDistance ||
            lookaheadShooterToTargetDistance >= maxDistance){
                state = ValidityState.OUT_OF_RANGE;
        }

        return state;
    }

    /**
     * 
     * @param estimatedPose robot estimated pose
     * @param robotRelativeVelocity robot relative velocity of the robot
     * @return The pose the robot will actually have at shooting
     */
    private Pose2d shooterPoseAtShooting(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity){
        estimatedPose =  estimatedPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay
            ));

        Transform2d transformRobotToShooter = 
        new Transform2d(
            FlywheelConstants.ROBOT_TO_FLYWHEEL.getTranslation().toTranslation2d(),
            Rotation2d.fromRadians(
                FlywheelConstants.ROBOT_TO_FLYWHEEL.getRotation().getZ()
            )
        );

        // Calculate distance from the shooter to the target
        Pose2d shooterPosition = estimatedPose.transformBy(transformRobotToShooter); 

        return shooterPosition;
    }

    /**
     * 
     * @param robotVelocityFieldRelative the robot velocity relative to the field
     * @param estimatedShooterPose the current estimated robot pose
     * @return Translation2d object encompassing the field relative X and Y speeds
     */
    private Translation2d getShooterFieldRelativeSpeeds(ChassisSpeeds robotVelocityFieldRelative, Pose2d estimatedShooterPose){
        
        Translation2d linearSpeed = new Translation2d(
            robotVelocityFieldRelative.vxMetersPerSecond,
            robotVelocityFieldRelative.vyMetersPerSecond
        );

        double tangentRobotAngularSpeed = robotVelocityFieldRelative.omegaRadiansPerSecond * 
            FlywheelConstants.ROBOT_TO_FLYWHEEL.getTranslation().getNorm();
        
        Rotation2d tangentRobotAngleFieldRelative = estimatedShooterPose.getRotation().rotateBy(Rotation2d.kCCW_90deg);
        
        Translation2d angularToLinearFieldRelativeSpeed = new Translation2d(
            tangentRobotAngularSpeed, tangentRobotAngleFieldRelative
        );

        Logger.recordOutput("ShotCalculator/AngularToFieldRelative", angularToLinearFieldRelativeSpeed);
        Logger.recordOutput("ShotCalculator/LinearSpeeds", linearSpeed);

        return linearSpeed.plus(angularToLinearFieldRelativeSpeed);

    }


    /**
     * 
     * @param shooterPosition Position of the shooter
     * @param shooterFieldRelativeSpeeds Speed of the shooter on the field
     * @param hub Translation 2d of where the hub is located
     * @param robotRelativeVelocity Robot relative velocity of the robot, which is also shooter's velocity
     * @return Shooting params calculated with movement
     */
    private ShootingParameters calculateShootingParametersWithMovement(Pose2d shooterPosition, Translation2d shooterFieldRelativeSpeeds,
     Translation2d hub, ChassisSpeeds robotRelativeVelocity){
        
        ShootCalculatorWithMovementParams shootWithMovementParams = 
                ShootCalculatorWithMovement.regressFuturePositionParams(shooterPosition, timeOfFlightMap,
                shooterFieldRelativeSpeeds, hub);
        
        Logger.recordOutput("ShotCalculator/shootWithMovementParams", shootWithMovementParams);
            
        Pose2d lookaheadPose = shootWithMovementParams.lookaheadPose();
        double lookaheadShooterToTargetDistance = shootWithMovementParams.lookaheadShooterToTargetDistance();

            // Find the robot angle to shoot at
        Rotation2d robotAngle = hub.minus(lookaheadPose.getTranslation()).getAngle();

        // Calculate the optimal hood angle
        var hoodAngle = shotHoodAngleMap.get(lookaheadShooterToTargetDistance);

        ValidityState state = findValidityState(robotRelativeVelocity, shooterFieldRelativeSpeeds, lookaheadShooterToTargetDistance);

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(state,
            robotAngle,
            hoodAngle,
            shotFlywheelSpeedMap.get(lookaheadShooterToTargetDistance));

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);


        Logger.recordOutput("ShotCalculator/output", latestParameters);


        return latestParameters;

    }

    /**
     * 
     * @param shooterPosition Position of the shooter
     * @param shooterFieldRelativeSpeeds Speed of the shooter on the field
     * @param hub Translation 2d of where the hub is located
     * @param robotRelativeVelocity Robot relative velocity of the robot, which is also shooter's velocity
     * @return Shooting params calculated without movement
     */
    public ShootingParameters calculateShootingParametersWithoutMovement(Pose2d shooterPosition, Translation2d shooterFieldRelativeSpeeds,
     Translation2d hub, ChassisSpeeds robotRelativeVelocity){
        
        double distanceFromHub = shooterPosition.getTranslation().getDistance(hub);

        Rotation2d robotAngle = hub.minus(shooterPosition.getTranslation()).getAngle();

        // Calculate the optimal hood angle
        var hoodAngle = shotHoodAngleMap.get(distanceFromHub);

        ValidityState state = findValidityState(robotRelativeVelocity, shooterFieldRelativeSpeeds, distanceFromHub);

        // Build new shooting params record
        latestParameters = 
            new ShootingParameters(state,
            robotAngle,
            hoodAngle,
            shotFlywheelSpeedMap.get(distanceFromHub));


        Logger.recordOutput("ShotCalculator/output", latestParameters);

        return latestParameters;
    }

    /**
     * Clear shooting parameters to allow recalculation
     */
    public void clearShootingParameters(){
        latestParameters = null;
    }



}
