package frc.robot.subsystems.drivetrain.chassisSpeedsCalculator;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.GeneralRobotState;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants.ChassisSpeedConfig;
import frc.robot.subsystems.drivetrain.constants.DrivetrainConstants;

public class HomeToSupplierChassisSpeedsCalculator extends ControllerChassisSpeedsCalculator {

    /** Profiled PID for the angle */
    private final ProfiledPIDController angularController;

    /** Supplier for a dynamically changing angle to aim for */
    private final Supplier<Rotation2d> angularSupplier;

    /** The name of thid instance of the HomeToSupplierChassisCalculator */
    private final String name;

    /**
     * Create a new HomeToSupplierChassisSpeedsCalculator instance
     * @param speedsConfig the drivetrain's speeds config
     * @param moduleDistanceFromCenterMeters the radius of the center of the module from the center of the robot
     * @param angularSupplier the supplier for the dynamically changing angle setpoint
     * @param name the name of the current instance of the calculator
     */
    public HomeToSupplierChassisSpeedsCalculator(ChassisSpeedConfig speedsConfig, 
            double moduleDistanceFromCenterMeters, 
            Supplier<Rotation2d> angularSupplier,
            String name){
        super(
            new ChassisSpeedConfig(speedsConfig.minLinearSpeed(), speedsConfig.maxLinearSpeed() / 4),
             moduleDistanceFromCenterMeters
        );

        this.angularSupplier = angularSupplier;
        this.angularController = DrivetrainConstants.THETA_CONTROLLER;
        this.name = name;
    }

    /**
     * Resets the theta controller using current robot state.
     */
    public void resetThetaController(){
        angularController.reset(
            new TrapezoidProfile.State(
                GeneralRobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                GeneralRobotState.getInstance().getChassisSpeedsSupplier().get().omegaRadiansPerSecond
            )
        );
    }

    /**
     * Calculates PID output toward target angle.
     *
     * @return angular velocity output
     */
    @Override
    public ChassisSpeeds getControllerInputs() {
        final double thetaMeasurement = GeneralRobotState.getInstance().getEstimatedPose().getRotation().getRadians();

        double thetaSpeed = angularController.calculate(
                thetaMeasurement,
                angularSupplier.get().getRadians()
        );

        thetaSpeed = angularDeadband(thetaSpeed);

        ChassisSpeeds controllerSpeeds = super.getControllerInputs();

        if (Math.abs(controllerSpeeds.omegaRadiansPerSecond) < DEADBAND){
            controllerSpeeds.omegaRadiansPerSecond = thetaSpeed;
        }

        Logger.recordOutput(name + "/thetaError", angularController.getPositionError());
        Logger.recordOutput(name + "/thetaSetpoint", angularController.getGoal());
        Logger.recordOutput(name + "/thetaMeasurement",
                thetaMeasurement);

        return controllerSpeeds;
    }

    /**
     * Apply a deadband to an angular output
     * @param angularOutput the raw angular output
     * @return 0 if the angular output is less than the deadband, otherwise the angular output itself
     */
    public double angularDeadband(double angularOutput){
        return angularOutput < DrivetrainConstants.ANGULAR_DEADBAND ? 0 : angularOutput;
    }
}
