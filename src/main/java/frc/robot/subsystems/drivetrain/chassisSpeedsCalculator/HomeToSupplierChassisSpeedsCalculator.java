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

    private final ProfiledPIDController angularController;
    private final Supplier<Rotation2d> angularSupplier;

    private final String name;

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

    public double angularDeadband(double angularOutput){
        return angularOutput < DrivetrainConstants.ANGULAR_DEADBAND ? 0 : angularOutput;
    }
}
