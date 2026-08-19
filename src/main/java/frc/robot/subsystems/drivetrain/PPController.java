//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class PPController implements PathFollowingController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private static Optional<DoubleSupplier> thetaOutputOverride = Optional.empty();

    public PPController(PIDConstants translationConstants, PIDConstants rotationConstants, double period) {

        this.xController = new PIDController(
            translationConstants.kP, translationConstants.kI,
            translationConstants.kD, period
        );

        this.xController.setIZone(0.15);



        this.yController = new PIDController(
            translationConstants.kP, translationConstants.kI, 
            translationConstants.kD, period
        );

        this.yController.setIZone(translationConstants.iZone);



        this.rotationController = new PIDController(
            rotationConstants.kP, rotationConstants.kI, 
            rotationConstants.kD, period
        );

        this.rotationController.setIZone(rotationConstants.iZone);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.xController.setIntegratorRange(-1.5, 1.5);

        Logger.recordOutput("thetaOutputOverride", false);
    }

    public PPController(PIDConstants translationConstants, PIDConstants rotationConstants) {
        this(translationConstants, rotationConstants, 0.02);
    }


    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        this.xController.reset();
        this.yController.reset();
        this.rotationController.reset();
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        double xFF = targetState.fieldSpeeds.vxMetersPerSecond;
        double yFF = targetState.fieldSpeeds.vyMetersPerSecond;

        double xFeedback = xController.calculate(currentPose.getX(), targetState.pose.getX());
        double yFeedback = yController.calculate(currentPose.getY(), targetState.pose.getY());

        var targetPose = targetState.pose;
        double rotationFF = targetState.fieldSpeeds.omegaRadiansPerSecond;

        double rotationFeedback = rotationController.calculate(currentPose.getRotation().getRadians(),
             targetPose.getRotation().getRadians());

        double rotationOutput = rotationFeedback + rotationFF;

        Logger.recordOutput("PP/xOutput", xFeedback);
        Logger.recordOutput("PP/yOutput", yFeedback);
        Logger.recordOutput("PP/rotationOutput", rotationFeedback);

        Logger.recordOutput("PP/xError", xController.getError());
        Logger.recordOutput("PP/yError", yController.getError());
        Logger.recordOutput("PP/rotationError", rotationController.getError());

        Logger.recordOutput("PP/xSetpoint", targetPose.getX());
        Logger.recordOutput("PP/ySetpoint", targetPose.getY());
        Logger.recordOutput("PP/rotationSetpoint", targetPose.getRotation().getRadians());

        Logger.recordOutput("PP/xFF", xFF);
        Logger.recordOutput("PP/yFF", yFF);
        Logger.recordOutput("PP/rotationFF", rotationFF);

        if(thetaOutputOverride.isPresent()){
            rotationOutput = thetaOutputOverride.get().getAsDouble();
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, rotationOutput, currentPose.getRotation());
    }

    @Override
    public boolean isHolonomic() {
        return true;
    }

    /**
     * override the rotationController in order to use a custom angularController 
     * @param output the supplier that overrides the angle
     */
    public static void setThetaOverride(DoubleSupplier output){
        thetaOutputOverride = Optional.of(output);
        Logger.recordOutput("thetaOutputOverride", true);
    }

    /**
     * Clear the override from the angular controller (use the rotationController again)
     */
    public static void clearThetaOverride(){
        thetaOutputOverride = Optional.empty();
        Logger.recordOutput("thetaOutputOverride", false);
    }
}
