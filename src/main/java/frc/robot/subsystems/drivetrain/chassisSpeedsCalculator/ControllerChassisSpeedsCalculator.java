package frc.robot.subsystems.drivetrain.chassisSpeedsCalculator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants.ChassisSpeedConfig;

public class ControllerChassisSpeedsCalculator {
        public enum ControllerMode {
        
        /** No exponent used */
        NONE(1),

        /** Exponent only on the brake. */
        BRAKE_EXPONENTIAL(0.5),

        /** Exponent only on the drive sticks */
        STICKS_EXPONENTIAL(0.5),

        /** Exponent on both the sticks and the brakes. */
        BOTH_EXPONENTIAL(0.5);

        /** The exponent to apply */
        public final double exponent;

        ControllerMode(double exponent) {
            this.exponent = exponent;
        }

        /**
         * Calculate based on exponent for the trigger 
         * @param value the raw trigger value
         * @return the exponentiated trigger value
         */
        public double calculateTrigger(double value){
            if(this == STICKS_EXPONENTIAL) return value;

            Logger.recordOutput("DriveCommand/trigger value input", value);

            return calculateExponential(value, exponent);
        }

        /**
         * Calculate based on exponent for the trigger 
         * @param value the raw stick value
         * @return the exponentiated stick value
         */
        public double calculateStick(double value){
            if(this == BRAKE_EXPONENTIAL) return value;

            return  calculateExponential(value, exponent);
        }

        /**
         * Exponentiate an input from the controller
         * @param value the raw value
         * @param exponential the exponent to apply to it
         * @return the exponentiated value
         */
        private static double calculateExponential(double value, double exponential) {
            return Math.copySign(Math.pow(Math.abs(value), exponential), value);
        }
    }

    private final ChassisSpeedConfig speedConfig;
    private final double MIN_ANGULAR_SPEED;
    private final double MAX_ANGULAR_SPEED;
    private static final ControllerMode CONTROLLER_MODE = ControllerMode.BRAKE_EXPONENTIAL;

    /**
     * Deadband threshold for controller inputs
     */
    protected static final double DEADBAND = 0.1;

    public ControllerChassisSpeedsCalculator(ChassisSpeedConfig speedConfig, double moduleDistanceFromCenterMeters){
        this.speedConfig = speedConfig;
        this.MIN_ANGULAR_SPEED = speedConfig.minLinearSpeed() / moduleDistanceFromCenterMeters;
        this.MAX_ANGULAR_SPEED = speedConfig.maxLinearSpeed() / moduleDistanceFromCenterMeters;
    }

        /**
     * Linear interpolation for speed scaling.
     *
     * @param value normalized input (0–1)
     * @return interpolated linear speed
     */
    private double interpolate(double value) {
        return speedConfig.minLinearSpeed() + (speedConfig.maxLinearSpeed() - speedConfig.minLinearSpeed()) * value;
    }

    /**
     * Angular interpolation for rotation scaling.
     *
     * @param value normalized input (0–1)
     * @return interpolated angular speed
     */
    private double angularInterpolate(double value) {
        return MIN_ANGULAR_SPEED + (MAX_ANGULAR_SPEED - MIN_ANGULAR_SPEED) * value;
    }

    /**
     * Applies deadband to joystick input.
     */
    protected static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }


    /**
     * Converts controller input into chassis speeds.
     */
    public ChassisSpeeds getControllerInputs() {
        CommandXboxController controller = RobotContainer.getInstance().getController();

        double triggerValue = CONTROLLER_MODE.calculateTrigger(1 - controller.getRightTriggerAxis());

        double speed = interpolate(triggerValue);
        double angularSpeed = angularInterpolate(triggerValue);

        Logger.recordOutput("DriveCommand/trigger value calculated", triggerValue);

        double xSpeed = CONTROLLER_MODE.calculateStick(deadband(-controller.getLeftY())) * speed;
        double ySpeed = CONTROLLER_MODE.calculateStick(deadband(-controller.getLeftX())) * speed;
        double thetaSpeed = CONTROLLER_MODE.calculateStick(deadband(-controller.getRightX())) * angularSpeed;

        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }

}
