package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;

public abstract class SwerveModuleIO {

    public final CTREModuleConstants constants;

    private final SwerveModuleState currentState = new SwerveModuleState();

    private final SwerveModulePosition position = new SwerveModulePosition();

    protected SwerveModuleIO(CTREModuleConstants constants){
        this.constants = constants;
    }

    /**
     * @return The drive motor speed in meters per second
     */
    protected abstract double getDriveVelocity();

    /**
     * @return The drive motor position in meters
     */
    protected abstract double getDriveDistance();

    /**
     * @return The steer motor position in rotations
     */
    protected abstract double getSteerAngle();

    /**
     * Sets the target state of the module and gives the command to the module motors
     * @param targetState The state that the module should go to
     */
    public void setTargetState(SwerveModuleState targetState){
        setTargetState(targetState, 0);
    }

    /**
     * Sets the target state of the module and gives the command to the module motors
     * @param targetState The state that the module should go to
     * @param slot The closed loop slot to use to go to that state
     */
    public abstract void setTargetState(SwerveModuleState targetState, int slot);

    /**
     * Set if the module is Brake or Coast
     * @param isBrake whether the module motor should resist outside change in disable
     */
    public abstract void setBrakeMode(boolean isBrake);

    /**
     * function used by sysID to profile the behavior of the module
     * @param voltage the voltage that the drive motor should apply
     * @param angle the angle of the steer motor
     */
    public abstract void setDriveVoltageAndSteerAngle(double voltage, Rotation2d angle);


    /**
     * function used by sysID to profile the behavior of the steer motor
     * @param voltage the voltage that the steer motor should apply
     */
    public abstract void setSteerVoltage(double voltage);


    /**
     *
     * @return the current state of the module
     */
    public SwerveModuleState getState(){
        return currentState;
    }

    /**
     *
     * @return the current position of the module
     */
    public SwerveModulePosition getPosition(){
        return position;
    }


    /**
     * Update the module's attributes
     */
    public void update(){
        currentState.angle = Rotation2d.fromRotations(getSteerAngle());
        position.angle = currentState.angle;
        currentState.speedMetersPerSecond = getDriveVelocity();
        position.distanceMeters = getDriveDistance();
    }
}
