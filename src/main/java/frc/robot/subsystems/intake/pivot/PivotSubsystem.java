package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.IsNear;

public class PivotSubsystem extends SubsystemBase {
    private final PivotInputsAutoLogged inputs;
    private final PivotIO io;

    public Rotation2d targetRotationTEMP = Rotation2d.kZero;
    //Create a new PivotSubsystem
    public PivotSubsystem(){
        inputs = new PivotInputsAutoLogged();
        io = RobotBase.isReal() ? new PivotIOCTRE() : new PivotIOSim(); 
    }

    @Override
    public void periodic(){
        updateInputs();
        Logger.processInputs(getName(), inputs);
        
        if (getCurrentCommand() == null){
            Logger.recordOutput("current command: ", "null");
        }
        
        else {
            Logger.recordOutput("current command: ", getCurrentCommand().getName());
        }
        


    }

    /**
     * Get the motor temprature
     * @return The temprature of the motor in celsius
     */
    public double getMotorTempC(){
        return inputs.motorTemperatureC;
    }

    /**
     * Get the pivot angle
     * @return The angle of the pivot
     */
    public Rotation2d getAngle(){
        return inputs.angle;
    }

    /**
     * Get the angular velocity of the roller
     * @return The angular velocity of the pivot in RPS
     */
    public double getAngularVelocityRPS(){
        return inputs.angularVelocityRPS;
    }

    /**
     * Command the hardware to go to a rotation
     * @param rotation the wanted rotation
     */

    public void setRotation(Rotation2d rotation){
        targetRotationTEMP = rotation;
        io.setRotation(rotation);
    }

    /**
     * Command the hardware to go to a rotation, using a trapezoid profile to slow the pivot. The goal is for the balls to not get stuck between the pivot and the shooter
     * @param rotation The wanted rotation
s    */
    public void setRotationSlow(Rotation2d rotation){
        targetRotationTEMP = rotation;
        io.setRotationSlow(rotation);
    }

    /**
     * Stops the pivot
     */
    public void stop(){
        io.stop();
    }

    /**
     * Updates the field values of the input object.
     */
    private void updateInputs(){
        io.updateInputs(inputs);

        if(targetRotationTEMP != null){
            System.out.println("target " + targetRotationTEMP.getDegrees());
            Logger.recordOutput("target rotation degrees", targetRotationTEMP.getDegrees());

            Logger.recordOutput("target rotation degrees", targetRotationTEMP.getDegrees() + 5);
        }
        else{
            System.out.println("target -10");
            Logger.recordOutput("target rotation degrees", -10);
        }
    }

    /**
     * Checks if the pivot is open
     * @return whether the pivot is open
     */
    public boolean isPivotOpen() {
        return IsNear.isNear(getAngle(), PivotConstants.PIVOT_OPEN_ANGLE, PivotConstants.TOLERANCE);
    }
    /**
     * Checks if the pivot is close
     * @return whether the pivot is close
     */
    public boolean isPivotClose() {
        return IsNear.isNear(getAngle(), PivotConstants.PIVOT_CLOSE_ANGLE, PivotConstants.TOLERANCE);
    }

    /**
     * A command that opens the pivot
     * @return A command that opens the pivot
     */
    public Command openPivot(){
        return Commands.run(
            () -> setRotation(PivotConstants.PIVOT_OPEN_ANGLE),
            this
        );
    }

    /**
     * A command that opens the pivot slowly, using a trapezoid profile.
     * @return A command that opens the pivot slowly, using a trapezoid profile.
     */
    public Command openPivotSlow(){
        return Commands.run(
            () -> setRotationSlow(PivotConstants.PIVOT_OPEN_ANGLE),
            this
        );
    }

    /**
     * A command that closes the pivot
     * @return A command that closes the pivot
     */
    public Command closePivot(){
        return Commands.run(
            () -> setRotation(PivotConstants.PIVOT_CLOSE_ANGLE),
            this
        );
    }

    /**
     * A command that opens the pivot slowly, using a trapezoid profile.
     * @return A command that opens the pivot slowly, using a trapezoid profile.
     */
    public Command closePivotSlow(){
        return Commands.run(
            () -> setRotationSlow(PivotConstants.PIVOT_CLOSE_ANGLE),
            this
        );
    }
}