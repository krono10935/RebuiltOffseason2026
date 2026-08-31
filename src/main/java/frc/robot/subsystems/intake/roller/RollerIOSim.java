package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO{
    private final SparkMax motorOne;
    private final SparkMax motorTwo; 
    private final FlywheelSim rollerSim;

    public RollerIOSim(){
        rollerSim = RollerConstants.getSim();

        motorOne = new SparkMax(RollerConstants.MOTOR_ONE_CANID, MotorType.kBrushless);
        motorTwo = new SparkMax(RollerConstants.MOTOR_TWO_CANID, MotorType.kBrushless);

        motorOne.configure(RollerConstants.getLeadConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorTwo.configure(RollerConstants.getFollowerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the dutyCycle of the roller
     * @param dutyCycle The effort the motor applies
     */
    @Override
    public void setDutyCycle(double dutyCycle) {
        motorOne.getClosedLoopController().setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    /** 
     * Stops the roller
     */
    @Override
    public void stop() {
        motorOne.stopMotor();
    }

    /**
     * Steps the sim by Constants.LOOP_PERIOD_SECONDS (20ms)
     */
    private void stepSimulation(){
        SparkMaxSim simState = new SparkMaxSim(motorOne, RollerConstants.GEAR_BOX);
        simState.setBusVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = simState.getBusVoltage();

        rollerSim.setInputVoltage(motorVoltage);
        rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

        simState.setPosition(
            UnitConversions.RPMtoRotationsPerCycle(
                rollerSim.getAngularVelocityRPM(), 
                Constants.LOOP_PERIOD_SECONDS)
                * RollerConstants.GEAR_RATIO + simState.getAbsoluteEncoderSim().getPosition());
        
        simState.setVelocity(
            rollerSim.getAngularVelocityRPM() * RollerConstants.GEAR_RATIO
        );
    }

    /**
     * Updates the inputs object
     * @param inputs The inputs object we will be updating.
     */
    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.motorOneTempC = motorOne.getMotorTemperature();
        inputs.motorTwoTempC = motorTwo.getMotorTemperature();
        inputs.speedMPS = motorOne.getEncoder().getVelocity();
    }
}