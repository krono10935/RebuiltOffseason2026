package frc.robot.subsystems.intake.roller;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO{
    private final SparkMax motorOne;
    private final SparkMax motorTwo;

    private final FlywheelSim rollerSim;
    private final SparkMaxSim simState;
    
    public RollerIOSim(){
        rollerSim = RollerConstants.getSim();

        motorOne = new SparkMax(RollerConstants.MOTOR_ONE_CANID, MotorType.kBrushless);
        motorTwo = new SparkMax(RollerConstants.MOTOR_TWO_CANID, MotorType.kBrushless);

        motorOne.configure(RollerConstants.getLeadConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorTwo.configure(RollerConstants.getFollowerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        simState = new SparkMaxSim(motorOne, RollerConstants.GEAR_BOX);

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
    private void simulationStep(){
        rollerSim.setInput(simState.getAppliedOutput() * RoboRioSim.getVInVoltage());

        rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

        simState.iterate(
            rollerSim.getAngularVelocityRPM() * RollerConstants.GEAR_RATIO,
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_PERIOD_SECONDS);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps()));
    }

    /**
     * Updates the inputs object
     * @param inputs The inputs object we will be updating.
     */
    @Override
    public void updateInputs(RollerInputs inputs) {
        simulationStep();
        
        Logger.recordOutput("rollerSim/Apllied outpus sim motor", simState.getAppliedOutput());
        Logger.recordOutput("rollerSim/Applied output", motorOne.getAppliedOutput());

        inputs.motorOneTemperatureC = motorOne.getMotorTemperature();
        inputs.motorTwoTemperatureC = motorTwo.getMotorTemperature();
        inputs.speedMPS = motorOne.getEncoder().getVelocity();

        //Logger.recordOutput("rollerSim/", null);
        Logger.recordOutput("rollerSim/full applied voltage", simState.getAppliedOutput() * RoboRioSim.getVInVoltage());
        Logger.recordOutput("rollerSim/motor MPS", motorOne.getEncoder().getVelocity());
        Logger.recordOutput("rollerSim/roller RPM", rollerSim.getAngularVelocityRPM());
        Logger.recordOutput("rollerSim/motor position", motorOne.getEncoder().getPosition());

        Logger.recordOutput("rollerSim/VIN voltage",  RoboRioSim.getVInVoltage());
    }
}