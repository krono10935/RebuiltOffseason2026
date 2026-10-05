package frc.robot.subsystems.drivetrain.module;


import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;


public class SwerveModuleSim extends SwerveModuleCTRE {

    protected final DCMotorSim driveMotorSim;
    protected final DCMotorSim steerMotorSim;


    public SwerveModuleSim(CTREModuleConstants constants){
        super(constants);

        LinearSystem<N2,N1,N2> driveMotorPlant = 
            constants.DRIVE_SIM_CONFIG().motorGains().isPresent() ? 
            constants.DRIVE_SIM_CONFIG().getCharacterizedBasedSimulation() : 
            constants.DRIVE_SIM_CONFIG().getMomentInertiaBasedSimulation();


        driveMotorSim = new DCMotorSim(
            driveMotorPlant, 
            constants.DRIVE_SIM_CONFIG().motor()
        );

        drivingMotor.getSimState().setMotorType(MotorType.KrakenX60);

        steeringMotor.getSimState().setMotorType(MotorType.KrakenX44);


        LinearSystem<N2,N1,N2> steerMotorPlant = 
            constants.STEER_SIM_CONFIG().motorGains().isPresent() ? 
            constants.STEER_SIM_CONFIG().getCharacterizedBasedSimulation() : 
            constants.STEER_SIM_CONFIG().getMomentInertiaBasedSimulation();

        steerMotorSim = new DCMotorSim(
            steerMotorPlant, 
            constants.STEER_SIM_CONFIG().motor()
        );
    }

    @Override
    public void update(){
        stepSimSteerMotor();
        stepSimDriveMotor();
        
        super.update();
    }

    /**
     * Step the Drive motor's simulation
     */
    private void stepSimDriveMotor(){
        TalonFXSimState simState = drivingMotor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = simState.getMotorVoltage();


        driveMotorSim.setInputVoltage(motorVoltage);
        driveMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

        simState.addRotorPosition(
            UnitConversions.rotationsPerMinutetoRotationsPerCycle(
                driveMotorSim.getAngularVelocityRPM(), 
                Constants.LOOP_PERIOD_SECONDS)
        );
        
        simState.setRotorVelocity(
            driveMotorSim.getAngularVelocity()
        );
    }

    /**
     * Step the steer motor's simulation
     */
    private void stepSimSteerMotor(){
        TalonFXSimState simState = steeringMotor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = simState.getMotorVoltage();

        steerMotorSim.setInputVoltage(motorVoltage);
        steerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

        simState.addRotorPosition(
            UnitConversions.rotationsPerMinutetoRotationsPerCycle(
                steerMotorSim.getAngularVelocityRPM(), 
                Constants.LOOP_PERIOD_SECONDS)
        );
        
        simState.setRotorVelocity(
            steerMotorSim.getAngularVelocity()
        );
    }
}
