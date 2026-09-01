package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class PivotConstants {
    public final static int MOTOR_CANID = 4;

    public final static DCMotor GEAR_BOX = DCMotor.getKrakenX60(1);
    private static final double MOMENT_OF_INERTIA = 0.001;                                  // TODO: change to correct
    private static final double PIVOT_LENGTH_METERS = 0.3;                                  // TODO: change to correct
    private static final Rotation2d PIVOT_CLOSE_ANGLE = Rotation2d.fromDegrees(67); // TODO: change to correct
    private static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0); // TODO: change to correct
    private static final boolean SIMULATE_GRAVITY = true;                              // TODO: change to wanted mode
    public static final double GEAR_RATIO = 90;                                              // TODO: change to correct


    /**
     * @return get the plant for the sim
     */
    private static LinearSystem<N2, N1, N2> getPlant(){
        return LinearSystemId.createSingleJointedArmSystem(
                GEAR_BOX,
                MOMENT_OF_INERTIA,
                GEAR_RATIO
        );
    }

    /**
     * @return the pivot's sim
     */
    public static SingleJointedArmSim getSim() {
        return new SingleJointedArmSim(
                getPlant(),
                GEAR_BOX,
                GEAR_RATIO,
                PIVOT_LENGTH_METERS,
                PIVOT_CLOSE_ANGLE.getRadians(),
                PIVOT_MAX_ANGLE.getRadians(),
                SIMULATE_GRAVITY,
                PIVOT_CLOSE_ANGLE.getRadians()
        );
    }


    public static TalonFXConfiguration getMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kP = 0;

        config.Slot0.kG = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 90;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        return config;        
    }
}
