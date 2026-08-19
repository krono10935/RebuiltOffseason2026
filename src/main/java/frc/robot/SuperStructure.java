package frc.robot;

import frc.lib.statemachine.SuperStructureBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SuperStructure extends SuperStructureBase{
    
    private final Drivetrain drivetrain;

    private static SuperStructure instance = null;

    public static SuperStructure getInstance(){
        if (instance == null){
            instance = new SuperStructure();
        }

        return instance;
    }

    private SuperStructure(){
        super();

        drivetrain = RobotContainer.getInstance().getDrivetrain();

        configureBindings();
    }

    protected void configureBindings(){

    }
}
