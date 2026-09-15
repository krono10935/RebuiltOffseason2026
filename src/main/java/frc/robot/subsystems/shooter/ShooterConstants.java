package frc.robot.subsystems.shooter;

import frc.lib.statemachine.StateMachine.StateName;

public class ShooterConstants {

    public static final double ANGULAR_DEADBAND_DEGREES_PER_SECOND = 0; // TODO: configurate
    public static final double LINEAR_SPEED_DEADBAND_MPS = 0; // TODO: configurate
    public static final boolean SHOOT_WITH_MOVEMENT = false; // TODO: configurate
    public static final StateName SET_ANGLE_STATE_NAME =  new StateName("set angle state");
    public static final StateName HOLD_ANGLE_STATE_NAME = new StateName("hold angle state");
    public static final StateName SET_SPEED_STATE_NAME =  new StateName("set speed state");
    public static final StateName HOLD_SPEED_STATE_NAME = new StateName("hold speed state");
}
