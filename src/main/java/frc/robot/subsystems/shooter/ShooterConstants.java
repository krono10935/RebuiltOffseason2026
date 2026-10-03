package frc.robot.subsystems.shooter;

import frc.lib.statemachine.StateMachine.StateName;

public class ShooterConstants {

    /** max rotational (how much the robot is spinning) speed for robot until the shotcalculator doesn't think the parameter is valid*/
    public static final double ANGULAR_DEADBAND_DEGREES_PER_SECOND = 0; // TODO: configurate

    /** max linear (XY) speed until the shotcalculator doesn't think the parameter is valid*/
    public static final double LINEAR_SPEED_DEADBAND_MPS = 0; // TODO: configurate

    /** whether the robot should calculate shooter parameters with movement */
    public static final boolean SHOOT_WITH_MOVEMENT = false; // TODO: configurate

    /** state name for hood set angle */
    public static final StateName SET_ANGLE_STATE_NAME =  new StateName("set angle state");

    /** state name for hood hold angle */
    public static final StateName HOLD_ANGLE_STATE_NAME = new StateName("hold angle state");

    /** state name for flywheel set speed */
    public static final StateName SET_SPEED_STATE_NAME =  new StateName("set speed state");

    /** state name for flywheel hold speed */
    public static final StateName HOLD_SPEED_STATE_NAME = new StateName("hold speed state");
}
