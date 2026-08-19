package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.statemachine.StateMachine.StateName;
import frc.robot.subsystems.drivetrain.constants.ChassisType;

public class Constants {
    public static final ChassisType CHASSIS_TYPE = ChassisType.COMPBOT;
    public static final boolean IS_COMP = false;
    
    public static final Mode simMode = Mode.REAL;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final boolean isPit = false;//TODO update before match

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final double LOOP_PERIOD_SECONDS = 0.02;
    public static final boolean USE_OBJECT_DETECTION = true;

    public static final StateName IDLE_STATE_NAME = new StateName("IDLE");

}
