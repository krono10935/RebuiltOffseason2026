package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.statemachine.StateMachine;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.statemachine.StateMachine.StateName;
import frc.lib.statemachine.SuperStructureBase;
import frc.robot.subsystems.drivetrain.constants.ChassisType;

public class Constants {
    public static final ChassisType CHASSIS_TYPE = ChassisType.COMPBOT;
    public static final boolean IS_COMP = false;
    public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;
    
    public static final Mode simMode = Mode.REAL;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static boolean isPit = false; //TODO update before match

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

    public static final StateName PIVOT_OPEN_ROLLER_ON_NAME = new StateName("PIVOT_OPEN_ROLLER_ON");
    public static final StateName PIVOT_OPEN_ROLLER_OFF_NAME = new StateName("PIVOT_OPEN_ROLLER_OFF");
    public static final StateName PIVOT_CLOSE_ROLLER_OFF_NAME = new StateName("PIVOT_CLOSE_ROLLER_OFF");
    public static final StateName OUTTAKE_NAME = new StateName("OUTTAKE");

    public static final StateName ACTIVATE_SHOOTING_NAME = new StateName("ACTIVATE_SHOOTING");
    public static final StateName DISABLE_SHOOTING_NAME = new StateName("DISABLE_SHOOTING");

    public static final StateName INDEXER_ON_NAME = new StateName("INDEXER_ON");
    public static final StateName INDEXER_OFF_NAME = new StateName("INDEXER_OFF");
    public static final StateName INDEXER_REVERSED_NAME = new StateName("INDEXER_REVERSED");

    public static final StateName INTAKING_CHECK_INPUTS_NAME = new StateName("INTAKE_CHECK_INPUTS");
    public static final StateName SHOOTING_CHECK_INPUTS_NAME = new StateName("SHOOTING_CHECK_INPUTS");
    public static final StateName INTAKING_SHOOTING_CHECK_INPUTS_NAME = new StateName("INTAKING_SHOOTING_CHECK_INPUTS");
    public static final StateName SHOOTING_INDEXER_CHECK_INPUTS_NAME = new StateMachine.StateName("SHOOTING_INDEXER_CHECK_INPUTS");
    public static final StateName INTAKING_INDEXER_CHECK_INPUTS_NAME = new StateMachine.StateName("INTAKING_INDEXER_CHECK_INPUTS");

    public static final SuperStructureBase.SuperMode RESET_GYRO_NAME = new SuperStructureBase.SuperMode("RESET_GYRO");

    public static final StateName DRIVE_MODE = new StateName("DRIVE_MODE");
    public static final StateName DEFENCE_MODE = new StateName("DEFENCE_MODE");

    public static final SuperStructureBase.SuperMode SHOOTING_MODE_STATE_NAME = new SuperStructureBase.SuperMode("SHOOTING_STATE");
    public static final SuperStructureBase.SuperMode INTAKING_MODE_STATE_NAME = new SuperStructureBase.SuperMode("INTAKING_STATE");
    public static final SuperStructureBase.SuperMode IDLE_STATE_NAME = new SuperStructureBase.SuperMode("IDLE");

}
