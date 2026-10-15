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


    public static final SuperStructureBase.SuperMode IDLE_STATE_NAME = new SuperStructureBase.SuperMode("IDLE");
    public static final SuperStructureBase.SuperMode SUPERSTRUCTURE_STATE_NAME = new SuperStructureBase.SuperMode("SUPERSTRUCTRUE");

    public static final SuperStructureBase.SuperMode SUPERSTRUCTURE_CHECK = new SuperStructureBase.SuperMode("SUPERSTRUCTURE_CHECK");
    public static final SuperStructureBase.SuperMode SUPERSTRUCTURE_SHOOTING_CHECK = new SuperStructureBase.SuperMode("SUPERSTRUCTURE_SHOOTING_CHECK");

    public static final SuperStructureBase.SuperMode NORMAL_CLOSED = new SuperStructureBase.SuperMode("NORMAL_CLOSED");
    public static final SuperStructureBase.SuperMode NORMAL_OPEN_OFF = new SuperStructureBase.SuperMode("NORMAL_OPEN_OFF");
    public static final SuperStructureBase.SuperMode NORMAL_OPEN_ON = new SuperStructureBase.SuperMode("NORMAL_OPEN_ON");
    public static final SuperStructureBase.SuperMode NORMAL_OUTTAKE = new SuperStructureBase.SuperMode("NORMAL_OUTTAKE");

    public static final SuperStructureBase.SuperMode SHOOTING_CLOSED = new SuperStructureBase.SuperMode("SHOOTING_CLOSED");
    public static final SuperStructureBase.SuperMode SHOOTING_INTAKE = new SuperStructureBase.SuperMode("SHOOTING_INTAKE");
}
