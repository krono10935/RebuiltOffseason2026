package frc.robot.subsystems.drivetrain.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants.ChassisSpeedConfig;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants.PPChassisConfig;
import frc.robot.subsystems.drivetrain.module.constants.SwerveModulesMK4;
import frc.robot.subsystems.drivetrain.module.constants.SwerveModulesMK5;

public enum ChassisType {
    DEVBOT(new ChassisConstants(
            SwerveModulesMK4.getConstants(),
            new ChassisSpeedConfig(1,4), SwerveModulesMK4.getGenericConf(),
            new PPChassisConfig(new PIDConstants(0),new PIDConstants(0)),14,
            new PathConstraints(0,0,0,0))),
    COMPBOT( new ChassisConstants(
        SwerveModulesMK5.getConstants(),
            new ChassisSpeedConfig(0.3,4.3), SwerveModulesMK5.getGenericConf(),
            new PPChassisConfig(new PIDConstants(9, 0.3,0.0267),new PIDConstants(6.7,0,0.067)),14,
            new PathConstraints(4.5,16,10,10)));

    public final ChassisConstants constants;

    ChassisType(ChassisConstants constants){
        this.constants = constants;
    }

}
