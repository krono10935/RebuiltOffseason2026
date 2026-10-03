package frc.robot.utils;

import org.littletonrobotics.conduit.ConduitApi;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpdateWidgets extends SubsystemBase{
    
    public UpdateWidgets(){

    }

    public void periodic(){
        SmartDashboard.putNumber("Battery Voltage", ConduitApi.getInstance().getPDPVoltage());
    }
}
