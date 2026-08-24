package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.hardware.TalonFX;

public class PivotIOCTRE implements PivotIO {
    TalonFX motor;
    public PivotIOCTRE() {
        motor= new TalonFX(PivotConstants.);
        
    }
    
}
