package frc.lib.motor;

import edu.wpi.first.math.system.plant.DCMotor;


//TODO make this class more accessable
public record MotorAttributes(int CAN_ID, DCMotor MOTOR, double GEAR_RATIO, double UNIT_CONVERSION, boolean IS_INVERTED) {

}
