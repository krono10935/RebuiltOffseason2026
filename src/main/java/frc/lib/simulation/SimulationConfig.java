package frc.lib.simulation;

import java.util.Optional;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.lib.math.FeedForwardsGains;
import frc.lib.math.UnitConversions;

/**
 * Immutable configuration describing how a DC-motor-driven mechanism should be simulated.
 *
 * <p>A mechanism can be modeled in one of two ways:
 * <ul>
 *   <li><b>Characterization-based</b>: uses the feedforward gains (kV and kA) obtained from
 *       system identification (e.g. SysId). See {@link #getCharacterizedBasedSimulation()}.</li>
 *   <li><b>Physics-based</b>: uses the mechanism's moment of inertia, gear ratio and motor
 *       model. See {@link #getMomentInertiaBasedSimulation()}.</li>
 * </ul>
 *
 * <p>Both methods produce a {@link LinearSystem} with two states (position, velocity),
 * one input (voltage) and two outputs (position, velocity), suitable for use with
 * WPILib simulation classes such as {@code DCMotorSim}.
 *
 * @param motorGains the characterized feedforward gains (kV, kA), or
 *                           {@link Optional#empty()} if the mechanism has not been characterized
 * @param momentInertia the mechanism's moment of inertia in kg·m², or {@code 0} if unknown
 * @param gearRatio          the gear reduction between the motor and the mechanism
 *                           (motor rotations per mechanism rotation)
 * @param motor              the motor model (type and count) driving the mechanism
 */
public record SimulationConfig (
    Optional<FeedForwardsGains> motorGains,
    double momentInertia,
    double gearRatio,
    DCMotor motor
    ) {


        /**
         * Creates a linear DC motor plant from the characterized feedforward gains.
         *
         * <p>The kV and kA gains stored in {@link #motorGains()} are expected to be in
         * per-RPM units and are converted to per-rotation-per-second units before being passed
         * to {@link LinearSystemId#createDCMotorSystem(double, double)}.
         *
         * @return a linear system with states [position, velocity], input [voltage]
         *         and outputs [position, velocity]
         * @throws IllegalStateException if missing characterized gains were configuration
         */
        public LinearSystem<N2,N1,N2> getCharacterizedBasedSimulation(){
            if (motorGains.isEmpty() || 
                motorGains.get().getkA() == 0 || 
                motorGains.get().getkV() == 0) {
                
                    throw new IllegalStateException("Missing configuration for kV and kA arguments!");
            }

            return LinearSystemId.createDCMotorSystem(
                UnitConversions.rotationsPerMinutetoRotationsPerSecond(
                    motorGains().get().getkV()),
                UnitConversions.rotationsPerMinutetoRotationsPerSecond(
                    motorGains().get().getkA()));
        }


        /**
         * Creates a linear DC motor plant from the mechanism's physical properties.
         *
         * <p>Intended to model the mechanism using {@link #momentInertiaBased()},
         * {@link #gearRatio()} and {@link #motor()}.
         *
         * @return a linear system with states [position, velocity], input [voltage]
         *         and outputs [position, velocity]
         * @throws IllegalStateException if the moment of inertia or gear ratio is {@code 0} (not configured)
         */
        public LinearSystem<N2,N1,N2> getMomentInertiaBasedSimulation(){
            if (momentInertia == 0 || gearRatio == 0) {
                throw new IllegalStateException("Missing configuration for momentInertia or gearRatio!");
            }
            
            return LinearSystemId.createDCMotorSystem(
                motorGains().get().getkV(),
                motorGains().get().getkA()
            );
        }
    }
