package frc.lib.math;

import com.ctre.phoenix6.configs.SlotConfigs;

public class PIDGains {
/**
     * The PID elements of the PID controller.
     */
    private final double k_P, k_I, K_D;

    /**
     * The integrator limiters.
     * This is used to limit the accumulation of the integrator to prevent windup.
     * The i_Zone is the zone where the integrator is active,
     * and the i_maxAccum is the maximum value that the integrator can accumulate.
     */
    private final double i_Zone, i_maxAccum;

    /**
     * The error tolerance of the PID controller.
     * Any error less than this value will be considered as at setpoint.
     */
    private final double tolerance;

    /**
     * Creates a PID gains object with the given values.
     * Use this constructor only if you know what you are doing.
     *
     * @param k_P        The proportional gain (>= 0) (units are volts per unit of control)
     * @param k_I        The integral gain (>= 0) (units are volt seconds per unit of control)
     * @param k_D        The derivative gain (>= 0) (units are volts per unit of control per second)
     * @param i_Zone     The integrator zone (>= 0) (units are unit of control).
     *                   If this is zero, I accumulation is disabled.
     *                   The Default value is infinity, meaning the integrator is always active.
     * @param i_maxAccum The maximum accumulation of the integrator (>= 0) (units are volts)
     *                   If this is zero, I accumulation is disabled.
     *                   The Default value is the maximum motor output of the motor controller.
     * @param tolerance  The tolerance of the PID controller (>= 0) (units are unit of control)
     */
    public PIDGains(double k_P, double k_I, double k_D, double i_Zone, double i_maxAccum, double tolerance) {
        if (k_P < 0) throw new IllegalArgumentException("k_P must be greater than zero");
        this.k_P = k_P;

        if (k_I < 0) throw new IllegalArgumentException("k_I must be greater than zero");
        this.k_I = k_I;

        if (k_D < 0) throw new IllegalArgumentException("k_D must be greater than zero");
        this.K_D = k_D;

        if (i_Zone < 0) throw new IllegalArgumentException("i_Zone must be greater than zero");
        this.i_Zone = i_Zone;

        if (i_maxAccum < 0) throw new IllegalArgumentException("i_maxAccum must be greater than zero");
        this.i_maxAccum = i_maxAccum;

        if (tolerance < 0) throw new IllegalArgumentException("tolerance must be greater than zero");
        this.tolerance = tolerance;
    }

    /**
     * Create an empty PIDGains object (all gains zero)
     */
    public PIDGains(){
        this(0, 0, 0, 0, 0, 0);
    }

    /**
     * Apply the FeedForwardGains object oa CTRE SlotConfigs Object
     * @param baseConfig the original slot config
     * @return the newly configurated SlotConfig from the gains
     */
    public SlotConfigs applyConfigCTRE(SlotConfigs baseConfig){
        return baseConfig
            .withKP(k_P)
            .withKI(k_I)
            .withKD(K_D);
    }

}
