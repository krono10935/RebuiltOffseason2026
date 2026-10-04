package frc.lib.math;

import com.ctre.phoenix6.configs.SlotConfigs;


public class FeedForwardsGains {
    /**
     * The KG gain, used for systems like arms and elevators.
     * this feedforward applies only to position control.
     */
    private double kG;

    /**
     * A constant voltage added to the motor output based on the direction of travel.(volts)
     * This is used to counteract friction in the mechanism.
     */
    private double kS;


    /**
     * A voltage that is multiplied by the setpoint of the motor, then added to the output.(volts per unit of control)
     * Useful for mechanisms that require a voltage that is proportional to the setpoint.
     * For example, a flywheel that requires a certain voltage to reach a certain speed.
     * Used only for velocity control.
     */
    private double kV;

    /**
     * A voltage that is multiplied by the acceleration of the setpoint when using motion profiles.
     * This feedforward only applies when using motion profile.
     * This improves the behavior of the mechanism using a motion profile.
     */
    private double kA;


    /**
     * Creates a feed forward gain with the given values.
     *
     * @param kG                          The gravity feedForward gain
     * @param kS                          The friction feed forward gain (volts) (Greater than or equal to zero)
     * @param kV                          The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param kA                          The acceleration feedforward gain
     */
    public FeedForwardsGains(double kG, double kS,
                             double kV, double kA) {

        // kS can be negative, as it is a constant voltage added to the output
        this.kG = kG;

        if (kS < 0)
            throw new IllegalArgumentException("frictionFeedForward must be greater than or equal to zero");
        this.kS = kS;

        if (kV < 0)
            throw new IllegalArgumentException("setpointFeedForward must be greater than or equal to zero");
        this.kV = kV;

        if (kA < 0)
            throw new IllegalArgumentException("setpointFeedForward must be greater than or equal to zero");
        this.kA = kA;

    }

    /**
     * @param kA The acceleration feedforward gain     
     * @param kV The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param kS The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kA, double kV, double kS){
        this(0.0, kS, kV, kA);
    }

    /**
     * @param kV The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param kS The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV, double kS){
        this(0,kV,kS);
    }

    /**
     * @param kV The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV){
        this(kV,0);
    }

    /**
     * Create an all zero gains FeedForwardGains object
     */
    public FeedForwardsGains(){
        this(0);
    }

    /**
     * Apply the FeedForwardGains object oa CTRE SlotConfigs Object
     * @param baseConfig the original slot config
     * @return the newly configurated SlotConfig from the gains
     */
    public SlotConfigs applyConfigCTRE(SlotConfigs baseConfig){
        return baseConfig
            .withKA(kA)
            .withKV(kV)
            .withKS(kS)
            .withKG(kG);
    }

    /**
     * Makes a copy of the feedforward gains
     * @return A copy of the feedforward gains
     */
    public FeedForwardsGains copy(){
        return new FeedForwardsGains(
            kG,
            kS,
            kV,
            kA
        );
    }

    /**
     * Adds the kA feedForward to a copy of this object
     * @param kA the kA gain added to the FeedForwardGains
     * @return a new FeedForwardGains object that contains a new kA gain
     */
    public FeedForwardsGains withkA(double kA){
        FeedForwardsGains ffs = copy();

        ffs.kA = kA;

        return ffs;
    }

    /**
     * Adds the kV feedForward to a copy of this object
     * @param kV the kV gain added to the FeedForwardGains
     * @return a new FeedForwardGains object that contains a new kV gain
     */
    public FeedForwardsGains withkV(double kV){
        FeedForwardsGains ffs = copy();

        ffs.kV = kV;

        return ffs;
    }

    /**
     * Adds the kS feedForward to a copy of this object
     * @param kS the kS gain added to the FeedForwardGains
     * @return a new FeedForwardGains object that contains a new kS gain
     */
    public FeedForwardsGains withkS(double kS){
        FeedForwardsGains ffs = copy();

        ffs.kS = kS;

        return ffs;
    }

    /**
     * Adds the kG feedForward to a copy of this object
     * @param kG the kG gain added to the FeedForwardGains
     * @return a new FeedForwardGains object that contains a new kG gain
     */
    public FeedForwardsGains withkG(double kG){
        FeedForwardsGains ffs = copy();

        ffs.kG = kG;

        return ffs;
    }


    /**
     * @return The held kV in the FeedForwardGains
     */
    public double getkV(){
        return kV;
    }

    /**
     * @return The held kA in the FeedForwardGains
     */
    public double getkA(){
        return kA;
    }

    /**
     * @return The held kS in the FeedForwardGains
     */
    public double getkS(){
        return kS;
    }

    /**
     * @return The held kG in the FeedForwardGains
     */
    public double getkG(){
        return kG;
    }
}
