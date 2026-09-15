package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

/**This class is used to coordinate the roller and pivot commands */
public class IntakeCoordinator {
    /**The pivot object we will be using */
    private final PivotSubsystem pivot;
    /**The roller object we will be using */
    private final RollerSubsystem roller;

    /**Create a new IntakeCoordinator */
    public IntakeCoordinator(){
        //TODO - we need to initialize the values in robotContainer
        pivot = new PivotSubsystem();
        roller = new RollerSubsystem();
    }
}
