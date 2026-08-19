// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A declarative state machine that can be used to implement complex command routines. State machine
 * setup should be done in stages: first, a state machine is created and its name is set; second,
 * states are added to the state machine using {@link #addState(Command)}; third, transitions
 * between states can be specified using {@link State#switchTo(State)}:
 *
 * <pre>{@code
 * // Declare the state machine
 * StateMachine stateMachine = new StateMachine("Example State Machine");
 *
 * // Declare states
 * State state1 = stateMachine.addState(...);
 * State state2 = stateMachine.addState(...);
 * State state3 = stateMachine.addState(...);
 *
 * // Set initial state
 * stateMachine.setInitialState(state1);
 *
 * // Declare transitions
 * state1.switchTo(state2).when(...);
 * state2.switchTo(state3).when(...);
 * }</pre>
 *
 * <p>Every state in a state machine runs a single command. While a state's command is running, the
 * state machine will continually check all transitions that can be triggered from that state. If a
 * transition is triggered, the state machine will cancel the state's command and move to the next
 * state as defined by that transition. If no transition is triggered by the time the command
 * completes, the state machine will exit unless a {@link
 * TransitionNeedsConditionStage#whenComplete()} transition was specified from that state:
 *
 * <pre>{@code
 * // switch from state1 to state2 when foo is true
 * state1.switchTo(state2).when(() -> foo == true);
 *
 * // but if foo never becomes true, switch to state3 when state1 finishes
 * state1.switchTo(state3).whenComplete();
 *
 * // no transitions are defined from state2 or state3,
 * // so the state machine will exit when either state completes
 * }</pre>
 */
public final class StateMachine extends Command {
    private State initialState = null;
    private final Map<StateName,State> states = new HashMap<>();


    private State currentState = null;
    private boolean queuedTransition = false;

    
    /**
    * Creates a new state machine.
    *
    * @param name The name of the state machine. Cannot be null. This will appear in telemetry as the
    *     {@link Command#name() name} of the state machine.
    */
    public StateMachine(String name) {
        if (name == null || name.isEmpty()){
            throw new IllegalArgumentException("The state machine wants a name👹👹👹!");
        }

        setName(name);
    }

    public State getState(StateName statename){
        return states.get(statename);
    }

    /**
    * Adds a new state to the state machine. State transitions can be specified on the new state
    * using {@link State#switchTo(State)}.
    *
    * @param command The command for the state to execute. Cannot be null.
    * @param stateName The name of the state.
    * @return The newly created state.
    */
    public State addState(Command command, StateName stateName){
        if (command == null){
            throw new IllegalArgumentException("Command in state cannot be null🔥🔥🔥!");
        }
        if (stateName == null){
            throw new IllegalArgumentException("State must have name.");
        }

        var state = new State(this, command);
        states.put(stateName, state);
        return state;
    }

    /**
     * Sets up a transition from any of the given states to a specific state. If no states are given,
     * the transition will apply to all states in the state machine <i>at the time this method is
     * called</i>.
     *
     * <pre>{@code
     * stateMachine.switchFromAny(state1, state2, state3).to(state4).when(() -> foo == true);
     *
     * // Functionally equivalent to:
     * state1.switchTo(state4).when(() -> foo == true);
     * state2.switchTo(state4).when(() -> foo == true);
     * state3.switchTo(state4).when(() -> foo == true);
     *
     * // Set up an early exit condition from any state
     * stateMachine.switchFromAny().toExitStateMachine().when(() -> bar == true);
     *
     * // Functionally equivalent to:
     * state1.exitStateMachine().when(() -> bar == true);
     * state2.exitStateMachine().when(() -> bar == true);
     * state3.exitStateMachine().when(() -> bar == true);
     * state4.exitStateMachine().when(() -> bar == true);
     * }</pre>
     *
     * @param states The states to transition from.
     * @return A builder for the transition.
     */
    public TransitionNeedsTargetStage switchFromAny(State... states) {
        if (states.length == 0) {
        return new TransitionNeedsTargetStage(List.copyOf(this.states.values()));
        } else {
        return new TransitionNeedsTargetStage(List.of(states));
        }
    }

     /**
     * Sets the initial state for the state machine. This must be called before the state machine is
     * scheduled. Failure to do so will result in an {@link IllegalStateException} being thrown when
     * the state machine is started. TODO make some sort of warning if this method was never called
     *
     * @param initialState The new initial state. Cannot be null.
     */
    public void setInitialState(State initialState) {
        if (initialState == null){
            throw new IllegalArgumentException("The intial state must be initialized.");
        }

        if (!this.equals(initialState.stateMachine)) {
            throw new IllegalArgumentException("Cannot set initial state in a different state machine");
        }

        this.initialState = initialState;
    }
    

    @Override
    public void initialize(){
        if (initialState == null) {
            throw new IllegalStateException(
                getName() + " does not have an initial state😱😱😱. Use .setInitialState() to provide one.");
            }

        setCurrentState(initialState);
    }

    @Override
    public boolean isFinished(){
        Logger.recordOutput(getName(), currentState == null ? "None" : currentState.command.getName());

        return currentState == null;
    }

    @Override
    public void execute(){
        var currentCommand = currentState.command();

        if (queuedTransition){
            CommandScheduler.getInstance().schedule(currentCommand);

            currentState.runEnterCallbacks();
            queuedTransition = false;
            return;
        }

        Logger.recordOutput("Bruh", CommandScheduler.getInstance().isScheduled(currentState.command()));
        if (CommandScheduler.getInstance().isScheduled(currentState.command())){
            Logger.recordOutput("Tung",currentState.transitions().size());
            for (var transition : currentState.transitions()){
                Logger.recordOutput(transition.nextSupplier.get().command.getName(), transition.condition.getAsBoolean());
                if (transition.shouldTransition()){
                    // Cancel the current state's command and move to the next state specified by the
                    // transition. Break the state loop early to avoid an unnecessary yield() call and
                    // allow the next state's command to start in the same loop iteration that the
                    // previous state completed. If the next state is null, the state machine will exit
                    // immediately.
                    // Note: to prevent infinite loops when states transition to themselves, we require
                    // the transition signal to be a rising edge on the user-supplied condition to ensure
                    // that the transition is only triggered once per loop iteration.
                    currentState.runExitCallbacks();
                    currentCommand.cancel();
                    setCurrentState(verifyState(transition.nextState()));
                    return;
                }
            }
            return;
        }

        currentState.runExitCallbacks();
        setCurrentState(verifyState(currentState.nextState()));
    }

    @Override
    public void end(boolean isFinished){
        Logger.recordOutput(getName(), "None");
    }

    private void setCurrentState(State state){
        currentState = state;
        queuedTransition = true;

    }

    private State verifyState(State next) {
        if (next == null || this.equals(next.stateMachine)) {
            // OK
            return next;
        }

        // Bad user setup
        throw new IllegalStateException(
            "The next state does not belong to this state machine👺👺👺. Check the state for "
            + next.command().getName());
    }



    /**
    * A state in a state machine. Each state has a command that will be run when it is active. States
    * can transition to other states when some condition is met when that state is active, or
    * automatically transition to another state when it completes if no transition conditions were
    * met. A state with no transitions will never transition to another state, and will cause the
    * state machine to exit when the state completes; likewise, a state with no incoming transitions
    * will never be active.
    */
    public static final class State {
        /** The state machine that this state belongs to. */
        private final StateMachine stateMachine;

        /** The command that will run when this state is active. */
        private final Command command;

        /** The possible states to transition to when this state completes. */
        private final List<Completion> m_completions = new ArrayList<>();

        /** The state to transition to by default when this state completes. May be null. */
        private Supplier<State> defaultNextState = () -> null;

        /**
         * The transitions that can be triggered from this state. If multiple transitions are triggered
         * at once, the first transition in the list will be used.
         */
        private final List<Transition> transitions = new ArrayList<>();

        private final List<Runnable> enterCallbacks = new ArrayList<>();
        private final List<Runnable> exitCallbacks = new ArrayList<>();

        private State(StateMachine stateMachine, Command command ){
            this.stateMachine = stateMachine;
            this.command = command;
        }

        public Command command(){
            return command;
        }

        private void runEnterCallbacks() {
            enterCallbacks.forEach(Runnable::run);
        }

        private void runExitCallbacks() {
            exitCallbacks.forEach(Runnable::run);
        }

        public List<Transition> transitions() {
            return transitions;
        }

        private void addTransition(Transition transition) {
            transitions.add(transition);
        }

        /**
         * Sets the next state to transition to when this state completes without having fired a
         * transition first, or if no conditional completion transition has been met.
         *
         * @param nextState A supplier for the next state to transition to. Cannot be null, but may
         *     return null.
         */
        private void setNextState(Supplier<State> nextState) {
            defaultNextState = nextState;
        }


        private void addCompletion(BooleanSupplier condition, Supplier<State> next) {
            // Remove any preexisting completion with the same condition
            m_completions.removeIf(c -> c.getCondition() == condition);
            m_completions.add(new Completion(next, condition));
        }


        private State nextState() {
            for (var completion : m_completions) {
                if (completion.shouldTransition()) {
                return completion.nextState();
                }
            }

            // No conditional transition has been met, use the default next state.
            // If this was never set or was set to be null, the state machine will exit.
            return defaultNextState.get();
        }

        /**
         * Adds a function to be called when this state is entered. Callbacks are invoked immediately
         * after the state's command is scheduled, and are run in the same order they were added.
         *
         * <p>Note: if a callback schedules any commands, those commands will be scoped to the lifetime
         * of the entire robot, <i>not</i> this state's lifetime. Doing so is unrecommended
         * 
         *
         * @param callback The callback to run. Cannot be null.
         */
        public void onEnter(Runnable callback) {
            if (callback == null){
                throw new IllegalArgumentException("Java does not appreciate null runnables📞📞📞.");
            }

            enterCallbacks.add(callback);
        }

        /**
         * Adds a function to be called when this state is exited. Callbacks are invoked immediately
         * before the state's command is canceled, and are run in the order they were added. If the
         * command finishes naturally, the callbacks are run immediately after it completes and before
         * the next state is entered.
         *
         * @param callback The callback to run. Cannot be null.
         */
        public void onExit(Runnable callback) {
            if (callback == null){
                throw new IllegalArgumentException("Java does not appreciate null runnables📞📞📞.");
            }

            exitCallbacks.add(callback);
        }

        /**
     * Starts building a transition to the specified state.
     *
     * @param to The state to transition to. Cannot be null.
     * @return A builder for the transition.
     */
    public TransitionNeedsConditionStage switchTo(State to) {
        if (to == null){
            throw new IllegalArgumentException("Null is bad.");
        }

        if (!stateMachine.equals(to.stateMachine)) {
            throw new IllegalArgumentException(
                "Cannot transition to a state in a different state machine");
        }
        return new TransitionNeedsTargetStage(List.of(this)).to(to);
        }
    }

    /**
   * A builder for a transition from one state to another. Use {@link #to(State)} to specify the
   * target state to transition to.
   */
    public static final class TransitionNeedsTargetStage {
        private final List<State> from;

        private TransitionNeedsTargetStage(List<State> from) {
            this.from = from;
        }

        /**
         * Specifies the target state to transition to.
         *
         * @param to The state to transition to. Cannot be null.
         * @return A builder to specify the transition condition.
         */
        public TransitionNeedsConditionStage to(State to) {
            if (to == null){
                throw new IllegalArgumentException("No nulls allowed 😤😤😤.");
            }

            for (var state : from) {
                if (!state.stateMachine.equals(to.stateMachine)) {
                    throw new IllegalArgumentException(
                        "Cannot transition to a state in a different state machine");
                }
            }
            return new TransitionNeedsConditionStage(from, () -> to);
        }

        /**
         * Specifies a dynamic target state to transition to. The supplier will be evaluated at the time
         * the transition condition is met.
         *
         * @param dynamic A dynamic supplier for next states. Cannot be null.
         * @return A builder to specify the transition condition.
         */
        public TransitionNeedsConditionStage to(Supplier<State> dynamic) {
            if (dynamic == null){
                throw new IllegalArgumentException("The dynamic state supplier may not be null💀!");
            }
            return new TransitionNeedsConditionStage(from, dynamic);
        }

        /**
         * Specifies the transition will exit the state machine when triggered, rather than moving to a
         * different state.
         *
         * @return A builder to specify the transition condition.
         */
        public TransitionNeedsConditionStage toExitStateMachine() {
            return new TransitionNeedsConditionStage(from, () -> null);
        }
    }

    /**
   * A builder to set conditions for a transition from one state to another. Use {@link
   * #when(BooleanSupplier)} to make the transition occur when some external condition becomes true,
   * or use {@link #whenComplete()} to make the transition occur when the originating state
   * completes without having reached any other transitions first.
   */
    public static final class TransitionNeedsConditionStage {
        private final List<State> originatingStates;

        // Note: A null result from the supplier indicates that the transition will cause the state
        //       machine to exit
        private final Supplier<State> targetStateSupplier;

        private TransitionNeedsConditionStage(List<State> from, Supplier<State> to) {
            if (from == null){
                throw new IllegalArgumentException("The list of originating states cannot be null💗!");
            }
            if (to == null){
                throw new IllegalArgumentException("The target state supplier cannot be null🥀!");
            }

            originatingStates = from;
            targetStateSupplier = to;
        }

        /**
         * Adds a transition that will be triggered when the specified condition becomes true.
         *
         * <p><strong>NOTE: this had no effect if the originating state is a InstantCommand </strong> Use {@link #whenComplete()} instead for transitions from InstantCommands.
         *
         * <p>If multiple transitions are triggered in the same scheduler loop iteration, the first
         * transition will fire and the rest will be ignored.
         *
         * <pre>{@code
         * StateMachine stateMachine = new StateMachine("Example State Machine");
         * State state1 = stateMachine.addState(...);
         * State state2 = stateMachine.addState(...);
         * State state3 = stateMachine.addState(...);
         *
         * state1.switchTo(state2).when(() -> foo == true);
         *
         * // never triggers because the first transition will be evaluated first
         * state1.switchTo(state3).when(() -> foo == true);
         * }</pre>
         *
         * @param condition The condition that will trigger the transition. Cannot be null.
         */
        public void when(BooleanSupplier condition) {
            if (condition == null){
                throw new IllegalArgumentException("Conditions cannot be null");
            }

            var transition = new Transition(targetStateSupplier, condition);
            originatingStates.forEach(originatingState -> originatingState.addTransition(transition));
        }

        /**
         * Adds a transition to the target state when the originating state completes without having
         * triggered any other transitions first. If this is called multiple times for the same
         * originating state, later calls will override the previous transitions. Any {@link
         * #whenCompleteAnd} transitions will take precedence over {@code whenComplete} transitions if
         * their conditions are met when the state exits.
         *
         * <pre>{@code
         * StateMachine stateMachine = new StateMachine("Example State Machine");
         * State state1 = stateMachine.addState(...);
         * State state2 = stateMachine.addState(...);
         * State state3 = stateMachine.addState(...);
         *
         * state1.switchTo(state2).whenComplete();
         * state1.switchTo(state3).whenComplete(); // Overrides the previous transition
         * state1.exitStateMachine().whenCompleteAnd(...); // Takes precedence if the condition is met
         * }</pre>
         */
        public void whenComplete() {
            originatingStates.forEach(state -> state.setNextState(targetStateSupplier));
        }

        /**
         * Similar to {@link #when(BooleanSupplier)}, but only triggers when the originating state
         * completes <i>and</i> some other condition is also met. {@code whenCompleteAnd} transitions
         * will be evaluated in declaration order and take precedence over any {@link #whenComplete()}
         * transitions that have been specified.
         *
         * <pre>{@code
         * StateMachine stateMachine = new StateMachine("Example State Machine");
         * State state1 = stateMachine.addState(...);
         * State state2 = stateMachine.addState(...);
         * State state3 = stateMachine.addState(...);
         *
         * state1.switchTo(state2).whenComplete();
         * state1.switchTo(state3).whenComplete(); // Overrides the previous transition
         * state1.exitStateMachine().whenCompleteAnd(...); // Takes precedence if the condition is met
         * }</pre>
         *
         * @param condition The condition that will trigger the transition.
         */
        public void whenCompleteAnd(BooleanSupplier condition) {
            if (condition == null){
                throw new IllegalArgumentException("Conditions cant be null.");
            }

            originatingStates.forEach(state -> state.addCompletion(condition, targetStateSupplier));
        }
    }

    
    /**
    * Similar to {@link Completion}, but tracks the state of the condition to avoid infinite loops.
    * This is intended to be checked every loop while the originating state is active.
    */
    private static final class Transition {

            /** The state to transition to. */
        private final Supplier<State> nextSupplier;

        /** The condition that will trigger the transition. */
        private final BooleanSupplier condition;

        private boolean previousSignal = false;

        private Transition(Supplier<State> next, BooleanSupplier condition) {
            this.nextSupplier = next;
            this.condition = condition;
        }


         /** Checks if the transition should be triggered. */
        private boolean shouldTransition() {
            // Wrap the condition in a rising edge detector so that it will only trigger a single time per
            // loop iteration. This prevents issues with a state transitioning to itself like so:
            // state1.switchTo(state1).when(() -> foo == true);
            // If the condition is itself a rising edge detector, this wrapping is redundant but harmless.
            boolean currentValue = condition.getAsBoolean();
            boolean isRisingEdge = currentValue && previousSignal;
            previousSignal = currentValue;
            return isRisingEdge;
        }

        private State nextState() {
            return nextSupplier.get();
        }

    }

    /**
    * Similar to {@link Transition}, but does not track the state of the condition. This is intended
    * to only be checked once, when the originating state completes.
    */
    private static final class Completion {
        private final Supplier<State> nextSupplier;
        private final BooleanSupplier condition;

        /**
         * Creates a new completion object.
         *
         * @param next A supplier for the state to transition to when the originating state completes.
         * @param condition The condition that will trigger the transition.
         */
        private Completion(Supplier<State> next, BooleanSupplier condition) {
            this.nextSupplier = next;
            this.condition = condition;
        }

        private boolean shouldTransition() {
            return condition.getAsBoolean();
        }

        public State nextState() {
            return nextSupplier.get();
        }

        public BooleanSupplier getCondition() {
            return condition;
        }
    }

    public static class StateName{
        private String name;
        
        public StateName(String name){
            this.name = name;
        }

        public boolean equals(StateName other) {
            return name.equals(other.name);
        }
    }
}
