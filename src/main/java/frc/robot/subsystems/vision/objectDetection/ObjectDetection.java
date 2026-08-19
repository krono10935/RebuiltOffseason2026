// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.objectDetection;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.subsystem.VirtualSubSystem;
import frc.robot.Constants;
import frc.robot.subsystems.vision.objectDetection.ObjectDetectionIO.ObjectDetectionIOReplay;

/** Add your docs here. */
public class ObjectDetection extends VirtualSubSystem {
    private static ObjectDetection instance = null;

    private final ObjectDetectionIO io;
    private final ObjectDetectionInputsAutoLogged inputs;

    private final Timer lastBallTimer = new Timer();
    private boolean shotLastBall = true;

    public static ObjectDetection getInstance(){
        if (instance == null){
            instance = new ObjectDetection();
        }
        return instance;
    }
    
    private ObjectDetection(){

        switch (Constants.currentMode) {
            case REAL -> io = new ObjectDetectionIOPhoton();

            case SIM -> io = new ObjectDetectionIOSim();
        
            default -> io = new ObjectDetectionIOReplay();
        }

        inputs = new ObjectDetectionInputsAutoLogged();
    }

    /**
     * if the camera sees any balls
     * if the camera is disconnected, it will return true
     * @return if the robot has balls
     */
    public boolean hasBalls(){
        return inputs.hasBalls || !shotLastBall || !Constants.USE_OBJECT_DETECTION ;
    }

    public Command waitUntilNoBalls(){
        return new WaitUntilCommand(() -> !this.hasBalls());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectDetection", inputs);

        if (inputs.hasBalls){
            shotLastBall = false;
            lastBallTimer.stop();
            lastBallTimer.reset();
        }

        Logger.recordOutput("ObjectDetection/lastBallTimer", lastBallTimer.get());
        Logger.recordOutput("ObjectDetection/hasBalls", hasBalls());

        if (!inputs.hasBalls && !shotLastBall && !lastBallTimer.isRunning()){
            lastBallTimer.start();
        } else if (lastBallTimer.hasElapsed(ObjectDetectionContstants.LAST_BALL_TIMEOUT)){
            shotLastBall = true;
            lastBallTimer.stop();
            lastBallTimer.reset();
        }
    }
}

