// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystem;

import java.util.ArrayList;

/** Add your docs here. */
public abstract class VirtualSubSystem {
    private static ArrayList<VirtualSubSystem> subSystems = new ArrayList<>();

    public static void virtualperiodic(){
        subSystems.forEach(VirtualSubSystem::periodic);
    }

    protected VirtualSubSystem(){
        subSystems.add(this);
    }

    public abstract void periodic();
}
