package org.usfirst.frc.team4099.robot.subsystems

import org.usfirst.frc.team4099.robot.loops.Loop

abstract class Subsystem {
    abstract val loop: Loop
    abstract fun outputToSmartDashboard()
    abstract fun stop()
    //abstract fun checkSystem()
    open fun zeroSensors() {}
    open fun readPeriodicInputs() {}
    open fun writePeriodicOutputs() {}
}
