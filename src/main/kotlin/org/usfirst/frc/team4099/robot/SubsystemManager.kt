package org.usfirst.frc.team4099.robot

import com.team2363.logger.HelixEvents
import com.team2363.logger.HelixLogger
import org.usfirst.frc.team4099.robot.loops.Loop
import org.usfirst.frc.team4099.robot.subsystems.Subsystem

object SubsystemManager {
    val subsystems = mutableListOf<Subsystem>()
    val enabledLoop = object : Loop {
        override fun onStart() {
            subsystems.forEach { it.loop.onStart() }
        }

        override fun onLoop() {
            subsystems.forEach { it.loop.onLoop() }
            outputTelemetry()
        }

        override fun onStop() {
            subsystems.forEach { it.loop.onStop() }
        }
    }

    val disabledLoop = object : Loop {
        override fun onStart() {}

        override fun onLoop() {
            outputTelemetry()
        }

        override fun onStop() {}
    }

    fun register(subsystem: Subsystem) {
        subsystems.add(subsystem)
    }

    fun register(subsystemColl: Collection<Subsystem>) {
        subsystems.addAll(subsystemColl)
    }

    fun outputTelemetry() {
        subsystems.forEach{ it.outputToSmartDashboard() }
        HelixLogger.getInstance().saveLogs()
    }

    fun checkSubsystems() {
        TODO("implement checkSubsystems")
    }

    fun stop() {
        subsystems.forEach{ it.stop() }
    }
}