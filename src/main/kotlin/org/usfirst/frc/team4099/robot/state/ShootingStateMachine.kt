package org.usfirst.frc.team4099.robot.state

import com.team2363.logger.HelixEvents
import org.usfirst.frc.team4099.robot.loops.Loop
import org.usfirst.frc.team4099.robot.subsystems.Drive
import org.usfirst.frc.team4099.robot.subsystems.Subsystem
import org.usfirst.frc.team4099.robot.subsystems.Vision

object ShootingStateMachine : Subsystem() {
    var state = ShootingState.INACTIVE
        set(value) {
            if (value != field) {
                when (value) {
                    ShootingState.INACTIVE -> HelixEvents.addEvent("SHOOTING_STATE_MACHINE", "Inactive")
                    ShootingState.TARGETING -> HelixEvents.addEvent("SHOOTING_STATE_MACHINE", "Targeting")
                    ShootingState.IDLE_LINED_UP -> HelixEvents.addEvent("SHOOTING_STATE_MACHINE", "Lined up")
                    ShootingState.SHOOTING -> HelixEvents.addEvent("SHOOTING_STATE_MACHINE", "Started shooting")
                }
            }
            field = value
        }

    enum class ShootingState {
        INACTIVE,
        TARGETING,
        IDLE_LINED_UP,
        SHOOTING
    }

    override val loop: Loop = object : Loop {
        override fun onStart(timestamp: Double) {
            state = ShootingState.INACTIVE
        }

        override fun onLoop(timestamp: Double) {
            when (state) {
                ShootingState.INACTIVE -> {
                    Vision.state = Vision.VisionState.INACTIVE
                    // Shooter.state = INACTIVE
                }
                ShootingState.TARGETING -> {
                    Drive.setLeftRightPower(
                        Vision.distanceAdjust + Vision.steeringAdjust,
                        Vision.distanceAdjust - Vision.steeringAdjust
                    )

                    if (Vision.state == Vision.VisionState.INACTIVE) Vision.state = Vision.VisionState.SEEKING
                    if (Vision.onTarget) {
                        state = ShootingState.IDLE_LINED_UP
                    }
                }
                ShootingState.IDLE_LINED_UP -> {}
                ShootingState.SHOOTING -> TODO("set shooter state to shooting")
            }
        }

        override fun onStop(timestamp: Double) {
            stop()
        }
    }

    override fun outputToSmartDashboard() {}

    override fun stop() {
        state = ShootingState.INACTIVE
    }

    fun handleDriverDesires(aim: Boolean, shoot: Boolean) {
        state = when {
            (aim && state == ShootingState.INACTIVE) -> ShootingState.TARGETING
            (shoot && state == ShootingState.IDLE_LINED_UP || state == ShootingState.SHOOTING) -> ShootingState.SHOOTING
            (!shoot && state == ShootingState.SHOOTING) -> ShootingState.IDLE_LINED_UP
            (state == ShootingState.IDLE_LINED_UP) -> ShootingState.IDLE_LINED_UP
            else -> ShootingState.INACTIVE
        }
    }
}
