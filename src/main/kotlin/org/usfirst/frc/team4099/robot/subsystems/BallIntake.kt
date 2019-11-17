package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class BallIntake : Subsystem() {
    private val talon = TalonSRX(Constants.Intake.TALON_ID)

    init {
        talon.configContinuousCurrentLimit(Constants.Intake.CURRENT_LIMIT) // Max amount of current you can send to the motor.
        talon.setNeutralMode(NeutralMode.Coast)
        talon.inverted = false
    }

    enum class IntakeState {
        IN, STOP
    }

    var intakeStage = IntakeState.STOP

    override val loop = object : Loop {
        override fun onStart(timestamp: Double) {
            TODO("not implemented")
        }

        override fun onLoop(timestamp: Double) {
            TODO("not implemented")
        }

        override fun onStop(timestamp: Double) = stop()
    }

    override fun stop() {
        TODO("not implemented")
    }

    override fun outputToSmartDashboard() {
        TODO("not implemented")
    }
}
