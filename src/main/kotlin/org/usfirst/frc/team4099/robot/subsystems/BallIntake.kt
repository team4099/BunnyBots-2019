package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class BallIntake: Subsystem() {
    private val talon = TalonSRX(Constants.Intake.INTAKE_TALON)

    init {
        talon.configPeakCurrentLimit(20) // Max amount of current you can send to the motor.
        talon.setNeutralMode(NeutralMode.Coast)
        talon.inverted = false
    }

    enum class IntakeStage {
        IN, STOP
    }

    var intakeStage = IntakeStage.STOP

    override val loop = object : Loop {
        override fun onStart() {
            TODO("not implemented")
        }

        override fun onLoop() {
            TODO("not implemented")
        }

        override fun onStop() = stop()
    }

    override fun stop() {
        TODO("not implemented")
    }

    override fun outputToSmartDashboard() {
        TODO("not implemented")
    }
}