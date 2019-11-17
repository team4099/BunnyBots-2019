package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class IntakeBhog : Subsystem() {
    private val talon = TalonSRX(Constants.Intake.INTAKE_TALON)

    init {
        talon.configPeakCurrentLimit(20)
        talon.setNeutralMode(NeutralMode.Brake)
        talon.inverted = false
    }

    var intakeState = IntakeState.STOP

    enum class IntakeState {
        IN, STOP, OUT, SLOW
    }

    fun setIntakePower(power: Double) {
        talon.set(ControlMode.PercentOutput, power)
    }

    override val loop = object : Loop {
        override fun onStart() {
            intakeState = IntakeState.STOP
        }

        override fun onLoop() {
            synchronized(this@IntakeBhog) {
                when (intakeState) {
                    IntakeState.STOP -> {
                        setIntakePower(0.0)
                    }
                    IntakeState.IN -> {
                        setIntakePower(1.0)
                    }
                    IntakeState.OUT -> {
                        setIntakePower(-1.0)
                    }
                    IntakeState.SLOW -> {
                        setIntakePower(0.5)
                    }
                }
            }
        }

        override fun onStop() = stop()
    }

    override fun zeroSensors() {}

    override fun stop() {
        intakeState = IntakeState.STOP
        setIntakePower(0.0)
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putString("intake/intakeState", intakeState.toString())

    }
}