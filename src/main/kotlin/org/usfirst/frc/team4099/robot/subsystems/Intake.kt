package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Intake : Subsystem() {
    private val talon = TalonSRX(Constants.Intake.TALON_ID)

    var intakeState = IntakeState.IDLE

    enum class IntakeState {
        IN, IDLE, OUT
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putString("intake/intakeState", intakeState.toString())
    }

    init {
        talon.configPeakCurrentLimit(20)
        talon.setNeutralMode(NeutralMode.Brake)
        talon.inverted = false
    }

    @Synchronized

    fun setIntakePower(power: Double) {
        talon.set(ControlMode.PercentOutput, power)
    }

    override val loop = object : Loop {
        override fun onStart(timestamp: Double) {
            intakeState = IntakeState.IDLE
        }

        /**
         * Sets Intake to -1 if pulling in, to 0 if stationary, and 1 if pushing out
         */
        override fun onLoop(timestamp: Double) {
            synchronized(this@Intake) {
                when (intakeState) {
                    IntakeState.IN -> setIntakePower(-1.0)
                    IntakeState.IDLE -> setIntakePower(0.0)
                    IntakeState.OUT -> setIntakePower(1.0)
                }
            }
        }
        override fun onStop(timestamp: Double) = stop()
    }
    override fun stop() {
        intakeState = IntakeState.IDLE
        setIntakePower(0.0)
    }

    override fun zeroSensors() {}
}