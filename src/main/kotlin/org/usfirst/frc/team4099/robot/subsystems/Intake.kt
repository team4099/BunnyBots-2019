package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Intake private constructor() : Subsystem {
    private val talon = TalonSRX(Constants.Intake.INTAKE_TALON_ID)

    var intakeState = IntakeState.IN
    private var intakePower = 0.0

    enum class IntakeState {
        IN, STOP, OUT
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

    val loop: Loop = object : Loop {
        override fun onStart() {
            intakeState = IntakeState.STOP
        }

        /**
         * Sets Intake to -1 if pulling in, to 0 if stationary, and 1 if pushing out
         */
        override fun onLoop() {
            synchronized(this@Intake) {
                when (intakeState) {
                    IntakeState.IN -> setIntakePower(-1.0)
                    IntakeState.STOP -> setIntakePower(0.0)
                    IntakeState.OUT -> setIntakePower(1.0)
                }

                override fun onStop() = stop()

            }
        }
    }
    override fun stop() {
        intakeState = IntakeState.STOP
        setIntakePower(0.0)
    }

    override fun zeroSensors() {}
}