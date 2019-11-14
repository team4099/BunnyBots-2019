package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class IntakeBhog:Subsystem {

    private val talon = TalonSRX(Constants.Intake.INTAKE_TALON)

    var intakeState = IntakeState.STOP

    enum class IntakeState {
        IN, STOP, OUT, SLOW
    }



    override fun outputToSmartDashboard() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        SmartDashboard.putString("intake/intakeState", intakeState.toString())

    }

    init{
        talon.configPeakCurrentLimit(20)
        talon.setNeutralMode(NeutralMode.Brake)
        talon.inverted = false
    }

    override fun stop() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        intakeState = IntakeState.STOP
        setIntakePower(0.0)
    }

    fun setIntakePower(power: Double) {
        talon.set(ControlMode.PercentOutput,power)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
            intakeState = IntakeState.STOP
        }

        override fun onLoop() {
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
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

}