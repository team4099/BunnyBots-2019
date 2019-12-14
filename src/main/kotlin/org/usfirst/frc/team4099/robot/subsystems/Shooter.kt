package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ControlType
import org.usfirst.frc.team4099.lib.util.CANMotorControllerFactory
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

object Shooter : Subsystem() {
    enum class ShooterState {
        SHOOTING,
        REVERSING,
        NOT_SHOOTING
    }

    var state = ShooterState.NOT_SHOOTING

    val leftFlywheel = CANSparkMax(Constants.Shooter.LEFT_FLYWHEEL_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val rightFlywheel = CANSparkMax(Constants.Shooter.RIGHT_FLYWHEEL_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    val accelerator = CANMotorControllerFactory.createDefaultTalon(Constants.Shooter.ACCELERATOR_ID)
    val feeder = CANMotorControllerFactory.createDefaultTalon(Constants.Shooter.FEEDER_ID)

    val flywheelPIDController = leftFlywheel.pidController

    init {
        rightFlywheel.follow(leftFlywheel)
        flywheelPIDController.p = Constants.Shooter.FLYWHEEL_P_GAIN
        flywheelPIDController.i = Constants.Shooter.FLYWHEEL_I_GAIN
        flywheelPIDController.d = Constants.Shooter.FLYWHEEL_D_GAIN
        flywheelPIDController.ff = Constants.Shooter.FLYWHEEL_F_GAIN
    }

    override val loop = object : Loop {
        override fun onStart(timestamp: Double) {
            state = ShooterState.NOT_SHOOTING
        }

        override fun onLoop(timestamp: Double) {
            when (state) {
                ShooterState.SHOOTING -> {
                    // TODO: do this
                    setVelocityPID(Constants.Shooter.TARGET_FLYWHEEL_SPEED)

                    accelerator.set(ControlMode.PercentOutput, 1.0)
                    feeder.set(ControlMode.PercentOutput, 1.0)
                }

                ShooterState.REVERSING -> {
                    leftFlywheel.set(-1.0)
                    accelerator.set(ControlMode.PercentOutput, -1.0)
                    feeder.set(ControlMode.PercentOutput, -1.0)
                }

                ShooterState.NOT_SHOOTING -> {
                    leftFlywheel.set(0.0)
                    accelerator.set(ControlMode.PercentOutput, 0.0)
                    feeder.set(ControlMode.PercentOutput, 0.0)
                }
            }
        }

        override fun onStop(timestamp: Double) {
            TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
        }
    }

    fun setVelocityPID(velocity: Double) {
        flywheelPIDController.setReference(velocity, ControlType.kVelocity)
    }

    fun setVelocityBangBang(velocity: Double) {
        // TODO: Maybe do this later
    }

    override fun outputToSmartDashboard() {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }

    override fun stop() {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }
}