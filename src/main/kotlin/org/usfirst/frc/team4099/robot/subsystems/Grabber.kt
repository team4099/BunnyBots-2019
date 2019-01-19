package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Loop

class Grabber private constructor() : Subsystem{
    private val pneumaticShifter: DoubleSolenoid = DoubleSolenoid(Constants.Intake.SHIFTER_FORWARD_ID, Constants.Intake.SHIFTER_REVERSE_ID)
    private var pushStartTime = 0.0
    var push = false
        set (wantsPush) {
            if (wantsPush) {
                pushStartTime = (System.currentTimeMillis()).toDouble()
                pneumaticShifter.set(DoubleSolenoid.Value.kForward)
            } else {
                pneumaticShifter.set(DoubleSolenoid.Value.kReverse)
            }
            field = wantsPush
        }

    override fun outputToSmartDashboard() {
        SmartDashboard.putBoolean("intake/isPushed", push)
        SmartDashboard.putNumber("intake/current", BrownoutDefender.instance.getCurrent(7))
    }

    @Synchronized override fun stop() {
        pneumaticShifter.set(DoubleSolenoid.Value.kOff)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            push = false
            pneumaticShifter.set(DoubleSolenoid.Value.kOff)
        }

        override fun onLoop() {
            synchronized(this@Grabber) {
                if(push == true && System.currentTimeMillis() + 100 > pushStartTime){
                    push = false
                }
            }
        }

        override fun onStop() = stop()
    }

    companion object {
        val instance = Grabber()
    }

    override fun zeroSensors() { }
}