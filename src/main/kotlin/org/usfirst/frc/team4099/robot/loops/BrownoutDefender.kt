package org.usfirst.frc.team4099.robot.loops

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.PowerDistributionPanel
import org.usfirst.frc.team4099.robot.Constants

// import org.usfirst.frc.team4099.robot.subsystems.Elevator

/** Manages the shutting off of components and subsystems when at risk of brownout.
 * It does this through a multitude of steps:
 * 1. Constantly monitor the Battery voltage
 * 2.
 * 3.
 */
class BrownoutDefender private constructor() : Loop {
    private val pdp = PowerDistributionPanel()
    // private val elevator = Elevator.instance
//    private val compressor = Compressor()

    override fun onStart(timestamp: Double) {
        pdp.clearStickyFaults()
    }

    override fun onLoop(timestamp: Double) {
//        if (pdp.voltage < Constants.BrownoutDefender.COMPRESSOR_STOP_VOLTAGE ||
//            pdp.totalCurrent > Constants.BrownoutDefender.COMPRESSOR_STOP_CURRENT) {
//            compressor.stop()
//        } else {
//            compressor.start()
//        }
    }

    override fun onStop(timestamp: Double) {}

    fun getCurrent(channel: Int): Double {
        return pdp.getCurrent(channel)
    }

    companion object {
        val instance = BrownoutDefender()
    }
}
