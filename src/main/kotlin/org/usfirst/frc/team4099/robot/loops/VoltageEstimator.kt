package org.usfirst.frc.team4099.robot.loops

import com.team2363.logger.HelixLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController

/** Constantly measures battery voltage before the match begins.
 *
 */

class VoltageEstimator : Loop {
    @get:Synchronized
    var averageVoltage = 12.0
        private set
    private val weight = 15.0

    init {
        HelixLogger.getInstance().addDoubleSource("VoltageEstimator Avg. Disabled Voltage") { averageVoltage }
    }

    override fun onStart() {
        println("Robot disabled: computing avg voltage")
    }

    @Synchronized override fun onLoop() {
        val curVoltage = RobotController.getBatteryVoltage()
        averageVoltage = (curVoltage + weight * averageVoltage) / (1.0 + weight)
    }

    override fun onStop() {
        println("Robot enabled: last avg voltage: $averageVoltage")
    }

    companion object {
        val instance = VoltageEstimator()
    }
}
