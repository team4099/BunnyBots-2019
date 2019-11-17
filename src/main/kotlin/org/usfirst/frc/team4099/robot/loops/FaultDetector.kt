package org.usfirst.frc.team4099.robot.loops

import com.team2363.logger.HelixEvents

object FaultDetector: Loop {
    var rio3v3Faults = 0
        set(faults) {
            if (field != faults) {
                HelixEvents.getInstance().addEvent("Fault Detector", "roboRIO 3.3V Fault")
            }
            field = faults
        }
    var rio5vFaults = 0
        set(faults) {
            if (field != faults) {
                HelixEvents.getInstance().addEvent("Fault Detector", "roboRIO 5V Fault")
            }
            field = faults
        }
    var rio6vFaults = 0
        set(faults) {
            if (field != faults) {
                HelixEvents.getInstance().addEvent("Fault Detector", "roboRIO 6V Fault")
            }
            field = faults
        }

    override fun onStart() {}

    override fun onLoop() {

    }

    override fun onStop() {}
}