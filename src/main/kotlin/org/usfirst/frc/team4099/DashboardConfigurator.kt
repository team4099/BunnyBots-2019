package org.usfirst.frc.team4099

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.auto.modes.AutoModeBase
import org.usfirst.frc.team4099.auto.modes.StandStillMode

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
object DashboardConfigurator {
    fun getAutonomousMode(): AutoModeBase {
        return when (SmartDashboard.getString("/SmartDashboard/autonomous/selectedAutoMode", "empty")) {
            else -> StandStillMode()
        }
    }

    fun getStartingPosition(): StartingPosition {
        var selectedStartingPosition = StartingPosition.LEFT

        when (SmartDashboard.getString("/SmartDashboard/autonomous/selectedPosition", "empty")) {
            "LEFT" -> selectedStartingPosition = StartingPosition.LEFT
            "RIGHT" -> selectedStartingPosition = StartingPosition.RIGHT
            "CENTER" -> selectedStartingPosition = StartingPosition.CENTER
        }
        return selectedStartingPosition
    }

    fun getStartingHeight(): StartingHeight {
        var selectedStartingHeight = StartingHeight.H2

        when (SmartDashboard.getString("/SmartDashboard/autonomous/selectedHeight", "empty")) {
            "H1" -> selectedStartingHeight = StartingHeight.H1
            "H2" -> selectedStartingHeight = StartingHeight.H2
        }

        return selectedStartingHeight
    }

    enum class StartingPosition(val dashboardName: String) { // TODO: Investigate starting locations
        LEFT("LEFT"),
        CENTER("CENTER"),
        RIGHT("RIGHT")
    }

    enum class StartingHeight(val dashboardName: String) {
        H1("H1"),
        H2("H2")
    }
    enum class DesiredRocketSide(val dashboardName: String) {
        LEFT("LEFT"),
        RIGHT("RIGHT")
    }

    fun getIntakeMode(): String {
        return SmartDashboard.getString("intakeStartingState", "")
    }

    fun initDashboard() {
        TODO("add stuff here to get rid of this suppress")
    }
}
