package org.usfirst.frc.team4099.robot

@Suppress("MagicNumber")
class Constants {
    object Universal {
        const val TIMEOUT = 10
        const val EPSILON = 1E-9
    }

    object Drive {
        const val LEFT_MASTER_ID = 10
        const val LEFT_SLAVE_1_ID = 9

        const val RIGHT_MASTER_ID = 5
        const val RIGHT_SLAVE_1_ID = 6

        const val STATUS_FRAME_PERIOD_MS = 5

        const val VOLTAGE_COMP_LEVEL = 12.0

        const val MAX_LEFT_OPEN_LOOP_POWER = 1.0
        const val MAX_RIGHT_OPEN_LOOP_POWER = 1.0

        const val LEFT_KV_FORWARD_HIGH = 0.4993
        const val RIGHT_KV_FORWARD_HIGH = 0.5412

        const val LEFT_KA_FORWARD_HIGH = 0.0468
        const val RIGHT_KA_FORWARD_HIGH = 0.0601

        const val LEFT_V_INTERCEPT_FORWARD_HIGH = 0.1879
        const val RIGHT_V_INTERCEPT_FORWARD_HIGH = 0.1364

        const val LEFT_KV_REVERSE_HIGH = 0.4987
        const val RIGHT_KV_REVERSE_HIGH = 0.5194

        const val LEFT_KA_REVERSE_HIGH = 0.0372
        const val RIGHT_KA_REVERSE_HIGH = 0.0644

        const val LEFT_V_INTERCEPT_REVERSE_HIGH = -0.1856
        const val RIGHT_V_INTERCEPT_REVERSE_HIGH = -0.2003

        const val FEET_PER_SEC_TO_NATIVE = 6.8 * 60.0 // 10.4

        const val CLOSED_LOOP_RAMP = 0.0

        const val PERCENT_DEADBAND = 0.04

        const val CONTINUOUS_CURRENT_LIMIT = 40

        const val AUTO_NOMINAL_OUTPUT = 0.0
        const val AUTO_PEAK_OUTPUT = 0.0

        const val NATIVE_TO_REVS = 12 / 2336

        const val WHEEL_DIAMETER_INCHES = 6
        const val WHEEL_TRACK_WIDTH_INCHES = 27
        const val WHEEL_GAIN = 0.05
        const val WHEEL_NON_LINEARITY = 0.05

        const val TRACK_SCRUB_FACTOR = 1.1

        const val GYRO_BAD_VALUE = -31337.0
    }

    object Gains {
        const val LEFT_LOW_KP = 0.0000 // .1 * 1500 / 70
        const val LEFT_LOW_KI = 0.0000
        const val LEFT_LOW_KD = 0.0000
        const val LEFT_LOW_KF = 0.0000 // 1023.0 / 2220.0

        // subject to change
        const val LEFT_HIGH_KP = 0.0000 // .1 * 1023 / 70
        const val LEFT_HIGH_KI = 0.0000
        const val LEFT_HIGH_KD = 0.0000
        const val LEFT_HIGH_KF = 0.0000 // 1023.0 / 4420.0

        const val RIGHT_LOW_KP = 0.0000 // .1 * 1500 / 70
        const val RIGHT_LOW_KI = 0.0000
        const val RIGHT_LOW_KD = 0.0000
        const val RIGHT_LOW_KF = 0.0000 // 1023.0 / 2220.0

        // subject to change
        const val RIGHT_HIGH_KP = 0.0000 // .1 * 1023 / 70
        const val RIGHT_HIGH_KI = 0.0000
        const val RIGHT_HIGH_KD = 0.0000
        const val RIGHT_HIGH_KF = 0.0000 // 1023.0 / 4420.0
    }

    object Loopers {
        const val LOOPER_DT = 0.02 // 50 Hz
    }

    object Dashboard {
        const val ALLIANCE_COLOR_KEY = "dashboard/allianceColor"
        const val ALLIANCE_OWNERSHIP_KEY = "dashboard/allianceOwnership"
    }

    object Autonomous {
        const val AUTO_OPTIONS_DASHBOARD_KEY = "autonomous/autoOptions"
        const val SELECTED_AUTO_MODE_DASHBOARD_KEY = "autonomous/selectedMode"

        const val AUTO_STARTS_DASHBOARD_KEY = "autonomous/autoStarts"
        const val SELECTED_AUTO_START_POS_KEY = "autonomous/selectedStart"

        const val SELECTED_AUTO_START_DELAY_KEY = "autonomous/selectedDelay"

        const val TURN_POWER = 0.5
        const val SLOW_TURN_POWER = 0.2

        const val FORWARD_POWER = 1.0
        const val SLOW_FORWARD_POWER = 0.6
        const val FORWARD_MAX_TIME_SECONDS = 3
        const val FORWARD_CORRECTION_KP = 0.01
        const val FORWARD_GIVE_UP_ANGLE = 30.0

        const val PATH_FOLLOW_TURN_KP = 0.8 * (-1.0 / 80.0)
    }

    object Paths {
        const val VELOCITY_CSV_INDEX = 4
        const val DISTANCE_CSV_INDEX = 3
        const val HEADING_CSV_INDEX = 7
        const val ACCELERATION_CSV_INDEX = 5
    }

    object Joysticks {
        const val DRIVER_PORT = 0
        const val SHOTGUN_PORT = 1

        const val QUICK_TURN_THROTTLE_TOLERANCE = 0.1
        const val THROTTLE_DEADBAND = 0.04
        const val TURN_DEADBAND = 0.035
    }

    object Intake {
        const val TALON_ID = 2
        const val CURRENT_LIMIT = 20
    }

    object BrownoutDefender {
        const val COMPRESSOR_STOP_VOLTAGE = 10
        const val COMPRESSOR_STOP_CURRENT = 70
    }
}
