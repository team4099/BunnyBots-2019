package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.tan

object Vision : Subsystem() {
    var state = VisionState.INACTIVE
    var onTarget = false
    var steeringAdjust = 0.0
    var distanceAdjust = 0.0

    // Data values from the camera reflections of the lime light?
    private val table: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight")

    var tx = table.getEntry("tx").getDouble(0.0)
    var tv = table.getEntry("tv").getDouble(0.0) // is this a valid target?
    var ty = table.getEntry("ty").getDouble(0.0)
    var ta = table.getEntry("ta").getDouble(0.0) // target area
    var distance = 0.0

    val pipeline: NetworkTableEntry = table.getEntry("pipeline")

    // Is it seeking for a light or is it aiming for a light?
    enum class VisionState {
        AIMING, INACTIVE, SEEKING
    }

    override val loop: Loop = object : Loop {
        override fun onStart(timestamp: Double) {
            state = VisionState.INACTIVE
        }

        override fun onLoop(timestamp: Double) {
            synchronized(this@Vision) {
                tx = table.getEntry("tx").getDouble(0.0)
                tv = table.getEntry("tv").getDouble(0.0)
                ty = table.getEntry("ty").getDouble(0.0)
                ta = table.getEntry("ta").getDouble(0.0)
                distance = (Constants.Vision.TARGET_HEIGHT - Constants.Vision.CAMERA_HEIGHT) /
                    tan(Constants.Vision.CAMERA_ANGLE + ty)

                when (state) {
                    VisionState.AIMING -> {
                        if (tv == 0.0) {
                            // TODO: rumble?
                        } else {
                            val distanceError = distance - Constants.Vision.SHOT_DISTANCE
                            steeringAdjust = tv * Constants.Vision.TURN_KP
                            distanceAdjust = distanceError * Constants.Vision.DISTANCE_KP
                            distanceAdjust += sign(distanceError) * Constants.Vision.MIN_COMMAND

                            if (abs(tx) < Constants.Vision.MAX_ERROR) onTarget = true
                        }
                    }
                    VisionState.SEEKING -> {
                        if (tv > 0.0) state = VisionState.AIMING
                        else {
                            steeringAdjust = 0.3
                            distanceAdjust = 0.0
                        }
                    }
                    VisionState.INACTIVE -> {
                        pipeline.setNumber(Constants.Vision.DRIVER_PIPELINE_ID)
                        steeringAdjust = 0.0
                        distanceAdjust = 0.0
                    }
                }
            }
        }

        override fun onStop(timestamp: Double) {
            stop()
        }
    }

    // we need to import smart dashboard
    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("LimelightX: ", tx)
        SmartDashboard.putNumber("LimelightTarget: ", tv)
        SmartDashboard.putNumber("LimelightY: ", ty)
    }

    override fun stop() {
        state = VisionState.INACTIVE // we want to stop this and to do that we need to set state to inactive
        pipeline.setNumber(Constants.Vision.DRIVER_PIPELINE_ID) // sets the pipeline ID
        steeringAdjust = 0.0
        distanceAdjust = 0.0
    }
}
