package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import kotlin.math.tan

object Vision : Subsystem() {
    var state = VisionState.INACTIVE
    var onTarget = false
    var steeringAdjust = 0.0
    var distanceAdjust = 0.0
    var distance = 0.0

    // Data values from the camera reflections of the lime light?
    private val table: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight")
    var tx = table.getEntry("tx").getDouble(0.0)
    var tv = table.getEntry("tv").getDouble(0.0) // is this a valid target?
    var ty = table.getEntry("ty").getDouble(0.0)
    var ta = table.getEntry("ta").getDouble(0.0) // target area

    var pipeline = table.getEntry("pipeline")

    // Is it seeking for a light or is it aiming for a light?
    enum class VisionState {
        AIMING, INACTIVE, ALIGNING
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
                distance = (Constants.Vision.TARGET_HEIGHT - Constants.Vision.CAMERA_HEIGHT) / tan(Constants.Vision.CAMERA_ANGLE + ty)
                when (state) {
                    VisionState.AIMING -> {
                        pipeline.setNumber(0)
                        if (tv == 0.0){
                        } else {
                            steeringAdjust = tx * Constants.Vision.AIMING_KP
                            //steeringAdjust += minCommand * sign(steeringAdjust)
                        }
                    }
                    VisionState.INACTIVE -> {
                        pipeline.setNumber(1)
                        steeringAdjust = 0.0
                        distanceAdjust = 0.0
                    }
                    VisionState.ALIGNING -> {
                        pipeline.setNumber(0)
                        if (tv == 0.0){
                        } else {
                            distanceAdjust = (Constants.Vision.SHOOTING_DISTANCE - distance) * Constants.Vision.ALIGNING_KP
                            //distanceAdjust += minCommand * sign(distanceAdjust)
                        }
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
    }
}
