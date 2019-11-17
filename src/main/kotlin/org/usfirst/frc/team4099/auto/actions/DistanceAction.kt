package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.subsystems.Drive
import kotlin.math.abs

class DistanceAction(inchesToMove: Double, slowMode: Boolean) : Action {
    private val drive = Drive.instance
    private val direction = if (inchesToMove > 0) 1 else -1
    private val inchesToMove: Double = abs(inchesToMove)
    private var startDist = 0.0
    private var otherStart = 0.0
    private var power = 1.0
    private var startAngle = 0.0
    private var done = false
    private var startTime = 0.0

    init {
        if (slowMode) {
            this.power = .6
        }
    }

    override fun isFinished(): Boolean {
        return abs(drive.getLeftDistanceInches()) - startDist >= inchesToMove || abs(drive.getRightDistanceInches()) - otherStart >= inchesToMove || done || Timer.getFPGATimestamp() - startTime > 3
    }

    override fun update() {
        val correctionAngle = startAngle - drive.angle
        if (abs(correctionAngle) > 30) {
            done = true
            return
        }
        drive.arcadeDrive(power * direction, correctionAngle * 0.01 * direction.toDouble())
        println("correctionAngle: $correctionAngle")
        SmartDashboard.putNumber("distanceInAction", abs(drive.getRightDistanceInches()) - otherStart)
    }

    override fun done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL)
        println("------- END DISTANCE -------")
    }

    override fun start() {
        startTime = Timer.getFPGATimestamp()
        startAngle = drive.angle
        println("------- NEW START AUTONOMOUS RUN -------")
        println("Starting angle: $startAngle")
        startDist = drive.getLeftDistanceInches()
        otherStart = drive.getRightDistanceInches()
    }
}