package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.subsystems.Drive
import kotlin.math.abs

/**
 * Created by Oksana on 2/16/2017.
 */
class TurnAction(angleToTurn: Double, slowMode: Boolean) : Action {
    private val drive = Drive.instance
    private val direction = if (angleToTurn > 0) 1 else -1
    private val angleToTurn = abs(angleToTurn)
    private var power = 0.5
    private var startAngle = 0.0
    private var done = false

    private val turnSignal = DriveSignal(direction * power, -direction * power)

    init {
        if (slowMode) {
            this.power = .2
        }
    }

    override fun isFinished(): Boolean {
        return abs(drive.angle - startAngle) >= angleToTurn || done
    }

    override fun update() {
        drive.setOpenLoop(turnSignal)
    }

    override fun done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL)
        println("------- END TURN -------")
    }

    override fun start() {
        startAngle = drive.angle
        println("------- NEW START AUTONOMOUS RUN -------")
        println("Starting angle: $startAngle")
    }
}
