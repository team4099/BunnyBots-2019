package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.auto.paths.FieldPaths
import org.usfirst.frc.team4099.auto.paths.Path
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.subsystems.Drive

class FollowPathAction(path: FieldPaths) : Action {
    private val pathF = Path(path)

    override fun start() {
        Drive.enablePathFollow(pathF)
    }
    override fun update() {
    }
    override fun done() {
        Drive.setOpenLoop(DriveSignal.NEUTRAL)
    }
    override fun isFinished(): Boolean {
        return Drive.isPathFinished()
    }
}
