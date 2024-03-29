package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.auto.AutoModeEndedException
import org.usfirst.frc.team4099.auto.actions.Action
abstract class AutoModeBase {
    protected var updateRate = 1.0 / 50.0
    var isActive = false
        protected set

    @Throws(AutoModeEndedException::class)
    abstract fun routine()

    fun run() {
        isActive = true
        try {
            routine()
        } catch (e: AutoModeEndedException) {
            println("Auto mode done, ended early")
            return
        }

        done()
        println("Auto mode done")
    }

    abstract fun done()

    fun stop() {
        isActive = false
    }

    val isActiveWithThrow: Boolean
        @Throws(AutoModeEndedException::class)
        get() {
            if (!isActive) {
                throw AutoModeEndedException()
            }
            return isActive
        }

    @Throws(AutoModeEndedException::class)
    fun runAction(action: Action) {
        isActiveWithThrow
        action.start()
        while (isActiveWithThrow && !action.isFinished()) {
            action.update()
            val waitTime = (updateRate * 1000.0).toLong()
            try {
                Thread.sleep(waitTime)
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
        action.done()
    }
}
