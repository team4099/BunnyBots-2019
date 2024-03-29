package org.usfirst.frc.team4099.auto.actions

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions report being done.
 *
 * @param actions
 * List of Action objects
 */
class ParallelAction(actions: List<Action>) : Action {
    private val actions = mutableListOf<Action>()

    init {
        for (action in actions) {
            this.actions.add(action)
        }
    }

    override fun isFinished(): Boolean {
        var allFinished = true
        for (action in actions) {
            if (!action.isFinished()) {
                allFinished = false
            }
        }
        return allFinished
    }

    override fun update() {
        for (action in actions) {
            action.update()
        }
    }

    override fun done() {
        for (action in actions) {
            action.done()
        }
    }

    override fun start() {
        for (action in actions) {
            action.start()
        }
    }
}
