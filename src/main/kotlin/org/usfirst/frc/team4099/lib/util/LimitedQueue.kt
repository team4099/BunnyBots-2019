package org.usfirst.frc.team4099.lib.util

import java.util.LinkedList

class LimitedQueue<E>(private val limit: Int) : LinkedList<E>() {
    override fun add(element: E): Boolean {
        super.add(element)
        while (size > limit) {
            super.remove()
        }
        return true
    }
}
