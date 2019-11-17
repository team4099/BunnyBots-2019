package org.usfirst.frc.team4099.lib.util

import kotlin.math.abs

object Utils {
    /**
     * Limits the given input to the given magnitude.
     * @param v value to limit
     * @param limit limited magnitude
     * @return the limited value
     */

    fun limit(v: Double, limit: Double): Double {
        if (abs(v) < limit) {
            return v
        }
        return if (v < 0) {
            -limit
        } else {
            limit
        }
    }

    fun diff(current: Double, prev: Double): Double {
        return abs(current - prev)
    }

    fun around(value: Double, around: Double, tolerance: Double): Boolean {
        return diff(value, around) <= tolerance
    }

    fun sameSign(new: Double, old: Double): Boolean {
        return new >= 0 && old >= 0 || new <= 0 && old <= 0
    }

    fun sign(value: Double): Int {
        return if (value >= 0) 1 else -1
    }

    fun getAverageFromList(list: List<Double>): Double {
        var total = 0.0
        for (d in list) {
            total += d
        }
        return total / list.size
    }
}
