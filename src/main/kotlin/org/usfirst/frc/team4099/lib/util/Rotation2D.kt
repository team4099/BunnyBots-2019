package org.usfirst.frc.team4099.lib.util

import org.usfirst.frc.team4099.robot.Constants
import java.text.DecimalFormat
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 *
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
class Rotation2D : Interpolable<Rotation2D> {

    protected var cosAngle: Double = 0.toDouble()
    protected var sinAngle: Double = 0.toDouble()

    @JvmOverloads constructor(x: Double = 1.0, y: Double = 0.0, normalize: Boolean = false) {
        cosAngle = x
        sinAngle = y
        if (normalize) {
            normalize()
        }
    }

    constructor(other: Rotation2D) {
        cosAngle = other.cosAngle
        sinAngle = other.sinAngle
    }

    /**
     * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this
     * object we might accumulate rounding errors. Normalizing forces us to
     * re-scale the sin and cos to reset rounding errors.
     */
    fun normalize() {
        val magnitude = Math.hypot(cosAngle, sinAngle)
        if (magnitude > Constants.Universal.EPSILON) {
            sinAngle /= magnitude
            cosAngle /= magnitude
        } else {
            sinAngle = 0.0
            cosAngle = 1.0
        }
    }

    fun cos(): Double {
        return cosAngle
    }

    fun sin(): Double {
        return sinAngle
    }

    fun tan(): Double {
        return if (cosAngle < Constants.Universal.EPSILON) {
            if (sinAngle >= 0.0) {
                java.lang.Double.POSITIVE_INFINITY
            } else {
                java.lang.Double.NEGATIVE_INFINITY
            }
        } else sinAngle / cosAngle
    }

    val radians: Double
        get() = atan2(sinAngle, cosAngle)

    val degrees: Double
        get() = Math.toDegrees(radians)

    /**
     * We can rotate this Rotation2D by adding together the effects of it and
     * another rotation.
     *
     * @param other
     * The other rotation. See:
     * https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    fun rotateBy(other: Rotation2D): Rotation2D {
        return Rotation2D(cosAngle * other.cosAngle - sinAngle * other.sinAngle,
                cosAngle * other.sinAngle + sinAngle * other.cosAngle, true)
    }

    /**
     * The inverse of a Rotation2D "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    fun inverse(): Rotation2D {
        return Rotation2D(cosAngle, -sinAngle, false)
    }

    override fun interpolate(other: Rotation2D, x: Double): Rotation2D {
        if (x <= 0) {
            return Rotation2D(this)
        } else if (x >= 1) {
            return Rotation2D(other)
        }
        val angleDiff = inverse().rotateBy(other).radians
        return this.rotateBy(fromRadians(angleDiff * x))
    }

    override fun toString(): String {
        val fmt = DecimalFormat("#0.000")
        return "(" + fmt.format(degrees) + " deg)"
    }

    companion object {
        var FORWARDS = fromDegrees(0.0)
        var BACKWARDS = fromDegrees(179.9)

        fun fromRadians(angleRadians: Double): Rotation2D {
            return Rotation2D(cos(angleRadians), sin(angleRadians), false)
        }

        fun fromDegrees(angleDegrees: Double): Rotation2D {
            return fromRadians(Math.toRadians(angleDegrees))
        }
    }
}
