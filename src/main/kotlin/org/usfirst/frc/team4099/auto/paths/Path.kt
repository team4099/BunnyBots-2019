package org.usfirst.frc.team4099.auto.paths
import org.usfirst.frc.team4099.robot.Constants
import java.io.File

class Path(path: FieldPaths) {
    private var leftAccelerations = ArrayList<Double>()
    private var rightAccelerations = ArrayList<Double>()
    private var leftVelocities = ArrayList<Double>()
    private var rightVelocities = ArrayList<Double>()
    private var leftDistances = ArrayList<Double>()
    private var rightDistances = ArrayList<Double>()
    private var robotHeadings = ArrayList<Double>()
    private val timeDelta: Double = Constants.Loopers.LOOPER_DT
    init {
        val leftFile: File = path.pathFileLeft
        val rightFile: File = path.pathFileRight
//        if (path == FieldPaths.LEFTH2_TO_LEFTROCKET3){
//            leftFile = File("/pathFiles/LeftH2ToLeftRocket3.left.pf1.csv")
//            rightFile = File("/pathFiles/LeftH2ToLeftRocket3.right.pf1.csv")
//        }
//        else if (path == FieldPaths.RIGHTH2_TO_RIGHTROCKET3){
//            leftFile = File("/pathFiles/RightH2ToRightRocket3.left.pf1.csv")
//            rightFile = File("/pathFiles/RightH2ToRightRocket3.right.pf1.csv")
//        }
//        else {
//            leftFile = File("")
//            rightFile = File("")
//        }
        fillLeftRight(leftFile, rightFile)
        println("******** ${leftVelocities.size}")
    }

    private fun fillLeftRight(leftTrajectory: File, rightTrajectory: File) {
        val linesL = leftTrajectory.readLines()
        for (i in 1..linesL.lastIndex) {
            val separated: List<String> = linesL[i].split(",")
            leftVelocities.add(separated[Constants.Paths.VELOCITY_CSV_INDEX].toDouble())
            leftAccelerations.add(separated[Constants.Paths.ACCELERATION_CSV_INDEX].toDouble())
            leftDistances.add(separated[Constants.Paths.DISTANCE_CSV_INDEX].toDouble())
            robotHeadings.add(separated[Constants.Paths.HEADING_CSV_INDEX].toDouble())
        }
        val linesR = rightTrajectory.readLines()
        for (i in 1..linesR.lastIndex) {
            val separated: List<String> = linesR[i].split(",")
            rightVelocities.add(separated[Constants.Paths.VELOCITY_CSV_INDEX].toDouble())
            rightAccelerations.add(separated[Constants.Paths.ACCELERATION_CSV_INDEX].toDouble())
            rightDistances.add(separated[Constants.Paths.DISTANCE_CSV_INDEX].toDouble())
        }
    }

    public fun getLeftVelocity(time: Double): Double {
        return leftVelocities.get((time / timeDelta).toInt())
    }
    public fun getRightVelocity(time: Double): Double {
        return rightVelocities.get((time / timeDelta).toInt())
    }
    public fun getLeftVelocityIndex(index: Int): Double {
        return leftVelocities.get(index)
    }
    public fun getRightVelocityIndex(index: Int): Double {
        return rightVelocities.get(index)
    }

    fun getLeftAccelerationIndex(index: Int): Double {
        return leftAccelerations[index]
    }

    fun getRightAccelerationIndex(index: Int): Double {
        return rightAccelerations[index]
    }

    fun getLeftDistanceIndex(index: Int): Double {
        return leftDistances.get(index)
    }
    fun getRightDistanceIndex(index: Int): Double {
        return rightDistances.get(index)
    }
    fun getHeading(time: Double): Double {
        return robotHeadings.get((time / timeDelta).toInt())
    }
    fun getHeadingIndex(index: Int): Double {
        return robotHeadings.get(index)
    }
    fun getTrajLength(): Int {
        return leftVelocities.size
    }
    fun getDeltaTime(): Double {
        return timeDelta
    }
}
