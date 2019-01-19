package org.usfirst.frc.team4099.auto.paths
import java.io.File

class Path(path: FieldPaths) {
    var leftVelocities: ArrayList<Double> = ArrayList<Double>()
    var rightVelocities: ArrayList<Double> = ArrayList<Double>()
    var timeDelta: Double = 0.02
    init {
        val leftFile: File
        val rightFile: File
        if (path == FieldPaths.LEFTH2_TO_LEFTROCKET3){
            leftFile = File("/pathFiles/LeftH2ToLeftRocket3.left.pf1.csv")
            rightFile = File("/pathFiles/LeftH2ToLeftRocket3.right.pf1.csv")
        }
        else if (path == FieldPaths.RIGHTH2_TO_RIGHTROCKET3){

        }



    }
    private fun fillVelocities(leftTraj: File, rightTraj: File){
        var linesL:List<String> = leftTraj.readLines()
        for(i in 1..linesL.lastIndex){
            val separated:List<String> = linesL.get(i).split(",")
//            println("Velocity: " + separated.get(4) + " Acceleration: " + separated.get(5))
//            val velocity = separated.get(4)
//            val accel = separated.get(5)
            leftVelocities.add(separated.get(4).toDouble())
        }
        var linesR:List<String> = rightTraj.readLines()
        for(i in 1..linesR.lastIndex){
            val separated:List<String> = linesR.get(i).split(",")
//            println("Velocity: " + separated.get(4) + " Acceleration: " + separated.get(5))
//            val velocity = separated.get(4)
//            val accel = separated.get(5)
            rightVelocities.add(separated.get(4).toDouble())
        }
    }
    public fun getLeftVelocity(time: Double): Double {
        return leftVelocities.get((time/timeDelta).toInt())
    }
    public fun getRightVelocity(time: Double): Double {
        return rightVelocities.get((time/timeDelta).toInt())
    }
}