package com.team2898.robot.commands
// Adapted from 2898's 2022 bunnybot code
import beaverlib.odometry.Vision
import beaverlib.utils.Sugar.clamp
import frc.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.atan2
import kotlin.math.log

/** Robot moves within 1 meter of target, assuming Apriltag is within visual range of camera */
class HomingVision : Command() {
    val vision = Vision("Bob", null)

    val timer = Timer()
    override fun initialize() {
        timer.restart()
        println("homing starting")
    }
    override fun execute() {
        //Turns to find tag if not in cameras FOV
        if (vision.getTargets().isEmpty()) {
            Drivetrain.rawDrive(1.0, -1.0)
            return
        }
        val bestCamToTarget = vision.getTargets()[0].bestCameraToTarget
        val magnitude2D = bestCamToTarget.x
        // Makes sure the robot is not closer than 2 meters
        if (magnitude2D > 2 || magnitude2D == 0.0) {
            // Throttle reduces as it gets closer to target
            val speedMultiplier = (log(magnitude2D, 10.0)*3).clamp(0.5,3.0)
            val speeds = DifferentialDrive.curvatureDriveIK(1.0, atan2(bestCamToTarget.x, bestCamToTarget.z) / 1.5, true)
            Drivetrain.rawDrive(speeds.left * -speedMultiplier, speeds.right * -speedMultiplier)
        } else {
            Drivetrain.rawDrive(0.0,0.0) // Full stop
        }
//        println(Vision.timeSinceLastFix)
    }

    override fun isFinished(): Boolean {
//        println("checking if finished")
        if (!(timer.hasElapsed(3.0))) { /*println("in grace period, not stopping");*/ return false }
//        println("dist: ${Vision.magnitude2D}")
        val bestCamToTarget = vision.getTargets()[0].bestCameraToTarget
        if (vision.getTargets().isEmpty()) return false
        val magnitude2D = bestCamToTarget.x
        val inRange = magnitude2D <= 2
        val zeroedOut = magnitude2D == 0.0
        if (inRange && !zeroedOut) {
            println("stopping homing due to distance")
        }
        return inRange && !zeroedOut // Checks if it is closer than 2 meters
    }

    override fun end(interrupted: Boolean) {
        println("homing finished")
        Drivetrain.rawDrive(0.0,0.0) // Full stop
    }

}