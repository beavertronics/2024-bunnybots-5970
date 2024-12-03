package frc.robot.subsystems
//File adapted from 2898's 2023 Charged Up Code
import beaverlib.odometry.PoseProvider
import com.kauailabs.navx.frc.AHRS as NAVX
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Drivetrain
// import beaverlib.odometry.Vision Unused and causes compilation to fail when not commented out
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Linear.Acceleration
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Linear.metersPerSecond

object OdometryConstants {

    val TrackWidth = (23.13).inches // Taken from CAD
}


object Odometry : SubsystemBase(), PoseProvider {


    var navx = NAVX()
//    private val vision = Vision("testCamera")

    private val visionProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(OdometryConstants.TrackWidth.asMeters), navx.rotation2d, 0.0, 0.0, Pose2d())
    private val encoderOnly = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(OdometryConstants.TrackWidth.asMeters), navx.rotation2d, 0.0, 0.0, Pose2d())

    val leftVel get() =  Drivetrain.leftEncoder.velocity.metersPerSecond
    val rightVel get() = Drivetrain.rightEncoder.velocity.metersPerSecond
    val vels get() = DifferentialDriveWheelSpeeds(leftVel.asMetersPerSecond, rightVel.asMetersPerSecond)
    val chassisSpeeds get() = ChassisSpeeds(leftVel.asMetersPerSecond, rightVel.asMetersPerSecond, navx.rate)
    val getChassesSpeeds: () -> ChassisSpeeds = { chassisSpeeds }
    val getCurrentSpeeds: () -> DifferentialDriveWheelSpeeds = { vels }
    private val thirdProvider = DifferentialDrivePoseEstimator(DifferentialDriveKinematics(OdometryConstants.TrackWidth.asMeters), navx.rotation2d, 0.0, 0.0, Pose2d())

    override var pose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set
    val getPose: () -> Pose2d = { pose }

    val field = Field2d()
    val initial = Pose2d(11.789039, 0.74, Rotation2d.fromDegrees(0.0))

    fun zero() {
//        val initial = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))
        reset(initial)
    }

    override fun reset(x: DistanceUnit, y: DistanceUnit, theta: AngleUnit) {
        val p = Pose2d(x.asMeters, y.asMeters, Rotation2d.fromDegrees(theta.asRadians))
        visionProvider.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position, p)
        encoderOnly.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position, p)
    }

    override fun periodic() { 
        //Updates vision subsystem

//        val visionMeasurements = vision.getEstimatedPose(pose)
//        if(visionMeasurements != null){
////            visionProvider.setVisionMeasurementStdDevs(Constants.OdometryConstants.VisionDeviation)
//            visionProvider.addVisionMeasurement(visionMeasurements.estimatedPose.toPose2d(), visionMeasurements.timestampSeconds)
//        }

        pose = visionProvider.update(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position)
        encoderOnly.update(navx.rotation2d, Drivetrain.leftEncoder.position, Drivetrain.rightEncoder.position)

        field.robotPose = pose
        field.getObject("pure odometry").pose = encoderOnly.estimatedPosition
        SmartDashboard.putData(field)
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}