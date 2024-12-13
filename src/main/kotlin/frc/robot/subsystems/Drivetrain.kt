package frc.robot.subsystems

import beaverlib.utils.Units.Linear.VelocityUnit
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.SubsystemBase
// import frc.engine.utils.`M/s`
import beaverlib.utils.Units.Linear.metersPerSecond
import beaverlib.utils.Units.Electrical.VoltageUnit

import edu.wpi.first.wpilibj2.command.Commands

object DriveConstants {
    const val MotorLMainID = 29
    const val MotorLSubID = 30
    const val MotorRMainID = 22
    const val MotorRSubID = 24
    const val KP = 0.0 // todo
    const val KS = 0.0 // todo
    const val KD = 0.0 // todo
    const val KV = 0.0 // todo
    const val KA = 0.0 // todo
    const val CurrentLimit = 40
}
object Drivetrain : SubsystemBase() {

    private val       leftMain = CANSparkMax(DriveConstants.MotorLMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val  leftSecondary = CANSparkMax(DriveConstants.MotorLSubID,  CANSparkLowLevel.MotorType.kBrushless)
    private val      rightMain = CANSparkMax(DriveConstants.MotorRMainID, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSecondary = CANSparkMax(DriveConstants.MotorRSubID,  CANSparkLowLevel.MotorType.kBrushless)

    val    leftEncoder: RelativeEncoder = leftMain.encoder
    val   rightEncoder: RelativeEncoder = rightMain.encoder

    private val drive = DifferentialDrive(leftMain, rightMain)

    private val leftPid = PIDController(DriveConstants.KP, 0.0, DriveConstants.KD)
    private val rightPid = PIDController(DriveConstants.KP, 0.0, DriveConstants.KD)
    private val FeedForward = SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA)

    private fun allMotors(code: CANSparkMax.() -> Unit) { //Run a piece of code for each drive motor controller.
        for (motor in listOf(leftMain, rightMain, leftSecondary, rightSecondary)) {
            motor.apply(code)
        }
    }

    init {
        leftSecondary.follow(leftMain)
        rightSecondary.follow(rightMain)

        allMotors {
            restoreFactoryDefaults()
            setSmartCurrentLimit(DriveConstants.CurrentLimit) //Todo: there's a fancy version of this function that may be worth using
            //TODO: finish initialize spark maxes
        }

        drive.setDeadband(0.0)

        leftMain.inverted = true
        leftSecondary.inverted = true

        /* TODO: set motor inversion correctly
        leftMain.inverted = false
        leftSecondary.inverted = false
        rightMain.inverted = true
        rightSecondary.inverted = true
        */
        //setDefaultCommand(Commands.run({stop()})) //Don't move unless we tell you to.
    }
    /** Drive by setting left and right power (-1 to 1).
     * @param left Power for left motors [-1.0.. 1.0]. Forward is positive.
     * @param right Voltage for right motors [-1.0.. 1.0]. Forward is positive.
     * */
    fun tankDrive(left: Double, right: Double) {
        drive.tankDrive(left, right, false)
    }
    /** Drive by setting left and right voltage (-12v to 12v)
     * @param left Voltage for left motors
     * @param right Voltage for right motors
     * */
    fun rawDrive(left: Double, right: Double) {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not neccesary?
        leftMain.setVoltage(left)
        rightMain.setVoltage(right)
    }
    fun stop(){
        rawDrive(0.0,0.0)
    }
    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: Double, right: Double) {
        leftPid.setpoint = left
        rightPid.setpoint = right

        val lPidCalculated = leftPid.calculate(leftEncoder.velocity)
        val rPidCalculated = rightPid.calculate(rightEncoder.velocity)

        val lFFCalculated = FeedForward.calculate(leftPid.setpoint)
        val rFFCalculated = FeedForward.calculate(rightPid.setpoint)

        rawDrive(lPidCalculated+lFFCalculated, rPidCalculated + rFFCalculated )
    }
    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: VelocityUnit, right: VelocityUnit) { closedLoopDrive(left.asMetersPerSecond, right.asMetersPerSecond) }
    fun closedLoopDrive(speeds: ChassisSpeeds){
        val kinematics = DifferentialDriveKinematics(OdometryConstants.TrackWidth.asMeters)
        val wheelSpeeds: DifferentialDriveWheelSpeeds = kinematics.toWheelSpeeds(speeds)
        closedLoopDrive(wheelSpeeds.leftMetersPerSecond,wheelSpeeds.rightMetersPerSecond)
    }
    val consumeDrive: (ChassisSpeeds) -> Unit = {
        closedLoopDrive(it)
    }

    // Drive the drivetrain at a specified voltage for specified amount of time.
    fun driveSeconds(left : VoltageUnit, right : VoltageUnit, secs: Double) 
    = Commands.run({Drivetrain.tankDrive(left.asVolts,right.asVolts)}, Drivetrain)
        .withTimeout(secs)


}