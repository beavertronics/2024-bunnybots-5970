package com.team2898.robot.subsystems

import beaverlib.controls.BeaverSparkMax
import beaverlib.utils.MovingAverage
import beaverlib.utils.Sugar.degreesToRadians
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.initMotorControllers
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sin

object ToteConstants {
    const val ToteGrabberMotorID = 0 //todo
    const val ARM_MAXSPEED = 1.0 //todo
    const val ARM_MAXACCEL = 1.5 //todo
    const val ARM_RAISED_KSIN = 2.0 //todo
    const val ARM_RAISED_KS = 2.0 //todo
    const val ARM_RAISED_KP = 2.0 //todo
    const val ARM_RAISED_KI = 2.0 //todo
    const val ARM_RAISED_KD = 2.0 //todo
    const val ARM_LOWERED_KP = 2.0 //todo
    const val ARM_LOWERED_KI = 2.0 //todo
    const val ARM_LOWERED_KD = 2.0 //todo

    const val LOWER_SOFT_STOP = 0.0 //todo

    //TODO: Set up the proper positions for the arm
    enum class ArmHeights(val position: Double) {
        STOWED(LOWER_SOFT_STOP),
        PICKUP(0.55),
        READY(0.75)
    }
}

object ToteGrabber : SubsystemBase() {

    private val armMotor = CANSparkMax(ToteConstants.ToteGrabberMotorID, CANSparkLowLevel.MotorType.kBrushless)
//    private val brakeSolenoid = DoubleSolenoid(PNUEMATICS_MODULE, PNEUMATICS_MODULE_TYPE, DISK_BRAKE_FORWARD, DISK_BRAKE_BACKWARD)
    private val encoder = armMotor.encoder
//    private val limitSwitch = DigitalInput(ARM_LIMIT_SWITCH)
    private var lastTick = false

    var setpoint = pos()

    private const val UPPER_SOFT_STOP = 1.95
    val LOWER_SOFT_STOP = 9.429947216.degreesToRadians()
    private var stopped = false
    var ksin = 0.0 //todo
    var ks   = 0.0 //todo
    var kv   = 0.0 //todo

    fun pos(): Double {
        val p = encoder.position
        return p
    }

    val velocityAverage = MovingAverage(15)

    val profileTimer = Timer()

    val constraints = TrapezoidProfile.Constraints(
        ToteConstants.ARM_MAXSPEED,
        ToteConstants.ARM_MAXACCEL
    )
    private val pid = PIDController(0.0, 0.0, 0.0)
    var profile: TrapezoidProfile? = null
    private val integral = MovingAverage(50)

    init {
        initMotorControllers(40, CANSparkBase.IdleMode.kBrake, armMotor)
        armMotor.inverted = true
//        armMotor.encoder.velocityConversionFactor = PI * 2.0 / 525.0 / 42.0 / 3.33333333
        SmartDashboard.putNumber("arm kp", 0.0)
        SmartDashboard.putNumber("arm kd", 0.0)
        SmartDashboard.putNumber("arm ks", ks)
        SmartDashboard.putNumber("arm ksin", ksin)
        SmartDashboard.putNumber("arm kv", kv)
    }

    var last = pos()
    val timer = Timer()

    override fun periodic() {
        SmartDashboard.putNumber("arm current", armMotor.outputCurrent)
        SmartDashboard.putNumber("arm duty cycle", armMotor.appliedOutput)
        ks = SmartDashboard.getNumber("arm ks", ks)
        ksin = SmartDashboard.getNumber("arm ksin", ksin)
        kv = SmartDashboard.getNumber("arm kv", kv)

//        val currentTick = limitSwitch.get()
//        lastTick = limitSwitch.get()
        val currentTick = false

        val p = pos()
        //Change in position
        val dp = p - last
        last = p
        //Change in time
        val dt = timer.get()
        timer.restart()
        // Add the velocity to the moving average
        velocityAverage.add(dp / dt)
        // Get averages
        val rate = velocityAverage.average

        integral.add((rate - armMotor.encoder.velocity).absoluteValue)

//        SmartDashboard.putNumber("arm encoder difference", integral.average * integral.size)

        if (stopped) {
            println("STOPPED")
            armMotor.set(0.0)
            return
        }

        pid.p = SmartDashboard.getNumber("arm kp", 0.0)
        pid.d = SmartDashboard.getNumber("arm kd", 0.0)
        if (setpoint == 0.0 ||
            setpoint !in LOWER_SOFT_STOP..UPPER_SOFT_STOP ||
            ((p - setpoint).absoluteValue < 0.05 && rate.absoluteValue < 0.1) ||
            profileTimer.get() > (profile?.totalTime() ?: 0.0))
        {
            profile = null
        }



        var output = 0.0
        if (profile != null) {
            val targetSpeed = profile?.calculate(profileTimer.get())?.velocity ?: 0.0
            SmartDashboard.putNumber("arm target speed", targetSpeed)

            var output = pid.calculate(rate, targetSpeed)
            output += kv * targetSpeed
            output += ks + sin(p) * ksin
        }



        if (p > UPPER_SOFT_STOP) {
            output = output.coerceAtMost(0.0)
            println("UPPER SOFT STOP")
        } else if (p < LOWER_SOFT_STOP || currentTick) {
            output = output.coerceAtLeast(0.0)
            println("LOWER SOFT STOP")
        }
        armMotor.set(output)

    }

    fun setGoal(newPos: Double) {
        if (newPos !in LOWER_SOFT_STOP..UPPER_SOFT_STOP) return
//        currentGoal = newPos
        setpoint = newPos
        profile = TrapezoidProfile(constraints,
            TrapezoidProfile.State(newPos, 0.0),
            TrapezoidProfile.State(pos(), velocityAverage.average)
        )
        profileTimer.restart()
    }

    fun stop() {
        profile = null
    }

    fun isMoving(): Boolean {
        return profile != null
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("position", { pos() }) {}
        builder.addDoubleProperty("raw position", { encoder.position }) {}
        builder.addDoubleProperty("arm motor rate", { armMotor.encoder.velocity }) {}
        builder.addDoubleProperty("rate", { velocityAverage.average }) {}
        builder.addBooleanProperty("limit switch", { lastTick }) {}
        builder.addDoubleProperty("motor output", { armMotor.appliedOutput }) {}
    }
}