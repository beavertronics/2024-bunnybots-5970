package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.initMotorControllers
import beaverlib.utils.Units.Electrical.VoltageUnit
import com.revrobotics.SparkLimitSwitch;

object ToteGrabber : SubsystemBase() {

    private val armMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushed) // Verified, 775 brushed
    private val forwardSwitch = armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed) // TODO Not verified- limit switches may be the other way around and they may be normally open instead of NC
    private val reverseSwitch = armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed)

    init {
        initMotorControllers(40, CANSparkBase.IdleMode.kBrake, armMotor)
        forwardSwitch.enableLimitSwitch(true); //Make 100% sure limit switches are enabled
        reverseSwitch.enableLimitSwitch(true);
        armMotor.inverted = true
    }

    fun runToteGrab(speed: VoltageUnit) {
        armMotor.setVoltage(speed.asVolts)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("motor output", { armMotor.appliedOutput }) {}
    }
}