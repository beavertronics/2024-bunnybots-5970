package frc.robot.subsystems

import beaverlib.utils.Units.Electrical.VoltageUnit
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {

    val intakeMotor   = CANSparkMax(14,MotorType.kBrushed) // confirmed, 775 brushed motor
    val conveyorMotor = CANSparkMax(23,MotorType.kBrushed) // confirmed, 775 brushed motor
    val leftIntakeSolonoid = Solenoid(PneumaticsModuleType.CTREPCM, 0) // todo, fix channel
    val rightIntakeSolonoid = Solenoid(PneumaticsModuleType.CTREPCM, 1) // todo, fix channel

    init {
        listOf(intakeMotor, conveyorMotor).forEach {
            it.restoreFactoryDefaults()
            it.setSmartCurrentLimit(40)
        }
    }

    fun runIntake(speed: VoltageUnit) {
        intakeMotor.setVoltage(speed.asVolts)
    }

    fun runConveyor(speed: VoltageUnit) {
        conveyorMotor.setVoltage(speed.asVolts)
    }

    // todo test and make work
    fun raiseIntake() {
        leftIntakeSolonoid.set(false)
        rightIntakeSolonoid.set(false)
    }

    // todo test and make work
    fun lowerIntake() {
        leftIntakeSolonoid.set(true)
        rightIntakeSolonoid.set(true)
    }
}