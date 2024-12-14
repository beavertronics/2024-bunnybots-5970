package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.initMotorControllers

object ToteGrabber : SubsystemBase() {


    init {
        initMotorControllers(40, CANSparkBase.IdleMode.kBrake, armMotor)
        armMotor.inverted = true
    }

    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("motor output", { armMotor.appliedOutput }) {}
    }
}