package frc.robot.subsystems
import beaverlib.controls.BeaverTalonSRX
import beaverlib.utils.Units.Electrical.VoltageUnit
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import frc.engine.utils.initMotorControllers
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel.MotorType

object Intake {

    val intakeMotor   = CANSparkMax(10,MotorType.kBrushed)   // todo fix ID, 775 brushed motor
    val conveyorMotor = CANSparkMax(11,MotorType.kBrushless) // todo fix ID, neo550
    val leftIntakeSolonoid = Solenoid(PneumaticsModuleType.CTREPCM, 0) // todo, fix channel
    val rightIntakeSolonoid = Solenoid(PneumaticsModuleType.CTREPCM, 1) // todo, fix channel

    init {
        initMotorControllers(40, intakeMotor, conveyorMotor) // todo change current limit?
    }

    fun runIntake(speed: VoltageUnit) {
        intakeMotor.setVoltage(speed)
    }

    fun runConveyor(speed: VoltageUnit) {
        conveyorMotor.setVoltage(speed)
    }

    fun raiseIntake() {
        leftIntakeSolonoid.set(false)
        rightIntakeSolonoid.set(false)
    }

    fun lowerIntake() {
        leftIntakeSolonoid.set(true)
        rightIntakeSolonoid.set(true)
    }
}