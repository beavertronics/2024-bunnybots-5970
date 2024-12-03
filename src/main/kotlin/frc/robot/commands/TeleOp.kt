package frc.robot.commands
import beaverlib.utils.Sugar.within
import beaverlib.utils.Units.Electrical.volts
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command

import kotlin.math.*

import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    override fun initialize() {
        addRequirements(Drivetrain,Intake/*,Tote Grab,Vision??*/)
    }

    override fun execute() {
        // invert driving direction if trigger pressed
        if (OI.rightJoystickTrigger) Drivetrain.tankDrive(OI.leftJoystickMovementValue, OI.rightJoystickMovementValue)
        else Drivetrain.tankDrive(OI.leftJoystickMovementValue * -1.0, OI.rightJoystickMovementValue * -1.0)

        // run intake in one direction or the other
        if (OI.runIntakeDirection > 0.01) Intake.runIntake(3.volts) // todo fix voltage amount
        else if (OI.runIntakeDirection < -0.01) Intake.runIntake((3 * -1).volts) // todo fix voltage amount
        if (OI.runConveyorDirection > 0.01) Intake.runConveyor(3.volts) // todo fix voltage amount
        else if (OI.runConveyorDirection > -0.01) Intake.runConveyor((3 * -1).volts) // todo fix voltage amount

        // move intake up and down if button pressed
        if (OI.raiseIntake) Intake.raiseIntake()
        else if (OI.lowerIntake) Intake.lowerIntake()
    }

    // operator interface
    object OI {
        private val leftDriverController = Joystick(0)
        private val rightDriverController = Joystick(1)
        private val OperatorController = XboxController(2)

        private fun Double.processInput(deadzone : Double = 0.1, squared : Boolean = false, cubed : Boolean = false, readjust : Boolean = true) : Double{
            var processed = this
            if(readjust) processed = ((this.absoluteValue - deadzone)/(1 - deadzone))*this.sign
            return when {
                this.within(deadzone) ->    0.0
                squared ->                  processed.pow(2) * this.sign
                cubed   ->                  processed.pow(3)
                else    ->                  processed
            }
        }
        private fun Double.abs_GreaterThan(target: Double): Boolean{
            return this.absoluteValue > target
        }

        val leftJoystickMovementValue get() = leftDriverController.y.processInput()
        val rightJoystickMovementValue get() = rightDriverController.y.processInput()
        val rightJoystickTrigger get() = rightDriverController.triggerPressed
        val runIntakeDirection get() = OperatorController.rightY
        val runConveyorDirection get() = OperatorController.leftY
        val lowerIntake get() = OperatorController.yButtonPressed
        val raiseIntake get() = OperatorController.aButtonPressed
    }
}






































































































// uwu