package frc.robot.commands
import beaverlib.utils.Sugar.within
import beaverlib.utils.Units.Electrical.volts
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command

import kotlin.math.*

import frc.robot.subsystems.Drivetrain
//import frc.robot.subsystems.Intake
//import frc.robot.subsystems.ToteGrabber

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.robot.subsystems.Intake
import frc.robot.subsystems.ToteGrabber
//TeleOp Code - Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {
    override fun initialize() {
        addRequirements(Drivetrain, Intake, ToteGrabber)
    }
    override fun execute() {

        //===== DRIVETRAIN =====//
        var power = 1.0; //True max power

        // drivetrain controls
        if (OI.quickReverse) Drivetrain.tankDrive(OI.leftSideDrive * power * -1.0, OI.rightSideDrive * power * -1.0)
        else Drivetrain.tankDrive(OI.leftSideDrive * power, OI.rightSideDrive* power)

        //===== SUBSYSTEMS =====//
        if (OI.conveyorForward) {
            Intake.runConveyor(5.volts)
        } else if (OI.conveyorBackward){
            Intake.runConveyor(-5.volts)
        }

        Intake.runIntake((OI.intakeControl * 5.0).volts)

        if (OI.raiseIntake) Intake.raiseIntake()
        else if (OI.lowerIntake) Intake.lowerIntake()
        
        ToteGrabber.runToteGrab((5.0 * OI.toteGrabControl).volts)
    }

    // operator interface
    object OI {
        private val leftJoystick = Joystick(0) //These numbers correspond to the USB order in the Driver Station App
        private val rightJoystick = Joystick(1)
        private val controller = XboxController(2)

        // Allows you to tweak controller inputs (ie get rid of deadzone, make input more sensitive by squaring or cubing it, etc)
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

        // ROBOT CONTROL BINDINGS!

        //Drive
        val leftSideDrive get() = leftJoystick.y.processInput()
        val rightSideDrive get() = rightJoystick.y.processInput()
        val quickReverse get() = rightJoystick.triggerPressed

        //Subsystems
        val intakeControl get() = controller.leftY
        val conveyorForward get() = controller.leftTriggerAxis > 0.1
        val conveyorBackward get() = controller.leftBumper

        val toteGrabControl get() = controller.rightY

        val raiseIntake get() = controller.aButtonPressed
        val lowerIntake get() = controller.yButtonPressed
    }
}






































































































// uwu