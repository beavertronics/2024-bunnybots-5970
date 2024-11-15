package frc.robot.commands

import beaverlib.utils.Sugar.within
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Command

import kotlin.math.*

import frc.robot.subsystems.Drivetrain
//import frc.robot.subsystems.Intake
//import frc.robot.subsystems.Shooter


//TeleOp Code- Controls the robot based off of inputs from the humans operating the Driver Station.

object TeleOp : Command() {

    //var shooting = false
    //val shootTimer = Timer()

    override fun initialize() {
        addRequirements(Drivetrain/*,Intake,Shooter*/)
    }

    override fun execute() {
    }

    object OI {
        private val driverController    = XboxController(0)
        private val operatorController  = Joystick(1)

        private fun Double.processInput(deadzone : Double = 0.1, squared : Boolean = false, cubed : Boolean = false, readjust : Boolean = true) : Double{
            var processed = this
            if(readjust) processed = ((this.absoluteValue - deadzone)/(1 - deadzone))*this.sign
            return when {
                this.within(deadzone) ->    0.0
                squared ->                  processed.pow(2) * this.sign
                cubed ->                    processed.pow(3)
                else ->                     processed
            }
        }
        private fun Double.abs_GreaterThan(target: Double): Boolean{
            return this.absoluteValue > target
        }
    }
}