package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand

import frc.robot.commands.TeleOp
import frc.robot.subsystems.Drivetrain

/* Main code for controlling the robot. Mainly just links everything together.

    The hardware of the robot (what motor controllers, etc) is defined in RobotHardware.kt

    Driver control is defined in TeleOp.kt

*/

object RobotController : TimedRobot() {

    val autos: Map<String,Command> = mapOf(
        //TODO: Autos go here!
        //ie 
        //"Description of auto" to TaxiAuto
    )    
    override fun robotInit() {
        //Initialize the robot!
        CameraServer.startAutomaticCapture() //TODO: Can we offload camera streaming to a Raspberry Pi? Should we?

    }
    override fun robotPeriodic() {
        //Runs while the robot is on, regarless of whether it is enabled.
        // (use for telemetry, command scheduler)
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
    }
    override fun autonomousPeriodic() {} //TODO: Unnecesary with command-based programming?

    override fun teleopInit() {
        TeleOp.schedule()
    }
    override fun teleopPeriodic() {} //TODO: Unnecessary with command-based programming?

    override fun simulationInit() {} //Runs only in simulation mode (other functions run regardless of whether robot is simulated or not)

    override fun disabledInit() {
        //Runs as soon as the robot is disabled, use to deactivate motors safely, etc
    }
    override fun disabledPeriodic() {
        //Runs only while robot is disabled- Use to hold motors in position for safety reasons.
        // Try to avoid putting code here- often unsafe.
    }

    override fun testInit() {}
    override fun testPeriodic() {}
}