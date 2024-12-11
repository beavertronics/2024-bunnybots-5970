package frc.robot.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Commands

import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake

import beaverlib.utils.Units.Electrical.volts
import beaverlib.utils.Units.Electrical.VoltageUnit

val ctPower = 3.volts //3 volts, cringe tote drive power
val ctConveyorPower = 3.volts //3 volts, conveyor power.

object CringeTote : SequentialCommandGroup (
    Drivetrain.driveSeconds(-ctPower, -ctPower, 3.0), // Drive toward tote (forward)
    Drivetrain.driveSeconds(0.volts,  -ctPower, 1.0), // Turn to face tote, hopefully clipping the edge if tote was placed in a bad location
    Drivetrain.driveSeconds(-ctPower, -ctPower, 2.0), // Move forward in case tote was further away
    Commands.run({Intake.runConveyor(ctConveyorPower)}) //Deposit balloons/bunnies
)