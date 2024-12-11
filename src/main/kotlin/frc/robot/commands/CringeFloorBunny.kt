package frc.robot.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Commands

import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake

import beaverlib.utils.Units.Electrical.volts

val cfPower = 3.volts //3 volts, cringe tote drive power
val cfConveyorPower = 3.volts //3 volts, conveyor power.

object CringeFloorBunny : SequentialCommandGroup (
    Drivetrain.driveSeconds(-cfPower, -cfPower, 4.0), // Drive toward tote (forward)
    Commands.run({Intake.runConveyor(cfConveyorPower)}) //Deposit balloons/bunnies
)