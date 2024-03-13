// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RiPo extends SequentialCommandGroup {
  /** Creates a new RedSide. */
  public RiPo(IntakeSubsystem intakeSubsystem, SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
   addRequirements(armSubsystem,shooterSubsystem,swerveSubsystem,intakeSubsystem); // Add your commands in the addCommands() call, e.g.
  
   addCommands(
//new RunCommand(()-> swerveSubsystem.drive(new Translation2d(1,0), 0,true)).withTimeout(3.2));
// new RunCommand( ()-> shooterSubsystem.ShooterMotorSpeed(-.1, .1)), // dont want

new InstantCommand(()-> armSubsystem.midShotPosition()).withTimeout(1),
new RunCommand(()-> shooterSubsystem.ShooterMotorSpeed(1, 1)).withTimeout(1),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0,1)).withTimeout(.1),
new WaitCommand (1),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0,0)).withTimeout(1),
new InstantCommand(()-> armSubsystem.stowedArm()).withTimeout(.5),
new RunCommand(()-> shooterSubsystem.ShooterMotorSpeed(0,0)).withTimeout(.1),
//new RunCommand(()-> swerveSubsystem.drive(new Translation2d(1,0), 0,true)).withTimeout(3.2)); //needed
new WaitCommand (1),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(1,.25)).withTimeout(2),
new WaitCommand (1),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0,-.15)).withTimeout(.25),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0,0)).withTimeout(.1),
new InstantCommand(()-> armSubsystem.midShotPosition()).withTimeout(.5),
new RunCommand(()-> shooterSubsystem.ShooterMotorSpeed(1,1)).withTimeout(1),
new WaitCommand(1),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(1,1)).withTimeout(.5),
new InstantCommand(()-> armSubsystem.stowedArm()).withTimeout(.5),
new RunCommand(()-> shooterSubsystem.ShooterMotorSpeed(0,0)).withTimeout(5),
new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0,0)) 
    );
  }
}
