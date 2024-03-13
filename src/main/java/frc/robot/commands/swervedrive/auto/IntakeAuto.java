// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAuto extends SequentialCommandGroup {
  public IntakeAuto(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
      addCommands(
        new RunCommand (()-> intakeSubsystem.IntakeMotorSpeed(1, 1))
        
      );

  }
  
}
