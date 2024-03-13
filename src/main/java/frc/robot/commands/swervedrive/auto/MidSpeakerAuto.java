// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MidSpeakerAuto extends SequentialCommandGroup {
  /** Creates a new MidSpeakerAuto. */
  public MidSpeakerAuto(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    
  
  //  new RunCommand(() -> 
  //  swerveSubsystem.driveAuto(new Translation2d (1, 0), new Rotation2d(0)).withTimeout(1));



new RunCommand(() -> 

swerveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d())));
    swerveSubsystem.drive(new Translation2d(), 0, true); // Setup kinematics correctly. Don't know why its needed yet...
    swerveSubsystem.driveAuto(new Translation2d(5, 0), new Rotation2d()).withTimeout(1);//drivebase.getAutonomousCommand("New Auto")




  }
}




//drive(Translation2d translation, double rotation, boolean fieldRelative)

//.withTimeout(3.2));