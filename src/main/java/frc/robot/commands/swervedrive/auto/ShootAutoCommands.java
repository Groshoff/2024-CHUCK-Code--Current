package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAutoCommands extends SequentialCommandGroup {
    public ShootAutoCommands(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem ) {
        addRequirements(shooterSubsystem,intakeSubsystem);
          addCommands(
            new RunCommand (()->shooterSubsystem.ShooterMotorSpeed (1,1)).withTimeout(4),
            new RunCommand (()->intakeSubsystem.IntakeMotorSpeed(0,-.1)).withTimeout(.25),
            new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0, 1)).withTimeout(1),
            new RunCommand (()-> intakeSubsystem.IntakeMotorSpeed(0, 0)).withTimeout(1),
            new RunCommand (()-> shooterSubsystem.ShooterMotorSpeed(0, 0)).withTimeout(.5)
        );
    }
}