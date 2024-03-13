package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandFactory {
    
    private final ArmSubsystem armSubsystem;
    private final ClimbSubsystem climbSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public CommandFactory(
            ArmSubsystem armSubsystem,
            ClimbSubsystem climbSubsystem,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem

    ) {
            this.armSubsystem = armSubsystem;
            this.climbSubsystem = climbSubsystem;
            this.intakeSubsystem = intakeSubsystem;
            this.shooterSubsystem = shooterSubsystem;
    }


    public Command ShootAuto(IntakeSubsystem intakeSubsystem2, ShooterSubsystem shooterSubsystem2) {
      
          return Commands.sequence(
            new RunCommand (()->shooterSubsystem.ShooterMotorSpeed (1,1)).withTimeout(4),
            new RunCommand (()->intakeSubsystem.IntakeMotorSpeed(0,-.1)).withTimeout(.25),
            new RunCommand(()-> intakeSubsystem.IntakeMotorSpeed(0, 1)).withTimeout(1),
            new RunCommand (()-> intakeSubsystem.IntakeMotorSpeed(0, 0)).withTimeout(1),
            new RunCommand (()-> shooterSubsystem.ShooterMotorSpeed(0, 0))
        );
    
    
   }

}
