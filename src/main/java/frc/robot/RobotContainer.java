// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.RiPo;
import frc.robot.commands.swervedrive.auto.IntakeAuto;
import frc.robot.commands.swervedrive.auto.ShootAutoCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

 /*SWERVE*/
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/vortex"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);


  /* Controllers */
  final CommandXboxController driver = new CommandXboxController (0);
  final CommandXboxController operator = new CommandXboxController (1);
  final CommandXboxController testing = new CommandXboxController (2);

 /* Subsystems */
  final ArmSubsystem armSubsystem = new ArmSubsystem();
  final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  /* Command Factory */
   final CommandFactory commandFactory;
  
  /*Auto Selector */
 private final SendableChooser<Command> autoChooser;


 
 public RobotContainer()
  {
    autoChooser = AutoBuilder.buildAutoChooser("MidSpeakerAutp");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    configureBindings(); // Configure the trigger bindings
    configureAutoSelector();

commandFactory = new CommandFactory(armSubsystem, climbSubsystem, intakeSubsystem, shooterSubsystem);

  // Register Named Commands
  //NamedCommands.registerCommand("shooter motor", shooterSubsystem.ShooterMotorSpeed(1,1));
  NamedCommands.registerCommand("shooterWheelsGo", new RunCommand(()->
        shooterSubsystem.ShooterMotorSpeed(1,1)));
        


  NamedCommands.registerCommand("shoot", commandFactory.ShootAuto(intakeSubsystem, shooterSubsystem));
  NamedCommands.registerCommand("intake", new IntakeAuto(intakeSubsystem));
  //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());   

  /*SWERVE */
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY()*-1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX()*-1, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX()*-1,
        () -> driverXbox.getRightY()*-1);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot


    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
        


    }
     private void configureAutoSelector() {

     // autoChooser.addOption("MidSpeakerAuto", new MidSpeakerAuto(drivebase));
     autoChooser.addOption("RiPo", new RiPo(intakeSubsystem, drivebase, armSubsystem, shooterSubsystem));
      autoChooser.addOption("ShootAuto", new ShootAutoCommands(intakeSubsystem, shooterSubsystem));
      SmartDashboard.putData("Auto Selector", autoChooser);
  

    
    
  }
    

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
 
  {
    
    /*SWERVE*/
 
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox,
                       2).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));




/*......................DRIVER CONTROLLER.............................. */
    
/*intake forward*/
  driver.rightBumper().whileTrue(new InstantCommand(() -> intakeSubsystem.IntakeMotorSpeed(1,1)));
  driver.rightBumper().whileFalse(new InstantCommand(() -> intakeSubsystem.IntakeMotorSpeed(0,0)));                             
  //driver.rightBumper().whileFalse(new SequentialCommandGroup(new InstantCommand(() -> intakeSubsystem.transferMotor(-1),
  //                                                            new WaitCommand(1)),
  //                                                            intakeSubsystem.IntakeMotorSpeed(0, 0)));


//intakeSubsystem.transferEncoderRotations(1000)
                                                        

         
/*intake reverse*/
  driver.rightTrigger().whileTrue(new StartEndCommand(() -> intakeSubsystem.IntakeMotorSpeed(-1,-1), 
                                                         () -> intakeSubsystem.IntakeMotorSpeed(0,0)));
/*Arm Brake*/
 driver.povUp().onTrue(new InstantCommand(()-> {armSubsystem.Servobrake();}));
  driver.povDown().onTrue(new InstantCommand(()-> {armSubsystem.Servoloose();}));

/*......................OPERATOR CONTROLLER.............................. */

  /*SHOOT */
    operator.povDown().whileTrue(new SequentialCommandGroup(
      new InstantCommand(()->{
        shooterSubsystem.ShooterMotorSpeed(1,1);
        intakeSubsystem.IntakeMotorSpeed(0,-.1);
      }),
        new WaitCommand(.5),
        new InstantCommand(() -> {
        intakeSubsystem.IntakeMotorSpeed(0,1);
        shooterSubsystem.ShooterMotorSpeed(1,1);
      })))
       .whileFalse(new SequentialCommandGroup(new InstantCommand(() -> {
       armSubsystem.stowedArm();
       intakeSubsystem.IntakeMotorSpeed(0,0);
       shooterSubsystem.ShooterMotorSpeed(0,0);
      })));

/*STOWED SHOT*/
operator.povUp().onTrue(new InstantCommand(() -> {armSubsystem.stowedArm(); shooterSubsystem.ShooterMotorSpeed(0,0);}));
    
/*MID SHOT(PODIUM)*/
operator.a().onTrue(new InstantCommand(() -> {armSubsystem.midShotPosition(); shooterSubsystem.ShooterMotorSpeed(0,0);}));

/*FAR SHOT*/
operator.x().onTrue(new InstantCommand(() -> {armSubsystem.farShotPosition(); shooterSubsystem.ShooterMotorSpeed(0,0);}));

/*AMP SHOT*/
operator.y().onTrue(new InstantCommand(() -> {armSubsystem.ampShotPosition(); shooterSubsystem.ShooterMotorSpeed(0,0);}));

/*STAGE SHOT(mid stage)*/
 operator.b().onTrue(new InstantCommand(() -> {armSubsystem.stageShotPosition(); shooterSubsystem.ShooterMotorSpeed(0,0);}));

 /*Arm Manual */
/*operator.povRight().whileTrue(new StartEndCommand(()-> armSubsystem.ArmMotorSpeed(1),
                              ()-> armSubsystem.ArmMotorSpeed(0)));
operator.povLeft().whileTrue(new StartEndCommand(()-> armSubsystem.ArmMotorSpeed(-1),
                            ()-> armSubsystem.ArmMotorSpeed(0)));*/
/*Arm Manual + Auto Brake */
operator.povRight().whileTrue(new StartEndCommand(
    () -> {
        armSubsystem.ArmMotorSpeed(1);
        armSubsystem.Servoloose();
    },
    () -> {
        armSubsystem.ArmMotorSpeed(0);
        armSubsystem.Servobrake();
    }
));
operator.povLeft().whileTrue(new StartEndCommand(
    () -> {
        armSubsystem.ArmMotorSpeed(-1);
        armSubsystem.Servoloose();
    },
    () -> {
        armSubsystem.ArmMotorSpeed(0);
        armSubsystem.Servobrake();
    }
));
/*CLIMB*/

    operator.leftBumper().whileTrue(new StartEndCommand(()-> climbSubsystem.climbMotorA_Speed(.8),
                                                        ()-> climbSubsystem.climbMotorA_Speed(0)));   
    operator.leftTrigger().whileTrue(new StartEndCommand(()-> climbSubsystem.climbMotorA_Speed(-.8),
                                                        ()-> climbSubsystem.climbMotorA_Speed(0)));  

    operator.rightBumper().whileTrue(new StartEndCommand(()-> climbSubsystem.climbMotorB_Speed(.8),
                                                        ()-> climbSubsystem.climbMotorB_Speed(0)));   
    operator.rightTrigger().whileTrue(new StartEndCommand(()-> climbSubsystem.climbMotorB_Speed(-.8),
                                                        ()-> climbSubsystem.climbMotorB_Speed(0))); 


/*INCREMENT ARM- future*/



/*......................TESTING CONTROLLER.............................. */

//ARM

//INTAKE


//SHOOTER


//CLIMB

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    

  
  //    drivebase.resetOdometry(new Pose2d(0,0, new Rotation2d()));
  //    drivebase.drive(new Translation2d(), 0, true); // Setup kinematics correctly. Don't know why its needed yet...
  //    return drivebase.driveAuto(new Translation2d(5, 0), new Rotation2d()).withTimeout(1);//drivebase.getAutonomousCommand("New Auto");


    
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}


