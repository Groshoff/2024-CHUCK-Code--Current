// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkFlex ShooterMotorLower = new CANSparkFlex(17, MotorType.kBrushless);
  private CANSparkFlex ShooterMotorUpper = new CANSparkFlex(16, MotorType.kBrushless);
  private RelativeEncoder ShooterMotorUpper_encoder;
  private RelativeEncoder ShooterMotorLower_encoder;
  private SparkPIDController ShooterMotorLower_pidController;
  private SparkPIDController ShooterMotorUpper_pidController;
 

    // PID coefficients............................................
    double kP = 6e-5;
    double kI = 0;
    double kD = 0;
    double kF = 0;
    double kIz = 0;
    double kFF = 0.000015;
    double kMaxOutput = 1; //set speed limits if needed
    double kMinOutput = -1;  //set speed limits if needed
    double Maxrpm = 10000;
    double upper = 9500;
    double lower = 9500;


  public ShooterSubsystem() {
        ShooterMotorUpper.restoreFactoryDefaults();
        ShooterMotorUpper.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        ShooterMotorUpper.setInverted(true);  
        ShooterMotorUpper.setSmartCurrentLimit(60);
        
        ShooterMotorLower.restoreFactoryDefaults();
        ShooterMotorLower.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        ShooterMotorLower.setInverted(true);
        ShooterMotorLower.setSmartCurrentLimit(60);
   
        ShooterMotorUpper_encoder = ShooterMotorUpper.getEncoder();
        ShooterMotorLower_encoder = ShooterMotorUpper.getEncoder();
         
        //Upper Motor
        ShooterMotorUpper_pidController = ShooterMotorUpper.getPIDController();
        ShooterMotorUpper_pidController.setP(kP);
        ShooterMotorUpper_pidController.setI(kI);
        ShooterMotorUpper_pidController.setD(kD);
        ShooterMotorUpper_pidController.setIZone(kIz);
        ShooterMotorUpper_pidController.setFF(kFF);
        ShooterMotorUpper_pidController.setOutputRange(kMinOutput, kMaxOutput);
        SmartDashboard.putNumber("ProcessVariable", ShooterMotorUpper_encoder.getVelocity());


        //Lower Motor
        ShooterMotorLower_pidController = ShooterMotorLower.getPIDController();
        ShooterMotorLower_pidController.setP(kP);
        ShooterMotorLower_pidController.setI(kI);
        ShooterMotorLower_pidController.setD(kD);
        ShooterMotorLower_pidController.setIZone(kIz);
        ShooterMotorLower_pidController.setFF(kFF);
        ShooterMotorLower_pidController.setOutputRange(kMinOutput, kMaxOutput);
       SmartDashboard.putNumber("ProcessVariable", ShooterMotorLower_encoder.getVelocity());

  }

  public void ShooterMotorSpeed(double UpperSpeed,double LowerSpeed) {
    ShooterMotorLower.set(LowerSpeed);
    ShooterMotorUpper.set(UpperSpeed);
  }
   public void ShooterMotorPID(){
    ShooterMotorLower_pidController.setReference(lower,CANSparkFlex.ControlType.kVelocity);
    ShooterMotorUpper_pidController.setReference(upper,CANSparkFlex.ControlType.kVelocity);
  }
 



}
