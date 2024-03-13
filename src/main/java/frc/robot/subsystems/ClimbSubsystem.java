// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkFlex climbMotorA = new CANSparkFlex(20, MotorType.kBrushless);
  private CANSparkFlex climbMotorB = new CANSparkFlex(21, MotorType.kBrushless);

  public ClimbSubsystem() {
    climbMotorA.restoreFactoryDefaults();
    climbMotorA.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    climbMotorA.setInverted(false);
    climbMotorA.setSmartCurrentLimit(60);
    climbMotorB.restoreFactoryDefaults();
    climbMotorB.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    climbMotorB.setInverted(true);
    climbMotorB.setSmartCurrentLimit(60);
   
  }

  public void climbMotorA_Speed(double Speed) {
    climbMotorA.set(Speed);
  }
  
  public void climbMotorB_Speed(double speed)  {
    climbMotorB.set(speed);
  }


   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


}
}



