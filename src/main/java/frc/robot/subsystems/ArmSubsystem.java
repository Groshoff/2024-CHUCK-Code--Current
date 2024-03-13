// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkFlex ArmMotor = new CANSparkFlex(18, MotorType.kBrushless);
  private CANSparkFlex ArmMotorFollower = new CANSparkFlex(19, MotorType.kBrushless);

  Servo brakeServo = new Servo(0);
  private SparkAbsoluteEncoder Arm_encoder = ArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkPIDController Arm_pidController;
  private DigitalInput lowerHardLimSwitch = new DigitalInput(9);
 
    // PID coefficients............................................
    double kP = 16;//
    double kI = 0;
    double kD = 0;
    double kF = 3;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1; //set speed limits if needed
    double kMinOutput = -1;  //set speed limits if needed
 
 
 //PID Setpoint....................................................
    double stowedArm = 0; //SET BACK TO ZERO!!!!!!
    double midShotPosition = 15.5; //tune
    double farShotPosition = 19.75; //tune
    double ampShotPosition = 45.0; //tune
    double stageShotPosition = 17; //tune


  //Servo Angle....................................................
    double brakeAngle = 180;
    double looseAngle = 90;


    public ArmSubsystem() {
     
      //motors...........
      ArmMotor.restoreFactoryDefaults();
      ArmMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
      ArmMotor.setInverted(true);
      ArmMotor.setSmartCurrentLimit(60);

      ArmMotorFollower.restoreFactoryDefaults();
      ArmMotorFollower.setIdleMode(CANSparkFlex.IdleMode.kCoast);
      ArmMotorFollower.setInverted(false);
      ArmMotorFollower.setSmartCurrentLimit(60);
               
      Arm_pidController = ArmMotor.getPIDController();
     
      
        // set PID coefficients
      Arm_pidController.setP(kP);
      Arm_pidController.setI(kI);
      Arm_pidController.setD(kD);
      Arm_pidController.setIZone(kIz);
      Arm_pidController.setFF(kFF);
      Arm_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
  }


public void stowedArm() {
  Arm_pidController.setReference(stowedArm, CANSparkFlex.ControlType.kPosition);
  }

public void midShotPosition () {
  Arm_pidController.setReference(midShotPosition, CANSparkFlex.ControlType.kPosition);
  }

public void farShotPosition () {
  Arm_pidController.setReference(farShotPosition, CANSparkFlex.ControlType.kPosition);
  }

public void ampShotPosition () {
  Arm_pidController.setReference(ampShotPosition, CANSparkFlex.ControlType.kPosition);
  }
 
  public void stageShotPosition () {
  Arm_pidController.setReference(stageShotPosition, CANSparkFlex.ControlType.kPosition);
  }
 
public void ArmMotorSpeed(double speed) {
    ArmMotor.set(speed);
    ArmMotorFollower.set(speed);
  }
  

public void Servobrake() {
    brakeServo.setAngle(brakeAngle);
}
public void Servoloose(){
      brakeServo.setAngle(looseAngle);
}
public double getArmPosition() {
    return Arm_encoder.getPosition();
  }


public boolean getLowerLimSwitch() {
    return !lowerHardLimSwitch.get();
  }



  @Override
  public void periodic() {


    SmartDashboard.putNumber("Arm Position", getArmPosition());

 
      /* Stops motor and resets encoder after limit switch reached */
      if (getLowerLimSwitch()) {
        ArmMotor.stopMotor();
        Arm_encoder.setZeroOffset(0);
    
       
      }

}
}

