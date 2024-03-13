// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {
    private CANSparkFlex intakeMotor = new CANSparkFlex(14, MotorType.kBrushless);
    private CANSparkFlex intakeMotorFollower = new CANSparkFlex(22, MotorType.kBrushless);
    private CANSparkFlex transferMotor = new CANSparkFlex(15, MotorType.kBrushless);
    private RelativeEncoder transfer_encoder = transferMotor.getEncoder();

    private SparkPIDController transfer_pidController;



  private AnalogInput irIntakeSensor = new AnalogInput (2);
  private DigitalInput intakeStopSwitch = new DigitalInput(8);



  public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(60);

        intakeMotorFollower.restoreFactoryDefaults();
        intakeMotorFollower.follow(intakeMotor);
        
        transferMotor.restoreFactoryDefaults();
        transferMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        transferMotor.setInverted(true);
        transferMotor.setSmartCurrentLimit(60); 
     
        transfer_pidController = transferMotor.getPIDController();
      
      
        // set PID coefficients transfer encoder
      transfer_pidController.setP(1);
      transfer_pidController.setI(0);
      transfer_pidController.setD(0);
      transfer_pidController.setIZone(0);
      transfer_pidController.setFF(0);
      transfer_pidController.setOutputRange(-0.2, 0.2);
     
      }

 
  public void transferMotor (double speed){
    transferMotor.set(speed);
  }

   public void transferEncoderGet (){
    transfer_encoder.getPosition();
  }

  public void transferEncoderRotations (double rotation){
    //transfer_encoder.setZeroOffset(distance);
    //transfer_encoder.setInverted(true);
    transfer_pidController.setReference((transfer_encoder.getPosition() - rotation),CANSparkFlex.ControlType.kPosition);
  }

  public void floorIntakeSpeed(double floorSpeed) {
    intakeMotor.set(floorSpeed);
       
  }

  public void IntakeMotorSpeed(double floorSpeed,double transferSpeed) {
    intakeMotor.set(floorSpeed);
    transferMotor.set(transferSpeed);
    
  }

  public int get_irIntakeSensor () {
      return (int) irIntakeSensor.getVoltage();
  }

  
  public boolean get_intakeStopSwitch () {
    return intakeStopSwitch.get();
  }



  @Override
  public void periodic() {

    /* Stops intaking motors if note in Hopper 
    //greater the voltage the closer the note >1.5
    
      if (get_irIntakeSensor() <1.5 ) {
        
        intakeMotor.stopMotor();
        transferMotor.set(0);
      }
      */

      SmartDashboard.putNumber("irIntake", get_irIntakeSensor());
      SmartDashboard.putBoolean("intake switch", get_intakeStopSwitch());

      

}





}
