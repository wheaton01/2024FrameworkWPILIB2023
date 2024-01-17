// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.subsystemConstants;

public class shooterSubsytem extends SubsystemBase {
  /** Creates a new shooterSubsytem. */
  CANSparkMax tShooter,bShooter ;
  RelativeEncoder ebShooter,etShooter;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public double currentSetpoint,shooterTolerance;
  int topMotor, botMotor;
  SparkMaxPIDController tShooterPID,bShooterPID;
  MotorControllerGroup shootingMotors;
  
  public shooterSubsytem(int topMotor, int botMotor) {
    this.topMotor = topMotor;
    this.botMotor = botMotor;
    tShooter = new CANSparkMax(topMotor, MotorType.kBrushless);
    bShooter = new CANSparkMax(botMotor, MotorType.kBrushless);

    bShooter.setInverted(true);
    tShooter.setInverted(false);//TODO: MAY NEED CHANGES WITH PHYSICAL ROBOT
    
    etShooter = tShooter.getEncoder();
    ebShooter = bShooter.getEncoder();


    tShooterPID = tShooter.getPIDController();
    kP = 0.0004;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00017; // .000015
    kMaxOutput = 1;
    kMinOutput = 0;

    tShooterPID = tShooter.getPIDController();
    tShooterPID.setP(kP);
    tShooterPID.setI(kI);
    tShooterPID.setD(kD);
    tShooterPID.setIZone(kIz);
    tShooterPID.setFF(kFF);
    tShooterPID.setOutputRange(kMinOutput, kMaxOutput);     
    bShooterPID = bShooter.getPIDController();

    bShooterPID.setP(kP);
    bShooterPID.setI(kI);
    bShooterPID.setD(kD);
    bShooterPID.setIZone(kIz);
    bShooterPID.setFF(kFF);
    bShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    bShooterPID.setReference(currentSetpoint, ControlType.kVelocity);
    tShooterPID.setReference(currentSetpoint, ControlType.kVelocity);
    
  }
  
  public void setSpeed(double setpoint){
    bShooterPID.setReference(setpoint, ControlType.kVelocity);
    tShooterPID.setReference(setpoint, ControlType.kVelocity);
    currentSetpoint = setpoint;
  }


  public double getTopMeasurement(){
    return etShooter.getVelocity();
  }
  public double getBotMeasurement(){
    return ebShooter.getVelocity();
  }
  public boolean inTolerance(){
    if (subsystemConstants.kShooterPIDTolerance>Math.abs(currentSetpoint - etShooter.getVelocity())){
      return true;
    }   
    if (subsystemConstants.kShooterPIDTolerance<Math.abs(currentSetpoint - etShooter.getVelocity())){
      return false;
    }else{ return false;}
  }
  public void setZero(){
    bShooterPID.setReference(0, ControlType.kVelocity);
    tShooterPID.setReference(0, ControlType.kVelocity);
    currentSetpoint=0;
  }

}