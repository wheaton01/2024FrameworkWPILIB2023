// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  CANSparkMax armMotor;
  double setpoint;
  int motorID,encoderPort1,encoderPort2;
  Encoder angEncoder;
  SparkMaxPIDController mArmPID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


  public armSubsystem(int motorID, int encoderPort1, int encoderPort2) {
    this.motorID = motorID;
    this.encoderPort1 = encoderPort1;
    this.encoderPort2 = encoderPort2;

    angEncoder = new Encoder(encoderPort1,encoderPort2);
    armMotor = new CANSparkMax(motorID, MotorType.kBrushless);

    mArmPID = armMotor.getPIDController();
    angEncoder.setDistancePerPulse(360.0/2048.0);//will return 360 units for every 2048 pulses
    kP = 0.0004;//TODO: TUNE PID HERE
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00017; // .000015
    kMaxOutput = 1;
    kMinOutput = 0;

    mArmPID.setP(kP);
    mArmPID.setI(kI);
    mArmPID.setD(kD);
    mArmPID.setIZone(kIz);
    mArmPID.setFF(kFF);
    mArmPID.setOutputRange(kMinOutput, kMaxOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPose(double Position){
    mArmPID.setReference(Position, ControlType.kPosition);
    System.out.println("arm Setpoint is : "+Position);
  }
}
