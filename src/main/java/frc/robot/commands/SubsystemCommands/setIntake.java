// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeSubsystem;

public class setIntake extends CommandBase {
  /** Creates a new setIntake. */
  intakeSubsystem sIntake;
  double setpoint;
  boolean useNoteSensor,noteSensorVal;
  public setIntake(intakeSubsystem sIntake, double setpoint,boolean useNoteSensor) {
    this.sIntake = sIntake;
    this.setpoint = setpoint;
    this.useNoteSensor = useNoteSensor;



    addRequirements(sIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sIntake.setSpeed(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(useNoteSensor){
      sIntake.setSpeed(0);
      System.out.println("NOTE SENSOR ON, INTAKE OFF");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (useNoteSensor){
      return sIntake.getNoteSensor();
    }else return false;
  }
}