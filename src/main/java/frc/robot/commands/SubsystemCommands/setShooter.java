// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterSubsytem;

public class setShooter extends CommandBase {
  /** Creates a new setShooter. */
  shooterSubsytem sShooter;
  double setpoint;
  boolean turnOff;
  public setShooter(shooterSubsytem sShooter, double setpoint, boolean turnOff, boolean useLimelight) {
    this.sShooter = sShooter;
    this.setpoint = setpoint;
    this.turnOff  = turnOff;


    addRequirements(sShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sShooter.setSpeed(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (turnOff){
      sShooter.setSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
