// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.aprilTagSwerve;
import frc.robot.commands.testSequence;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.swervedrive.drivebase.zeroGyroCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.parser.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //SwerveDrive swerveDrive= new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve/neo")).createSwerveDrive(Units.feetToMeters(14.5));
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  XboxController driverXbox = new XboxController(OperatorConstants.kDriverPort);

  TeleopDrive teleopDrive = new TeleopDrive(drivebase, 
  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
  OperatorConstants.LEFT_Y_DEADBAND),
() -> MathUtil.applyDeadband(driverXbox.getLeftX(),
    OperatorConstants.LEFT_X_DEADBAND),
() -> MathUtil.applyDeadband(driverXbox.getRightX(),
    OperatorConstants.RIGHT_X_DEADBAND),
    () -> driverXbox.getRightBumper());



    AbsoluteDrive absDrive = new AbsoluteDrive(drivebase,  
  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
      OperatorConstants.LEFT_Y_DEADBAND),
  () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
      OperatorConstants.LEFT_X_DEADBAND),
  () -> MathUtil.applyDeadband(driverXbox.getRightX(),
     OperatorConstants.RIGHT_X_DEADBAND),  
  () -> MathUtil.applyDeadband(driverXbox.getRightY(),
      OperatorConstants.RIGHT_Y_DEADBAND));



AbsoluteFieldDrive teleopField = new AbsoluteFieldDrive(drivebase,   
() -> MathUtil.applyDeadband(driverXbox.getLeftY(),
OperatorConstants.LEFT_Y_DEADBAND),
() -> MathUtil.applyDeadband(driverXbox.getLeftX(),
  OperatorConstants.LEFT_X_DEADBAND),
() -> MathUtil.applyDeadband(driverXbox.getRightX(),
  OperatorConstants.RIGHT_X_DEADBAND) );




  //LIMELIGHT DRIVE
  aprilTagSwerve limelightSwerve = new aprilTagSwerve(drivebase, 
  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
  OperatorConstants.LEFT_Y_DEADBAND),
  () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
    OperatorConstants.LEFT_X_DEADBAND),
  () -> MathUtil.applyDeadband(driverXbox.getRightX(),
    OperatorConstants.RIGHT_X_DEADBAND),
    () -> driverXbox.getRightBumper());




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_driverController.leftBumper().whileTrue(new aprilTagSwerve(drivebase, 
    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
    OperatorConstants.LEFT_Y_DEADBAND),
  () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
      OperatorConstants.LEFT_X_DEADBAND),
  () -> MathUtil.applyDeadband(driverXbox.getRightX(),
      OperatorConstants.RIGHT_X_DEADBAND),
      () -> driverXbox.getRightBumper()));
    m_driverController.a().onTrue(new zeroGyroCommand(drivebase));
    
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Trajectory PPtraj= loadPPtraj("generatedJSON");


    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.Auton.MAX_SPEED,
            Constants.Auton.MAX_ACCELERATION)
                    .setKinematics(Constants.kDriveKinematics);

    

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Auton.kPXController, 0, 0);
    PIDController yController = new PIDController(Auton.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            Auton.kPThetaController, 0, 0, Auton.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            Constants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            drivebase::setModuleStates,
            drivebase);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
            new InstantCommand(() -> drivebase.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> drivebase.stopModules()));
}

  public void setDriveMode()
  {
   drivebase.setDefaultCommand(absDrive);
    //drivebase.setDefaultCommand();
  }

  public Trajectory loadPPtraj(String Filename){
    Trajectory PPtrajectory = new Trajectory();
    try{
      Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(Filename);
      PPtrajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
      System.out.println("PATH NAME : "+ Filename + " HAS BEEN LOADED");

    }catch(IOException exception){
      System.out.println("COULD NOT FIND PATH!! "+ Filename );
    }
  
  return PPtrajectory;
}



}
