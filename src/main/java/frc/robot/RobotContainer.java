// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;

import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.dConstants;
import frc.robot.commands.cartesianMecanumDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  String trajectoryJSON = "Output/Ball1.wpilib.json";
  private Sensors m_sensors = new Sensors();
  private Trajectory Auto1;
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_sensors);
  private XboxController m_driveController = new XboxController(0);
  private XboxController m_operateController = new XboxController(1);

  private cartesianMecanumDrive m_cartesianMecanumDrive = new cartesianMecanumDrive(m_driveSubsystem, m_sensors,
    () -> -m_driveController.getRawAxis(XboxController.Axis.kLeftX.value), 
      () -> m_driveController.getRawAxis(XboxController.Axis.kLeftY.value), 
        () -> -m_driveController.getRawAxis(XboxController.Axis.kRightX.value));

  public RobotContainer() {

    // The first argument is the root container
    // The second argument is whether logging and config should be given separate tabs
    Logger.configureLoggingAndConfig(this, false);

    // Configure the button bindings
    configureButtonBindings();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Auto1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    m_driveSubsystem.setDefaultCommand(m_cartesianMecanumDrive);

  }

  private void configureButtonBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(m_cartesianMecanumDrive::backRotation)); //Back rotation on A button

    new JoystickButton(m_driveController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(m_cartesianMecanumDrive::rightRotation)); //Back rotation on A button
    
    new JoystickButton(m_driveController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(m_cartesianMecanumDrive::leftRotation)); //Back rotation on A button
    
    new JoystickButton(m_driveController, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(m_cartesianMecanumDrive::frontRotation)); //Back rotation on A button
    
    new JoystickButton(m_driveController, XboxController.Button.kLeftStick.value)
      .onTrue(new InstantCommand(m_cartesianMecanumDrive::centeredRotation)); //Centered rotation on left stick press
  }

  public Command getAutonomousCommand() {

        // Create config for trajectory
        TrajectoryConfig config =
        new TrajectoryConfig(
                autoConstants.kMaxSpeedMetersPerSecond,
                autoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_driveSubsystem.m_kinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            m_driveSubsystem.getPose(),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

  MecanumControllerCommand mecanumControllerCommand =
  new MecanumControllerCommand(
      exampleTrajectory,
      m_driveSubsystem::getPose,
      autoConstants.kFeedforward,
      m_driveSubsystem.m_kinematics,

      // Position contollers
      new PIDController(autoConstants.kPXController, autoConstants.kIXController, autoConstants.kDXController),
      new PIDController(autoConstants.kPYController, autoConstants.kIYController, autoConstants.kDYController),
      new ProfiledPIDController(
          autoConstants.kPThetaController, autoConstants.kIThetaController, autoConstants.kPThetaController, autoConstants.kThetaControllerConstraints),

      // Needed for normalizing wheel speeds
      autoConstants.kMaxSpeedMetersPerSecond,

      // Velocity PID's
      new PIDController(autoConstants.kPwheelVel, autoConstants.kIwheelVel, autoConstants.kDwheelVel),
      new PIDController(autoConstants.kPwheelVel, autoConstants.kIwheelVel, autoConstants.kDwheelVel),
      new PIDController(autoConstants.kPwheelVel, autoConstants.kIwheelVel, autoConstants.kDwheelVel),
      new PIDController(autoConstants.kPwheelVel, autoConstants.kIwheelVel, autoConstants.kDwheelVel),
      m_driveSubsystem::getCurrentWheelSpeeds,
      m_driveSubsystem::setDriveMotorControllersVolts, // Consumer for the output motor voltages
      m_driveSubsystem);
      System.out.println("Path following started!");
      return mecanumControllerCommand.andThen(() -> m_driveSubsystem.stopMotors());

  }
}


