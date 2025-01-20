// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2023 FRC season for team 4643, Butte Built Bots

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.cartesianMecanumDrive;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  String trajectoryJSON = "Output/Ball1.wpilib.json";
  private Sensors m_sensors = new Sensors();
  private Trajectory Auto1;

  private DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_sensors);
  private TelescopingSubsystem m_telescopingSubsystem = new TelescopingSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem(m_telescopingSubsystem);
  private WristSubsystem m_wristSubsystem = new WristSubsystem(m_armSubsystem);
  private MandibleSubsystem m_mandibleSubsystem = new MandibleSubsystem();

  private XboxController m_driveController = new XboxController(0);
  private XboxController m_operateController = new XboxController(1);

  public cartesianMecanumDrive m_cartesianMecanumDrive = new cartesianMecanumDrive(m_driveSubsystem, m_sensors,
      () -> -m_driveController.getRawAxis(XboxController.Axis.kLeftX.value),
      () -> m_driveController.getRawAxis(XboxController.Axis.kLeftY.value),
      () -> -m_driveController.getRawAxis(XboxController.Axis.kRightX.value));

  public Command unStow() {
    return new SequentialCommandGroup(new InstantCommand(m_armSubsystem::unStow1))
        .alongWith(new InstantCommand(m_wristSubsystem::holdForUnstow)).andThen(new WaitUntilCommand(
            m_armSubsystem::atSetpoint).andThen(m_wristSubsystem::unStow)
            .andThen(new WaitUntilCommand(m_wristSubsystem::atSetpoint).andThen(m_armSubsystem::unStow2)));

  }

  Trigger coneTrigger = new Trigger(() -> m_operateController.getLeftBumperPressed());

  Trigger cubeTrigger = new Trigger(() -> m_operateController.getRightBumperPressed());

  Trigger intakeTrigger = new Trigger (() -> (m_operateController.getLeftTriggerAxis() > 0.5));

  Trigger deployTrigger = new Trigger (() -> (m_operateController.getRightTriggerAxis() > 0.5));

  Trigger substationTrigger = new Trigger (() -> (m_operateController.getPOV() == 0));

  Trigger intakePositionTrigger = new Trigger (() -> ((m_operateController.getPOV() == 90) || (m_operateController.getPOV() == 270)));

  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Auto1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

  }

  private void configureButtonBindings() {
    // Operator controls

    new JoystickButton(m_operateController, XboxController.Button.kB.value).onTrue(
        new SequentialCommandGroup(
          new InstantCommand(m_telescopingSubsystem::matchStow),
          new WaitUntilCommand(m_telescopingSubsystem::atSetpoint),
          new InstantCommand(m_wristSubsystem::matchStow),
          new WaitUntilCommand(m_wristSubsystem::atSetpoint),
          new InstantCommand(m_armSubsystem::matchStow)));

    new JoystickButton(m_operateController, XboxController.Button.kA.value).and(coneTrigger)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(m_armSubsystem::lowCone),
            new WaitUntilCommand(m_armSubsystem::atSetpoint),
            new InstantCommand(m_wristSubsystem::lowCone),
            new WaitUntilCommand(m_wristSubsystem::atSetpoint),
            new InstantCommand(m_telescopingSubsystem::lowCone)));

    new JoystickButton(m_operateController, XboxController.Button.kX.value).and(coneTrigger)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(m_armSubsystem::midCone),
            new WaitUntilCommand(m_armSubsystem::atSetpoint),
            new InstantCommand(m_wristSubsystem::midCone),
            new WaitUntilCommand(m_wristSubsystem::atSetpoint),
            new InstantCommand(m_telescopingSubsystem::midCone)));

    new JoystickButton(m_operateController, XboxController.Button.kA.value).and(cubeTrigger)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(m_armSubsystem::lowCube),
            new WaitUntilCommand(m_armSubsystem::atSetpoint),
            new InstantCommand(m_wristSubsystem::lowCube),
            new WaitUntilCommand(m_wristSubsystem::atSetpoint),
            new InstantCommand(m_telescopingSubsystem::lowCube)));

    new JoystickButton(m_operateController, XboxController.Button.kX.value).and(cubeTrigger)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(m_armSubsystem::midCube),
            new WaitUntilCommand(m_armSubsystem::atSetpoint),
            new InstantCommand(m_wristSubsystem::midCube),
            new WaitUntilCommand(m_wristSubsystem::atSetpoint),
            new InstantCommand(m_telescopingSubsystem::midCube)));

    new JoystickButton(m_operateController, XboxController.Button.kY.value).and(cubeTrigger)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(m_armSubsystem::highCube),
            new WaitUntilCommand(m_armSubsystem::atSetpoint),
            new InstantCommand(m_wristSubsystem::highCube),
            new WaitUntilCommand(m_wristSubsystem::atSetpoint),
            new InstantCommand(m_telescopingSubsystem::highCube)));

    intakeTrigger.onTrue(new InstantCommand(m_mandibleSubsystem::intake)).onFalse(new InstantCommand(m_mandibleSubsystem::intakeHold));

    deployTrigger.onTrue(new InstantCommand(m_mandibleSubsystem::shootHalf)).onFalse(new InstantCommand(m_mandibleSubsystem::stopMotors)); 

    

    // Driver controls
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(m_cartesianMecanumDrive::backRotation)); // Back rotation on A button

    new JoystickButton(m_driveController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(m_cartesianMecanumDrive::rightRotation)); // Back rotation on A button

    new JoystickButton(m_driveController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(m_cartesianMecanumDrive::leftRotation)); // Back rotation on A button

    new JoystickButton(m_driveController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(m_cartesianMecanumDrive::frontRotation)); // Back rotation on A button

    new JoystickButton(m_driveController, XboxController.Button.kLeftStick.value)
        .onTrue(new InstantCommand(m_cartesianMecanumDrive::centeredRotation)); // Centered rotation on left stick press

    new JoystickButton(m_driveController, XboxController.Button.kRightStick.value)
        .onTrue(new InstantCommand(m_cartesianMecanumDrive::zeroHeading));

  }

  public RepeatCommand updateArmSmartDashValues = new RepeatCommand(
      new InstantCommand(m_armSubsystem::updateArmSmartDashValues).ignoringDisable(true));

  public RepeatCommand updateWristSmartDashValues = new RepeatCommand(
      new InstantCommand(m_wristSubsystem::updateWristSmartDashValues).ignoringDisable(true));

  public Command getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        autoConstants.kMaxSpeedMetersPerSecond,
        autoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(driveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the estimated current robot pose
        m_driveSubsystem.getPose(),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(.5, .5), new Translation2d(1, -.5)),
        // End exactly where we started
        m_driveSubsystem.getPose(),
        config);

    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
        exampleTrajectory,
        m_driveSubsystem::getPose,
        driveConstants.kDriveKinematics,
        autoConstants.kxPID,
        autoConstants.kyPID,
        autoConstants.kthetaPID,
        autoConstants.kMaxSpeedMetersPerSecond,
        m_driveSubsystem::setDriveWheelMetersPerSecond,
        m_driveSubsystem);

    return mecanumControllerCommand.andThen(m_driveSubsystem::stopMotors);
  }
}
