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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  String trajectoryJSON = "Output/Ball1.wpilib.json";
  private Sensors m_sensors = new Sensors();
  private Trajectory Auto1;
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
  private IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private ShooterPIDSubsystem m_shooterPIDsubsystem = new ShooterPIDSubsystem();
  private HoodPIDSubsystem m_hoodPIDsubsystem = new HoodPIDSubsystem();
  private VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private TurretPIDSubsystem m_turretPIDsubsystem = new TurretPIDSubsystem(m_visionSubsystem, m_sensors);
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_sensors);
  private XboxController m_driveController = new XboxController(0);
  private XboxController m_operateController = new XboxController(1);
  private ClimberSubsystem m_climbersubsystem = new ClimberSubsystem();
  private LookForTarget m_lookfortarget = new LookForTarget(m_turretPIDsubsystem);
  private TrackTarget m_tracktarget = new TrackTarget(m_turretPIDsubsystem);
  private TurretPosition m_turretposition = new TurretPosition(m_turretPIDsubsystem, TurretConstants.HUBposition);

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


    // climber up
    new JoystickButton(m_driveController, 6).toggleOnTrue(new InstantCommand(m_climbersubsystem::up))
        .toggleOnFalse(new InstantCommand(m_climbersubsystem::idle));

    // climber down
    new JoystickButton(m_driveController, 5).toggleOnTrue(new InstantCommand(m_climbersubsystem::down))
        .toggleOnFalse(new InstantCommand(m_climbersubsystem::idle));

    // Front intake
    new JoystickButton(m_operateController, 4)
        .toggleOnTrue(new InstantCommand(m_pneumaticsSubsystem::frontIntakeOpen))
        .toggleOnFalse(new InstantCommand(m_pneumaticsSubsystem::frontIntakeClosed));

    // Rear intake
    new JoystickButton(m_operateController, 2)
        .toggleOnTrue(new InstantCommand(m_pneumaticsSubsystem::rearIntakeOpen))
        .toggleOnFalse(new InstantCommand(m_pneumaticsSubsystem::rearIntakeClosed));

    // shooter+hood activate
    new JoystickButton(m_operateController, 1).whileTrue(
        new AutoAim(m_hoodPIDsubsystem, m_visionSubsystem, m_shooterPIDsubsystem));

    // compressor toggle
    new JoystickButton(m_operateController, 1)
    .toggleOnTrue(new InstantCommand(m_pneumaticsSubsystem::compOff))
        .toggleOnFalse(new InstantCommand(m_pneumaticsSubsystem::compOn));

  }

  public InstantCommand DriveSimStart = new InstantCommand(m_driveSubsystem::DriveSiminit);

  public Command getAutonomousCommand() {

      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(Auto1.getInitialPose());

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            Auto1,
            m_driveSubsystem::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_driveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel),
            // RamseteCommand passes volts to the callback
            m_driveSubsystem::tankDriveVolts,
            m_driveSubsystem);
    
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen( () -> m_driveSubsystem.tankDriveVolts(0,0));
    
  }

  public void teleInit() {
    m_intakeSubsystem.setDefaultCommand(
                    new AutoIndex(m_intakeSubsystem, m_indexerSubsystem, m_pneumaticsSubsystem, m_sensors,
                                    m_operateController));

    m_driveSubsystem.setDefaultCommand(new StickDrive(m_driveSubsystem, m_driveController, m_turretPIDsubsystem));

  }
}
  // Create a voltage constraint to ensure we don't accelerate too fast


