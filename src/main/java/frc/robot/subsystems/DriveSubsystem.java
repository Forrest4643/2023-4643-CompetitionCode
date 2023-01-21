// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {


  // defining motor names and CAN ID's
  private final CANSparkMax frontLeftSparkMax = new CANSparkMax(DriveConstants.leftFrontID, MotorType.kBrushless);
  private final CANSparkMax rearLeftSparkMax = new CANSparkMax(DriveConstants.leftRearID, MotorType.kBrushless);
  private final CANSparkMax frontRightSparkMax = new CANSparkMax(DriveConstants.rightFrontID, MotorType.kBrushless);
  private final CANSparkMax rearRightSparkMax = new CANSparkMax(DriveConstants.rightRearID, MotorType.kBrushless);

  // defining encoders
  public RelativeEncoder m_frontLeftDriveEncoder = frontLeftSparkMax.getEncoder();
  public RelativeEncoder m_rearLeftDriveEncoder = rearLeftSparkMax.getEncoder();
  public RelativeEncoder m_rearRightDriveEncoder = rearRightSparkMax.getEncoder();
  public RelativeEncoder m_frontRightDriveEncoder = frontRightSparkMax.getEncoder();

  // Locations of the wheels relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(0.7, 0.7);
  Translation2d m_frontRightLocation = new Translation2d(0.7, -0.7);
  Translation2d m_backLeftLocation = new Translation2d(-0.7, 0.7);
  Translation2d m_backRightLocation = new Translation2d(-0.7, -0.7);
  
  // Defining mecanum drive kinematics
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  MecanumDriveWheelPositions m_driveWheelPositions = new MecanumDriveWheelPositions(
    m_frontLeftDriveEncoder.getPosition(), 
    m_frontRightDriveEncoder.getPosition(), 
    m_rearLeftDriveEncoder.getPosition(), 
    m_rearRightDriveEncoder.getPosition());

  MecanumDriveOdometry m_driveOdometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d(), m_driveWheelPositions);

  MecanumDrive m_robotDrive = new MecanumDrive(frontLeftSparkMax, rearLeftSparkMax, frontRightSparkMax, rearRightSparkMax);

  SlewRateLimiter drivLimiter = new SlewRateLimiter(.01);
  Sensors m_sensors;

  private Field2d m_field = new Field2d();


  // Creating a simulated photonvision system
  SimVisionSystem simVision = new SimVisionSystem(
      "photonvision",
      VisionConstants.camDiagFOV,
      new Transform3d(
          new Translation3d(0, 0, VisionConstants.cameraHeightM), 
          new Rotation3d()), VisionConstants.maxLEDRange, 
          VisionConstants.camResolutionWidth, 
          VisionConstants.camResolutionHeight, 
          VisionConstants.minTargetAreaPIX);

  // Creates DriveSubsystem
  public DriveSubsystem(Sensors m_sensors) { 

    this.m_sensors = m_sensors;

    // defining velocity conversion factor
    m_frontLeftDriveEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    m_frontRightDriveEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    m_rearLeftDriveEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    m_rearRightDriveEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);


    frontRightSparkMax.setInverted(false);
    rearRightSparkMax.setInverted(false);
    frontLeftSparkMax.setInverted(true);
    rearLeftSparkMax.setInverted(true);
    frontRightSparkMax.setSmartCurrentLimit(80);
    frontLeftSparkMax.setSmartCurrentLimit(80);
    rearRightSparkMax.setSmartCurrentLimit(80);
    rearLeftSparkMax.setSmartCurrentLimit(80);
    
    // Defining simulated vision target
    simVision.addSimVisionTarget(
        new SimVisionTarget(FieldConstants.aprilTags.get(1), 
          .1, .3, 1));
      
    // sending simulated field data to SmartDashboard
    SmartDashboard.putData("Field", m_field);

  } // end Public DriveSubsystem


  public void DriveSiminit() {
    // this runs once at the start of Simulation

  }

  // reset DriveTrain encoders to 0
  public void resetDriveEncoders() {
    m_frontLeftDriveEncoder.setPosition(0);
    m_frontRightDriveEncoder.setPosition(0);
    m_rearLeftDriveEncoder.setPosition(0);
    m_rearRightDriveEncoder.setPosition(0);
  }

  public void cartesianMecanumDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed) {

    m_robotDrive.driveCartesian(xSpeed.getAsDouble() / 2, -ySpeed.getAsDouble() / 2, rotationSpeed.getAsDouble(), m_sensors.navXRotation2d());
    m_robotDrive.setDeadband(.02);
  }

  @Override
  public void periodic() {
    
    // pass telemetry data to get Odometry data
    m_driveOdometry.update(m_sensors.navXRotation2d(), m_driveWheelPositions);
    m_field.setRobotPose(getPose());

  } // end void periodic

  @Override
  public void simulationPeriodic() {

     //update photonvision simulation
    simVision.processFrame(m_driveOdometry.getPoseMeters());
    m_driveOdometry.update(m_sensors.navXRotation2d(), m_driveWheelPositions);
    m_field.setRobotPose(getPose());

    

     //Debug info


     //sending simulated gyro heading to the main robot code
     m_sensors.setNavXAngle(getPose().getRotation().getDegrees()); //TODO

  } // end simulationPeriodic

  // returns the read position of the robot on the field
  public Pose2d getPose() {
    return m_driveOdometry.getPoseMeters();
  }

  // resets the read position of the robot on the field
  public void resetOdometry(Pose2d pose) {
    resetDriveEncoders();
    m_driveOdometry.resetPosition(m_sensors.navXRotation2d(), m_driveWheelPositions, new Pose2d());
  }
}
