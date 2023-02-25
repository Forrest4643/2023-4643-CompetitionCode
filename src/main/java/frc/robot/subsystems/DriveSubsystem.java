// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2023 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.dConstants;
import frc.robot.Constants.vConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.sql.Array;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  

  private double frontLeftRPM = 0;

  //@Log.Graph
  private double frontRightRPM;

  //@Log.Graph
  private double rearLeftRPM;

  //@Log.Graph
  private double rearRightRPM;

  @Log.Graph
  private double desiredFrontLeftRPM;

  //@Log.Graph
  private double desiredFrontRightRPM;

  //@Log.Graph
  private double desiredRearLeftRPM;

  //@Log.Graph
  private double desiredRearRightRPM;
 
  // defining motor names and CAN ID's
  private final CANSparkMax frontLeftSparkMax = new CANSparkMax(dConstants.leftFrontID, MotorType.kBrushless);
  private final CANSparkMax rearLeftSparkMax = new CANSparkMax(dConstants.leftRearID, MotorType.kBrushless);
  private final CANSparkMax frontRightSparkMax = new CANSparkMax(dConstants.rightFrontID, MotorType.kBrushless);
  private final CANSparkMax rearRightSparkMax = new CANSparkMax(dConstants.rightRearID, MotorType.kBrushless);

  // defining encoders
  public RelativeEncoder m_frontLeftEnc = frontLeftSparkMax.getEncoder();
  public RelativeEncoder m_rearLeftEnc = rearLeftSparkMax.getEncoder();
  public RelativeEncoder m_rearRightEnc = rearRightSparkMax.getEncoder();
  public RelativeEncoder m_frontRightEnc = frontRightSparkMax.getEncoder();
  // defining spark PID controllers
  
  private SparkMaxPIDController m_frontLeftPIDFF = frontLeftSparkMax.getPIDController();
  private SparkMaxPIDController m_rearLeftPIDFF = rearLeftSparkMax.getPIDController();
  private SparkMaxPIDController m_rearRightPIDFF = rearRightSparkMax.getPIDController();
  private SparkMaxPIDController m_frontRightPIDFF = frontRightSparkMax.getPIDController();

  // Locations of the wheels relative to the robot center.
  Translation2d m_frontLeftLocation = dConstants.frontLeftWheel;
  Translation2d m_frontRightLocation = dConstants.frontRightWheel;
  Translation2d m_backLeftLocation = dConstants.rearLeftWheel;
  Translation2d m_backRightLocation = dConstants.rearRightWheel;

  Translation2d centeredRotation = new Translation2d(0, 0);

  Translation2d frontRotation = new Translation2d(0, .5);

  
  /*creating a mecanum drive kinematics object which contains
  each of the four wheels position on the robot*/
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  //creating one object which contains each wheels rotational position
  MecanumDriveWheelPositions m_driveWheelPositions = new MecanumDriveWheelPositions(
    m_frontLeftEnc.getPosition(), 
    m_frontRightEnc.getPosition(), 
    m_rearLeftEnc.getPosition(), 
    m_rearRightEnc.getPosition());

  //creating a mecanum drive odometry object, this returns the robot's estimated position on the field. 
  MecanumDriveOdometry m_driveOdometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d(), m_driveWheelPositions);

  MecanumDrivePoseEstimator m_drivePoseEstimator = new MecanumDrivePoseEstimator(m_kinematics, new Rotation2d(), m_driveWheelPositions, new Pose2d(), 
  new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Kinematic estimate std deviations: X meters, Y meters, and RAD heading
  new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01)); // Vision estimate std deviations: X meters, Y meters, and RAD heading

  Sensors m_sensors;

  private Field2d m_field = new Field2d();

  // Creating a simulated photonvision system
  SimVisionSystem simVision = new SimVisionSystem(
      "frontAprilTagCamera",
      vConstants.camDiagFOV,
      new Transform3d(
          new Translation3d(0, 0, vConstants.cameraHeightM), 
          new Rotation3d()), vConstants.maxLEDRange, 
          vConstants.camResolutionWidth, 
          vConstants.camResolutionHeight, 
          vConstants.minTargetAreaPIX);

  // Creates DriveSubsystem
  public DriveSubsystem(Sensors m_sensors) { 

    //telling this class where to find the Sensors subsystem
    this.m_sensors = m_sensors;

    
    // defining all spark configs and burning it to flash memory. 
    m_frontLeftEnc.setVelocityConversionFactor(dConstants.velocityConversionFactor);
    m_frontRightEnc.setVelocityConversionFactor(dConstants.velocityConversionFactor);
    m_rearLeftEnc.setVelocityConversionFactor(dConstants.velocityConversionFactor);
    m_rearRightEnc.setVelocityConversionFactor(dConstants.velocityConversionFactor);

    m_frontLeftEnc.setPositionConversionFactor(dConstants.positionConversionFactor);
    m_frontRightEnc.setPositionConversionFactor(dConstants.positionConversionFactor);
    m_rearLeftEnc.setPositionConversionFactor(dConstants.positionConversionFactor);
    m_rearRightEnc.setPositionConversionFactor(dConstants.positionConversionFactor);

    frontRightSparkMax.setInverted(false);
    rearRightSparkMax.setInverted(false);
    frontLeftSparkMax.setInverted(true);
    rearLeftSparkMax.setInverted(true);

    frontRightSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);
    frontLeftSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);
    rearRightSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);
    rearLeftSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);

    m_frontLeftPIDFF.setP(dConstants.sparkMAXVelocitykP, 0);
    m_frontRightPIDFF.setP(dConstants.sparkMAXVelocitykP, 0);
    m_rearLeftPIDFF.setP(dConstants.sparkMAXVelocitykP, 0);
    m_rearRightPIDFF.setP(dConstants.sparkMAXVelocitykP, 0);

    m_frontLeftPIDFF.setI(dConstants.sparkMAXVelocitykI, 0);
    m_frontRightPIDFF.setI(dConstants.sparkMAXVelocitykI, 0);
    m_rearLeftPIDFF.setI(dConstants.sparkMAXVelocitykI, 0);
    m_rearRightPIDFF.setI(dConstants.sparkMAXVelocitykI, 0);

    m_frontLeftPIDFF.setD(dConstants.sparkMAXVelocitykD, 0);
    m_frontRightPIDFF.setD(dConstants.sparkMAXVelocitykD, 0);
    m_rearLeftPIDFF.setD(dConstants.sparkMAXVelocitykD, 0);
    m_rearRightPIDFF.setD(dConstants.sparkMAXVelocitykD, 0);

    m_frontLeftPIDFF.setFF(dConstants.sparkMAXVelocitykF, 0);
    m_frontRightPIDFF.setFF(dConstants.sparkMAXVelocitykF, 0);
    m_rearLeftPIDFF.setFF(dConstants.sparkMAXVelocitykF, 0);
    m_rearRightPIDFF.setFF(dConstants.sparkMAXVelocitykF, 0);

    m_frontLeftPIDFF.setSmartMotionMaxAccel(dConstants.wheelRPMaccel, 0);
    m_frontRightPIDFF.setSmartMotionMaxAccel(dConstants.wheelRPMaccel, 0);
    m_rearRightPIDFF.setSmartMotionMaxAccel(dConstants.wheelRPMaccel, 0);
    m_rearLeftPIDFF.setSmartMotionMaxAccel(dConstants.wheelRPMaccel, 0);  

    m_frontLeftPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_frontRightPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_rearRightPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_rearLeftPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_frontLeftPIDFF.setSmartMotionAllowedClosedLoopError(0, 0);
    m_frontRightPIDFF.setSmartMotionAllowedClosedLoopError(0, 0);
    m_rearRightPIDFF.setSmartMotionAllowedClosedLoopError(0, 0);
    m_rearLeftPIDFF.setSmartMotionAllowedClosedLoopError(0, 0);

    m_frontLeftPIDFF.setSmartMotionMaxVelocity(570, 0);
    m_frontRightPIDFF.setSmartMotionMaxVelocity(570, 0);
    m_rearRightPIDFF.setSmartMotionMaxVelocity(570, 0);
    m_rearLeftPIDFF.setSmartMotionMaxVelocity(570, 0);

  
    m_frontLeftPIDFF.setSmartMotionMinOutputVelocity(0, 0);
    m_frontRightPIDFF.setSmartMotionMinOutputVelocity(0, 0);
    m_rearRightPIDFF.setSmartMotionMinOutputVelocity(0, 0);
    m_rearLeftPIDFF.setSmartMotionMinOutputVelocity(0, 0);


    frontRightSparkMax.burnFlash();
    frontLeftSparkMax.burnFlash();
    rearRightSparkMax.burnFlash();
    rearLeftSparkMax.burnFlash();

    // Defining simulated vision target
    simVision.addVisionTargets(m_sensors.aprilTagFieldLayout);

    // sending simulated field data to SmartDashboard
    SmartDashboard.putData("Field", m_field);

  } // end Public DriveSubsystem


  public void DriveSiminit() {
    // this runs once at the start of Simulation

  }

  // reset DriveTrain encoders to 0
  public void resetDriveEncoders() {
    m_frontLeftEnc.setPosition(0);
    m_frontRightEnc.setPosition(0);
    m_rearLeftEnc.setPosition(0);
    m_rearRightEnc.setPosition(0);
  }

  public void cartesianMecanumDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed, Translation2d COR) {
  
  ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds
    (xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotationSpeed.getAsDouble(), m_sensors.navXRotation2d());
 
  double frontLeftMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).frontLeftMetersPerSecond;
  double frontRightMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).frontRightMetersPerSecond;
  double rearRightMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).rearRightMetersPerSecond;
  double rearLeftMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).rearLeftMetersPerSecond;

  desiredFrontRightRPM = metersPerSecondToRPM(frontRightMetersPerSecond);
  desiredFrontLeftRPM = metersPerSecondToRPM(frontLeftMetersPerSecond);
  desiredRearLeftRPM = metersPerSecondToRPM(rearLeftMetersPerSecond);
  desiredRearRightRPM = metersPerSecondToRPM(rearRightMetersPerSecond);

  m_frontLeftPIDFF.setReference(desiredFrontLeftRPM, ControlType.kSmartVelocity, 0);
  m_frontRightPIDFF.setReference(desiredFrontRightRPM, ControlType.kSmartVelocity, 0);
  m_rearRightPIDFF.setReference(desiredRearRightRPM, ControlType.kSmartVelocity, 0);
  m_rearLeftPIDFF.setReference(desiredRearLeftRPM, ControlType.kSmartVelocity, 0);

  frontLeftRPM = frontLeftSparkMax.getEncoder().getVelocity();
  frontRightRPM = frontRightSparkMax.getEncoder().getVelocity();
  rearRightRPM = rearRightSparkMax.getEncoder().getVelocity();
  rearLeftRPM = rearLeftSparkMax.getEncoder().getVelocity();

  }

  public void stopMotors() {
    frontRightSparkMax.set(0);
    frontLeftSparkMax.set(0);
    rearLeftSparkMax.set(0);
    rearRightSparkMax.set(0);
  }
  

  public void updateOdometry() {

    System.out.println(m_frontLeftEnc.getPosition());
    //sends gyro heading and wheel positions to pose estimator
    m_drivePoseEstimator.update(m_sensors.navXRotation2d(), m_driveWheelPositions);
    
      //gets estimated photonvision pose
      Optional<EstimatedRobotPose> result =
      m_sensors.getEstimatedGlobalPose(m_drivePoseEstimator.getEstimatedPosition());

    
    if(result.isPresent()) {
      //if there's a valid vision target, it's data is sent to the drive pose estimator
      EstimatedRobotPose camPose = result.get();
      m_drivePoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
      //if there's no target, the camera is moved off screen. 
      m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    //updating the robots estimated pose on the field
    m_field.setRobotPose(m_drivePoseEstimator.getEstimatedPosition());
  }

  private double metersPerSecondToRPM(double meterspersecond) {
    return (meterspersecond / 0.4787787204) * 60;
  }

  @Override
  public void periodic() {
    
    //updating drive odometry with most recent wheel and gyro data
    updateOdometry();



  } // end void periodic

  @Override
  public void simulationPeriodic() {

    //update photonvision simulation
    simVision.processFrame(m_driveOdometry.getPoseMeters());

    //sending simulated gyro heading to the main robot code
    m_sensors.setSimNavXAngle(m_driveOdometry.getPoseMeters().getRotation().getDegrees());

    updateOdometry();
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
