// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2023 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Constants.aConstants;
import frc.robot.Constants.dConstants;
import frc.robot.Constants.vConstants;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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


    frontRightSparkMax.setInverted(false);
    rearRightSparkMax.setInverted(false);
    frontLeftSparkMax.setInverted(true);
    rearLeftSparkMax.setInverted(true);

    frontRightSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);
    frontLeftSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);
    rearRightSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);
    rearLeftSparkMax.setSmartCurrentLimit(dConstants.driveSparkSmartCurrentLimit);

    m_frontLeftPIDFF.setP(dConstants.sparkMAXVelocitykP);
    m_frontRightPIDFF.setP(dConstants.sparkMAXVelocitykP);
    m_rearLeftPIDFF.setP(dConstants.sparkMAXVelocitykP);
    m_rearRightPIDFF.setP(dConstants.sparkMAXVelocitykP);

    m_frontLeftPIDFF.setI(dConstants.sparkMAXVelocitykI);
    m_frontRightPIDFF.setI(dConstants.sparkMAXVelocitykI);
    m_rearLeftPIDFF.setI(dConstants.sparkMAXVelocitykI);
    m_rearRightPIDFF.setI(dConstants.sparkMAXVelocitykI);

    m_frontLeftPIDFF.setD(dConstants.sparkMAXVelocitykD);
    m_frontRightPIDFF.setD(dConstants.sparkMAXVelocitykD);
    m_rearLeftPIDFF.setD(dConstants.sparkMAXVelocitykD);
    m_rearRightPIDFF.setD(dConstants.sparkMAXVelocitykD);

    m_frontLeftPIDFF.setFF(dConstants.sparkMAXVelocitykF);
    m_frontRightPIDFF.setFF(dConstants.sparkMAXVelocitykF);
    m_rearLeftPIDFF.setFF(dConstants.sparkMAXVelocitykF);
    m_rearRightPIDFF.setFF(dConstants.sparkMAXVelocitykF);
    
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

  public void cartesianMecanumDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed) {
  
  ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotationSpeed.getAsDouble(), m_sensors.navXRotation2d());
 
  double frontLeftMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).frontLeftMetersPerSecond;
  double frontRightMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).frontRightMetersPerSecond;
  double rearRightMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).rearRightMetersPerSecond;
  double rearLeftMetersPerSecond = m_kinematics.toWheelSpeeds(chassisSpeeds).rearLeftMetersPerSecond;

  m_frontLeftPIDFF.setReference(metersPerSecondToRPM(frontLeftMetersPerSecond), ControlType.kVelocity);
  m_frontRightPIDFF.setReference(metersPerSecondToRPM(frontRightMetersPerSecond), ControlType.kVelocity);
  m_rearRightPIDFF.setReference(metersPerSecondToRPM(rearRightMetersPerSecond), ControlType.kVelocity);
  m_rearLeftPIDFF.setReference(metersPerSecondToRPM(rearLeftMetersPerSecond), ControlType.kVelocity);

  SmartDashboard.putNumber("desiredFrontLeftRPM:", metersPerSecondToRPM(frontLeftMetersPerSecond));
  SmartDashboard.putNumber("desiredFrontRightRPM", metersPerSecondToRPM(frontRightMetersPerSecond));
  SmartDashboard.putNumber("desiredRearLeftRPM", metersPerSecondToRPM(rearLeftMetersPerSecond));
  SmartDashboard.putNumber("desiredRearRightRPM", metersPerSecondToRPM(rearRightMetersPerSecond));

  SmartDashboard.putNumber("frontRightRPM", m_frontRightEnc.getVelocity());
  SmartDashboard.putNumber("frontLeftRPM", m_frontLeftEnc.getVelocity());
  SmartDashboard.putNumber("rearRightRPM", m_rearRightEnc.getVelocity());
  SmartDashboard.putNumber("rearLeftRPM", m_rearLeftEnc.getVelocity());
  }

  public void stopMotors() {
    frontRightSparkMax.set(0);
    frontLeftSparkMax.set(0);
    rearLeftSparkMax.set(0);
    rearRightSparkMax.set(0);
  }
  

  public void updateOdometry() {
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


    m_field.setRobotPose(m_driveOdometry.getPoseMeters());

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
