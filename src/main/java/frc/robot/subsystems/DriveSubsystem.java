// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2023 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.dConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

public class DriveSubsystem extends SubsystemBase implements Loggable {

  private double m_desiredFrontLeftRPM = 0;
  private double m_desiredFrontRightRPM = 0;
  private double m_desiredRearLeftRPM = 0;
  private double m_desiredRearRightRPM = 0;

  private double m_rotationX = 0;
  private double m_rotationY = 0;

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

  public double sparkMAXVelocitykP = 0.00015;
  @Config
  public void setSparkkP(double kP) {
      if (kP != 0){
        sparkMAXVelocitykP = kP;
      }
  }

  public double sparkMAXVelocitykI = 0;
  @Config
  public void setSparkkI(double kI) {
    if (kI != 0){
      sparkMAXVelocitykI = kI;
    }
  }

  public double sparkMAXVelocitykD = 0.0;
  @Config
  public void setSparkkD(double kD) {
    if (kD != 0) {
      sparkMAXVelocitykD = kD;
    }
  }

  public double sparkMAXVelocitykF = 0.0025;
  @Config
  public void setSparkkF(double kF) {
    if (kF != 0) {
      sparkMAXVelocitykF = kF;
    }
  }

  /*
   * creating a mecanum drive kinematics object which contains
   * each of the four wheels position on the robot
   */
  public MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // creating one object which contains each wheels rotational position
  MecanumDriveWheelPositions m_driveWheelPositions = new MecanumDriveWheelPositions();

  // creating a mecanum drive odometry object, this returns the robot's estimated
  // position on the field.
  Sensors m_sensors;

  MecanumDriveOdometry m_driveOdometry = new MecanumDriveOdometry(
    m_kinematics, new Rotation2d(), m_driveWheelPositions);

  MecanumDrivePoseEstimator m_drivePoseEstimator = new MecanumDrivePoseEstimator(m_kinematics, new Rotation2d(),
      m_driveWheelPositions, new Pose2d(),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.25, 0.25, Units.degreesToRadians(0.1)), // Kinematic estimate std deviations: X meters, Y
                                                                   // meters, and RAD heading
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.08, 0.08, Units.degreesToRadians(10))); // Vision estimate std deviations: X meters, Y
                                                                    // meters, and RAD heading

  public Pose2d navXPose2d = new Pose2d(-100, -100, null);

  private Pose2d lastVisionPose = new Pose2d();

  private Field2d m_field = new Field2d();


  // Creates DriveSubsystem
  public DriveSubsystem(Sensors m_sensors) {

    // telling this class where to find the Sensors subsystem
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

    m_frontLeftPIDFF.setP(sparkMAXVelocitykP, 0);
    m_frontRightPIDFF.setP(sparkMAXVelocitykP, 0);
    m_rearLeftPIDFF.setP(sparkMAXVelocitykP, 0);
    m_rearRightPIDFF.setP(sparkMAXVelocitykP, 0);

    m_frontLeftPIDFF.setI(sparkMAXVelocitykI, 0);
    m_frontRightPIDFF.setI(sparkMAXVelocitykI, 0);
    m_rearLeftPIDFF.setI(sparkMAXVelocitykI, 0);
    m_rearRightPIDFF.setI(sparkMAXVelocitykI, 0);

    m_frontLeftPIDFF.setD(sparkMAXVelocitykD, 0);
    m_frontRightPIDFF.setD(sparkMAXVelocitykD, 0);
    m_rearLeftPIDFF.setD(sparkMAXVelocitykD, 0);
    m_rearRightPIDFF.setD(sparkMAXVelocitykD, 0);

    m_frontLeftPIDFF.setFF(sparkMAXVelocitykF, 0);
    m_frontRightPIDFF.setFF(sparkMAXVelocitykF, 0);
    m_rearLeftPIDFF.setFF(sparkMAXVelocitykF, 0);
    m_rearRightPIDFF.setFF(sparkMAXVelocitykF, 0);

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

  public void cartesianMecanumDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed, DoubleSupplier corX, DoubleSupplier corY) {

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed.getAsDouble() * dConstants.maxAttainableMetersPerSecond, 
      ySpeed.getAsDouble() * dConstants.maxAttainableMetersPerSecond,
        rotationSpeed.getAsDouble(), Rotation2d.fromDegrees(m_sensors.NavXFusedHeading()));    

    m_rotationX = corX.getAsDouble();
    m_rotationY = corY.getAsDouble();

    MecanumDriveWheelSpeeds driveWheelSpeeds = new MecanumDriveWheelSpeeds(
      m_kinematics.toWheelSpeeds(chassisSpeeds, new Translation2d(m_rotationX, m_rotationY)).frontLeftMetersPerSecond, 
      m_kinematics.toWheelSpeeds(chassisSpeeds,  new Translation2d(m_rotationX, m_rotationY)).frontRightMetersPerSecond, 
      m_kinematics.toWheelSpeeds(chassisSpeeds,  new Translation2d(m_rotationX, m_rotationY)).rearLeftMetersPerSecond, 
      m_kinematics.toWheelSpeeds(chassisSpeeds,  new Translation2d(m_rotationX, m_rotationY)).rearRightMetersPerSecond);

    driveWheelSpeeds.desaturate(dConstants.maxAttainableMetersPerSecond);

    m_desiredFrontLeftRPM = metersPerSecondToRPM(driveWheelSpeeds.frontLeftMetersPerSecond);
    m_desiredFrontRightRPM = metersPerSecondToRPM(driveWheelSpeeds.frontRightMetersPerSecond);
    m_desiredRearLeftRPM = metersPerSecondToRPM(driveWheelSpeeds.rearLeftMetersPerSecond);
    m_desiredRearRightRPM = metersPerSecondToRPM(driveWheelSpeeds.rearRightMetersPerSecond);

    m_frontLeftPIDFF.setReference(m_desiredFrontLeftRPM, ControlType.kSmartVelocity, 0);
    m_frontRightPIDFF.setReference(m_desiredFrontRightRPM, ControlType.kSmartVelocity, 0);
    m_rearRightPIDFF.setReference(m_desiredRearRightRPM, ControlType.kSmartVelocity, 0);
    m_rearLeftPIDFF.setReference(m_desiredRearLeftRPM, ControlType.kSmartVelocity, 0);

  }

  public void stopMotors() {
    frontRightSparkMax.set(0);
    frontLeftSparkMax.set(0);
    rearLeftSparkMax.set(0);
    rearRightSparkMax.set(0);
  }

  public void updateOdometry() {

    // gets estimated photonvision pose
    Optional<EstimatedRobotPose> result = m_sensors.getEstimatedGlobalPose(m_drivePoseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      // if there's a valid vision target, it's data is sent to the drive pose
      // estimator
      EstimatedRobotPose camPose = result.get();

      m_drivePoseEstimator.addVisionMeasurement
        (camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

      m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());

      lastVisionPose = getPose();
      m_sensors.resetNavxDisplacement();
    } else {
      // if there's no target, the camera is moved off screen.
      m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));

      navXPose2d = lastVisionPose.transformBy(m_sensors.navXdisplacement());

      m_drivePoseEstimator.addVisionMeasurement(navXPose2d, Timer.getFPGATimestamp(), 
        // NavX estimate std deviations: X meters, Y meters, and RAD heading
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 999)); 
    }

    m_driveWheelPositions = new MecanumDriveWheelPositions(
      -m_frontLeftEnc.getPosition(),
      -m_frontRightEnc.getPosition(),
      -m_rearLeftEnc.getPosition(),
      -m_rearRightEnc.getPosition());

    // sends gyro heading and wheel positions to pose estimator
    m_drivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(Units.degreesToRadians(m_sensors.NavXFusedHeading())), m_driveWheelPositions);

    // updating the robots estimated pose on the field
    m_field.setRobotPose(m_drivePoseEstimator.getEstimatedPosition());
  }

  private double metersPerSecondToRPM(double meterspersecond) {
    return (meterspersecond / 0.4787787204) * 60;
  }

  @Override
  public void periodic() {
    // updating drive odometry with most recent wheel and gyro data
    updateOdometry();

  } // end void periodic

  @Override
  public void simulationPeriodic() {
  } // end simulationPeriodic

  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      m_frontLeftEnc.getVelocity(), m_frontRightEnc.getVelocity(), 
      m_rearLeftEnc.getVelocity(), m_rearRightEnc.getVelocity());
  }

  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    frontLeftSparkMax.setVoltage(volts.frontLeftVoltage);
    rearLeftSparkMax.setVoltage(volts.rearLeftVoltage);
    frontRightSparkMax.setVoltage(volts.frontRightVoltage);
    rearRightSparkMax.setVoltage(volts.rearRightVoltage);
  }

  // returns the read position of the robot on the field
  public Pose2d getPose() {
    return m_drivePoseEstimator.getEstimatedPosition();
  }

  // resets the read position of the robot on the field
  public void resetOdometry(Pose2d pose) {
    resetDriveEncoders();
    m_driveOdometry.resetPosition(new Rotation2d(Units.degreesToRadians(m_sensors.NavXFusedHeading())), m_driveWheelPositions, new Pose2d());
  }
}
