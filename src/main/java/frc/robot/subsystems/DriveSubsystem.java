// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2023 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import io.github.oblarg.oblog.Loggable;
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
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

public class DriveSubsystem extends SubsystemBase implements Loggable {

  private double m_rotationX = 0;
  private double m_rotationY = 0;

  // defining motor names and CAN ID's
  private final CANSparkMax frontLeftSparkMax = new CANSparkMax(driveConstants.kLeftFrontID, MotorType.kBrushless);
  private final CANSparkMax rearLeftSparkMax = new CANSparkMax(driveConstants.kLeftRearID, MotorType.kBrushless);
  private final CANSparkMax frontRightSparkMax = new CANSparkMax(driveConstants.kRightFrontID, MotorType.kBrushless);
  private final CANSparkMax rearRightSparkMax = new CANSparkMax(driveConstants.kRightRearID, MotorType.kBrushless);

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

  public double sparkMAXVelocitykP = 0.0002;

  public double sparkMAXVelocitykI = 0;

  public double sparkMAXVelocitykD = 0.0;

  public double sparkMAXVelocitykF = 0.002;


  /*
   * creating a mecanum drive kinematics object which contains
   * each of the four wheels position on the robot
   */


  // creating one object which contains each wheels rotational position
  MecanumDriveWheelPositions m_driveWheelPositions = new MecanumDriveWheelPositions();

  // creating a mecanum drive odometry object, this returns the robot's estimated
  // position on the field.
  Sensors m_sensors;

  MecanumDriveOdometry m_driveOdometry = new MecanumDriveOdometry(
    driveConstants.kDriveKinematics, new Rotation2d(), m_driveWheelPositions);

  MecanumDrivePoseEstimator m_drivePoseEstimator = new MecanumDrivePoseEstimator(driveConstants.kDriveKinematics, new Rotation2d(),
      m_driveWheelPositions, new Pose2d(),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.25, 0.25, Units.degreesToRadians(0.1)), // Kinematic estimate std deviations: X meters, Y
                                                                   // meters, and RAD heading
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.08, 0.08, Units.degreesToRadians(10))); // Vision estimate std deviations: X meters, Y
                                                                    // meters, and RAD heading

  public Pose2d navXPose2d = new Pose2d(-100, -100, null);

  private Field2d m_field = new Field2d();

  MecanumDriveWheelSpeeds m_driveMeterSeconds = new MecanumDriveWheelSpeeds();



  // Creates DriveSubsystem
  public DriveSubsystem(Sensors m_Sensors) {

    // telling this class where to find the Sensors subsystem
    this.m_sensors = m_Sensors;

    // defining all spark configs and burning it to flash memory.
    m_frontLeftEnc.setVelocityConversionFactor(driveConstants.kVelocityConversionFactorRPM);
    m_frontRightEnc.setVelocityConversionFactor(driveConstants.kVelocityConversionFactorRPM);
    m_rearLeftEnc.setVelocityConversionFactor(driveConstants.kVelocityConversionFactorRPM);
    m_rearRightEnc.setVelocityConversionFactor(driveConstants.kVelocityConversionFactorRPM);

    m_frontLeftEnc.setPositionConversionFactor(driveConstants.kPositionConversionFactorMeters);
    m_frontRightEnc.setPositionConversionFactor(driveConstants.kPositionConversionFactorMeters);
    m_rearLeftEnc.setPositionConversionFactor(driveConstants.kPositionConversionFactorMeters);
    m_rearRightEnc.setPositionConversionFactor(driveConstants.kPositionConversionFactorMeters);

    frontRightSparkMax.setInverted(false);
    rearRightSparkMax.setInverted(false);
    frontLeftSparkMax.setInverted(true);
    rearLeftSparkMax.setInverted(true);

    frontRightSparkMax.setSmartCurrentLimit(driveConstants.kDriveSparkSmartCurrentLimit);
    frontLeftSparkMax.setSmartCurrentLimit(driveConstants.kDriveSparkSmartCurrentLimit);
    rearRightSparkMax.setSmartCurrentLimit(driveConstants.kDriveSparkSmartCurrentLimit);
    rearLeftSparkMax.setSmartCurrentLimit(driveConstants.kDriveSparkSmartCurrentLimit);

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

    m_frontLeftPIDFF.setSmartMotionMaxAccel(driveConstants.kWheelRPMaccel, 0);
    m_frontRightPIDFF.setSmartMotionMaxAccel(driveConstants.kWheelRPMaccel, 0);
    m_rearRightPIDFF.setSmartMotionMaxAccel(driveConstants.kWheelRPMaccel, 0);
    m_rearLeftPIDFF.setSmartMotionMaxAccel(driveConstants.kWheelRPMaccel, 0);

    m_frontLeftPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_frontRightPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_rearRightPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_rearLeftPIDFF.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_frontLeftPIDFF.setSmartMotionAllowedClosedLoopError(driveConstants.kAllowedVelocityErrorRPM, 0);
    m_frontRightPIDFF.setSmartMotionAllowedClosedLoopError(driveConstants.kAllowedVelocityErrorRPM, 0);
    m_rearRightPIDFF.setSmartMotionAllowedClosedLoopError(driveConstants.kAllowedVelocityErrorRPM, 0);
    m_rearLeftPIDFF.setSmartMotionAllowedClosedLoopError(driveConstants.kAllowedVelocityErrorRPM, 0);

    m_frontLeftPIDFF.setSmartMotionMaxVelocity(driveConstants.kMaxWheelRPM, 0);
    m_frontRightPIDFF.setSmartMotionMaxVelocity(driveConstants.kMaxWheelRPM, 0);
    m_rearRightPIDFF.setSmartMotionMaxVelocity(driveConstants.kMaxWheelRPM, 0);
    m_rearLeftPIDFF.setSmartMotionMaxVelocity(driveConstants.kMaxWheelRPM, 0);

    m_frontLeftPIDFF.setSmartMotionMinOutputVelocity(0, 0);
    m_frontRightPIDFF.setSmartMotionMinOutputVelocity(0, 0);
    m_rearRightPIDFF.setSmartMotionMinOutputVelocity(0, 0);
    m_rearLeftPIDFF.setSmartMotionMinOutputVelocity(0, 0);

    frontRightSparkMax.burnFlash();
    frontLeftSparkMax.burnFlash();
    rearRightSparkMax.burnFlash();
    rearLeftSparkMax.burnFlash();
    
    // sending simulated field data to SmartDashboard
    SmartDashboard.putData("Field", m_field);

  } // end Public DriveSubsystem

  // reset DriveTrain encoders to 0
  public void resetDriveEncoders() {
    m_frontLeftEnc.setPosition(0);
    m_frontRightEnc.setPosition(0);
    m_rearLeftEnc.setPosition(0);
    m_rearRightEnc.setPosition(0);
  }

  public void cartesianMecanumDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed, DoubleSupplier corX, DoubleSupplier corY) {

    //creating a chassis speeds object which using the onboard NavX, contains wheel speeds to move relative to the field.
    //the speed inputs to this should be in meters/s, and rotation in rads/s 
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      //converting the -1 to 1 controller input to meters per second
      xSpeed.getAsDouble() * driveConstants.kMaxAttainableMetersPerSecond, 
      ySpeed.getAsDouble() * driveConstants.kMaxAttainableMetersPerSecond,
        rotationSpeed.getAsDouble(), Rotation2d.fromDegrees(m_sensors.wrappedNavXHeading()));    

    //these are the x and y offsets for the center of rotation
    m_rotationX = corX.getAsDouble();
    m_rotationY = corY.getAsDouble();

    //calculating each wheels requested M/s velocity from the chassis speeds object
    MecanumDriveWheelSpeeds driveWheelSpeeds = new MecanumDriveWheelSpeeds(
      driveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds, new Translation2d(m_rotationX, m_rotationY)).frontLeftMetersPerSecond, 
      driveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds,  new Translation2d(m_rotationX, m_rotationY)).frontRightMetersPerSecond, 
      driveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds,  new Translation2d(m_rotationX, m_rotationY)).rearLeftMetersPerSecond, 
      driveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds,  new Translation2d(m_rotationX, m_rotationY)).rearRightMetersPerSecond);

    driveWheelSpeeds.desaturate(driveConstants.kMaxAttainableMetersPerSecond);

    setDriveWheelMetersPerSecond(new MecanumDriveWheelSpeeds(driveWheelSpeeds.frontLeftMetersPerSecond, driveWheelSpeeds.frontRightMetersPerSecond,
    driveWheelSpeeds.rearLeftMetersPerSecond, driveWheelSpeeds.rearRightMetersPerSecond));

  }

  //stops all drive motors
  public void stopMotors() {
    frontRightSparkMax.set(0);
    frontLeftSparkMax.set(0);
    rearLeftSparkMax.set(0); 
    rearRightSparkMax.set(0);
  }

  //updating global estimated robot pose
  public void updateOdometry() {

    // updates and gets estimated photonvision pose
    Optional<EstimatedRobotPose> result = m_sensors.getEstimatedGlobalPose(m_drivePoseEstimator.getEstimatedPosition());

    if (result.isPresent()) {

      // if there's a valid vision target, it's data is sent to the drive pose
      // estimator
      EstimatedRobotPose camPose = result.get();

      //adding a vision measurement to the drive pose estimator, 
      //the timestamp "camPose.timestampSeconds" is used for latency compensation. 
      m_drivePoseEstimator.addVisionMeasurement
        (camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

      //set camera pose on field
      m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {

      // if there's no target, the camera is moved off screen.
      m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    //creating an object which contains the rotations spun of each wheel, in the natively converted units of Meters traveled. 
    m_driveWheelPositions = new MecanumDriveWheelPositions(
      m_frontLeftEnc.getPosition(), m_frontRightEnc.getPosition(), 
      m_rearLeftEnc.getPosition(), m_rearRightEnc.getPosition());

    // sends gyro heading and wheel positions to pose estimator
    m_drivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), 
      new Rotation2d(Units.degreesToRadians(m_sensors.wrappedNavXHeading())), 
        m_driveWheelPositions);

    // updating the robots estimated pose on the field
    m_field.setRobotPose(m_drivePoseEstimator.getEstimatedPosition());
  }

  //conversion from wheel metersperSecond to rpm. 
  private double metersPerSecondToRPM(double meterspersecond) {
    return (meterspersecond / driveConstants.kWheelCircumferenceMeters) * 60;
  }

  @Override
  public void periodic() {
    // updating drive odometry with most recent wheel and gyro data
    updateOdometry();

  } // end void periodic

  @Override
  public void simulationPeriodic() {
  } // end simulationPeriodic

  //returns an object with each of the wheels current velocity in M/s
  public MecanumDriveWheelSpeeds getCurrentWheelSpeedRPMs() {
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

  public void setDriveWheelMetersPerSecond(MecanumDriveWheelSpeeds speeds){
    m_frontLeftPIDFF.setReference(metersPerSecondToRPM(speeds.frontLeftMetersPerSecond), ControlType.kSmartVelocity, 0);
    m_frontRightPIDFF.setReference(metersPerSecondToRPM(speeds.frontRightMetersPerSecond), ControlType.kSmartVelocity, 0);
    m_rearRightPIDFF.setReference(metersPerSecondToRPM(speeds.rearRightMetersPerSecond), ControlType.kSmartVelocity, 0);
    m_rearLeftPIDFF.setReference(metersPerSecondToRPM(speeds.rearLeftMetersPerSecond), ControlType.kSmartVelocity, 0);

  }

  // returns the read position of the robot on the field
  public Pose2d getPose() {
    return m_drivePoseEstimator.getEstimatedPosition();
  }

  // resets the read position of the robot on the field
  public void resetOdometry(Pose2d pose) {
    resetDriveEncoders();

    m_driveOdometry.resetPosition(new Rotation2d(
      Units.degreesToRadians(m_sensors.wrappedNavXHeading())), 
        m_driveWheelPositions, new Pose2d());
  }
}
