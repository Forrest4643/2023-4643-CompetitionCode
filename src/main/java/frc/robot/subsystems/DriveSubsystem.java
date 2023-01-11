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

public class DriveSubsystem extends SubsystemBase {


  // defining motor names and CAN ID's
  private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.leftFrontID, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.leftRearID, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.rightFrontID, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.rightRearID, MotorType.kBrushless);

  // defining encoders
  public RelativeEncoder m_leftEncoder = leftLeader.getEncoder();
  public RelativeEncoder m_rightEncoder = rightLeader.getEncoder();

  // Defining drivetrain DifferentialDrive
  public final DifferentialDrive m_robotDrive = new DifferentialDrive(leftLeader, rightLeader);

  SlewRateLimiter driveSlew = new SlewRateLimiter(DriveConstants.driveSlew);
  SlewRateLimiter turnSlew = new SlewRateLimiter(DriveConstants.turnSlew);

  Sensors m_sensors;

  private Field2d m_field = new Field2d();

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      new Rotation2d(0), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d(0, 0, new Rotation2d()));

  Pose2d farTargetPose = new Pose2d(new Translation2d(VisionConstants.tgtXPos, VisionConstants.tgtYPos),
      new Rotation2d(0.0));

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

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
      7.29, // 7.29:1 gearing reduction.
      7.5, // MOI of 7.5 kg m^2 (from CAD model).
      60.0, // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.7112, // The track width is 0.7112 meters.

      // The standard deviations for measurement noise:
      // x and y: 0.001 m
      // heading: 0.001 rad
      // l and r velocity: 0.1 m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)); // end m_driveSim

  // Creates DriveSubsystem
  public DriveSubsystem(Sensors m_sensors) { 

    this.m_sensors = m_sensors;


    // motor inversions
    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    // defining velocity conversion factor
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);

    m_leftEncoder.setPositionConversionFactor(DriveConstants.positionConversionFactor);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.positionConversionFactor);
    // setting leftFront to follow leftRear,
    // and rightFront to follow rightRear
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    // Defining simulated vision target
    simVision.addSimVisionTarget(
        new SimVisionTarget(FieldConstants.aprilTags.get(1), 
          .1, .3, 1));

    simVision.addSimVisionTarget(
        new SimVisionTarget(new Pose3d(Units.inchesToMeters(610.77), 
          Units.inchesToMeters(42.19), 
          Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.PI)), 
          .1, .3, 1));
      

    // sending simulated field data to SmartDashboard
    SmartDashboard.putData("Field", m_field);

  } // end Public DriveSubsystem

  public void DriveSiminit() {
    // this runs once at the start of Simulation
  }

  // reset DriveTrain encoders to 0
  public void resetDriveEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    
    // pass telemetry data to get Odometry data
    m_odometry.update(m_sensors.navXRotation2d(),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());
    m_field.setRobotPose(getPose());

  } // end void periodic

  @Override
  public void simulationPeriodic() {
     // Sets the sim inputs, -1 to 1 signal multiplied by robot controller voltage
     m_driveSim.setInputs(leftLeader.get() * RobotController.getInputVoltage(),
     rightLeader.get() * RobotController.getInputVoltage());

     // Advance the model by 20 ms. Note that if you are running this
     // subsystem in a separate thread or have changed the nominal timestep
     // of TimedRobot, this value needs to match it.
     m_driveSim.update(0.02);

     // update photonvision simulation
     simVision.processFrame(m_driveSim.getPose());

     // sending simulated encoder values to the main robot code
     m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
     m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());

     // Debug info
     System.out.println("leftDriveDist:" + m_leftEncoder.getPosition());
     System.out.println("rightDriveDist:" + m_rightEncoder.getPosition());

     // sending simulated gyro heading to the main robot code
     m_sensors.setNavXAngle(-m_driveSim.getHeading().getDegrees());

  } // end simulationPeriodic

  // main setDrive void, this is used for the StickDrive command in TeleOp
  public void setDrive(double Speed, double turnRate) {

    double sinTurn = DriveConstants.turnSin * (Math.sin(turnRate));

    double sinSpeed = (Math.sin(Speed));

    // this ensures that negative inputs yield negative outputs,
    // and vise versa
    if (Speed < 0) {
      sinSpeed = Math.abs(sinSpeed) * -1;
    }
    if (turnRate < 0) {
      sinTurn = Math.abs(sinTurn) * -1;
    }

    m_robotDrive.arcadeDrive(driveSlew.calculate(sinSpeed), turnSlew.calculate(sinTurn) / 1.5);

    // debug info
    SmartDashboard.putNumber("sinturn", sinTurn);
    SmartDashboard.putNumber("sinspeed", sinSpeed);
  } // end setDrive

  // sets drive motors to a given voltage
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //This uses .set and a divider of 12 because to run simulation the .set 
    //command updates the simulated motor values, the .setVoltage command does not.
    leftLeader.set(leftVolts / 12);
    rightLeader.set(rightVolts / 12);
    m_robotDrive.feed();
  }

  // this is used for path-following.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  // returns the average distance travelled between the left and right wheels
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  // returns the read position of the robot on the field
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // resets the read position of the robot on the field
  public void resetOdometry(Pose2d pose) {
    resetDriveEncoders();
    m_odometry.resetPosition(m_sensors.navXRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }
}
