// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors extends SubsystemBase {

  //creating AHRS device "navX"
  private AHRS navX;

  //simulated yaw angle, for simulation.
  private SimDouble m_simYawAngle;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private PhotonPoseEstimator photonPoseEstimator;

  private PhotonCamera frontCamera = new PhotonCamera("frontAprilTagCamera");

  PhotonCamera rearCamera = new PhotonCamera("rearAprilTagCamera");

  //Cam mounted facing forward, half a meter forward of center.
  Transform3d robotToFrontCam = new Transform3d(
      new Translation3d(0.5, 0.0, 0), 
      new Rotation3d(0, Units.degreesToRadians(0),0)); 
  
  //Cam mounted facing backwards, half a meter back of center.
  Transform3d robotToRearCam = new Transform3d(
      new Translation3d(-0.5, 0.0, 0), 
      new Rotation3d(0, 0, Units.degreesToRadians(180)));


  /** Creates a new IndexSensors. */
  public Sensors() {

    this.photonPoseEstimator = photonPoseEstimator;
    this.aprilTagFieldLayout = aprilTagFieldLayout;
    // instantiate navx over USB
    try {
      navX = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-USB: " + ex.getMessage(), true);
    }

     // sends WPI provided AprilTag locations to the PhotonVision field layout object
    try {
      AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Error loading AprilTagFieldLayout on init" + e.getMessage(), true);
    }

    // Construct PhotonPoseEstimator
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, frontCamera, robotToFrontCam);
  }

  @Override
  public void periodic() {
    //sending info for debugging to the SmartDashboard
    SmartDashboard.putNumber("Yaw", navXYaw());
    SmartDashboard.putNumber("Pitch", navXPitch());
    SmartDashboard.putNumber("Roll", navXRoll());
  }

  @Override
  public void simulationPeriodic() {
    //getting simulated device handle for navX
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    //setting simulated yaw angle to the JNI angle
    m_simYawAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  }

  public Rotation2d navXRotation2d() {
    return Rotation2d.fromDegrees(navXYaw());
  }

  public Pose3d photonPoseEstimate() {
    return null;
  }
  

  public void setNavXAngle(double angle) {
    m_simYawAngle.set(angle);
  }

  public double navXYaw() {
    return navX.getYaw();
  }

  public double navXPitch() {
    return navX.getPitch();
  }

  public double navXRoll() {
    return navX.getRoll();
  }

  public void zeroNavXHeading() {
    navX.reset();
  }

  public double navXTurnRate() {
    return navX.getRate();
  }
}
