// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors extends SubsystemBase {

  //creating AHRS device "navX"
  private AHRS navX;

  //simulated yaw angle, for simulation.
  private SimDouble m_simYawAngle;

  public AprilTagFieldLayout aprilTagFieldLayout;

  public PhotonPoseEstimator photonFrontPoseEstimator;

  //public PhotonPoseEstimator photonrearPoseEstimator;

  public PhotonCamera frontCamera = new PhotonCamera("frontAprilTagCamera");

  //public PhotonCamera rearCamera = new PhotonCamera("rearAprilTagCamera");

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

    // instantiate navx over USB
    try {
      navX = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP: " + ex.getMessage(), true);
    }

     // sends WPI provided AprilTag locations to the PhotonVision field layout object
    try {
      this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Error loading AprilTagFieldLayout in Sensors:" + e.getMessage(), true);
    }

    // Construct PhotonPoseEstimator
    this.photonFrontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, frontCamera, robotToFrontCam);
    //PhotonPoseEstimator photonRearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, rearCamera, robotToRearCam);

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
    return navX.getRotation2d();
  }

  public double navXYaw() {
    return navX.getYaw();
  }

  public double NavXFusedHeading() {
    return navX.getFusedHeading();
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

  public void setSimNavXAngle(double angle) {
    m_simYawAngle.set(angle);
  }

  public double navXTurnRate() {
    return navX.getRate();
  }

  public Translation2d navXdisplacement() {
    return new Translation2d(navX.getDisplacementX(), navX.getDisplacementY());
  }

  public void resetNavxDisplacement() {
    navX.resetDisplacement();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      
    photonFrontPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    return photonFrontPoseEstimator.update();
  }

}
