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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.visionConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors extends SubsystemBase {

  //creating AHRS device "navX"
  private AHRS navX;

  //private AHRS armNavX;

  //simulated yaw angle, for simulation.
  private SimDouble m_simYawAngle;

  public AprilTagFieldLayout aprilTagFieldLayout;

  public PhotonPoseEstimator photonFrontPoseEstimator;

  private double m_fusedHeadingOffset = 0;

  //public PhotonPoseEstimator photonrearPoseEstimator;

  public PhotonCamera frontCamera = new PhotonCamera("frontAprilTagCamera");

  /** Creates a new IndexSensors. */
  public Sensors() {

    // instantiate navx over MXP
    try {
      navX = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP: " + ex.getMessage(), true);
    }

    // instantiate navx over USB
    // try {
    //   armNavX = new AHRS(SerialPort.Port.kUSB);
    // } catch (RuntimeException ex) {
    //   DriverStation.reportError("Error instantiating armNavX-USB: " + ex.getMessage(), true);
    // }

     // sends WPI provided AprilTag locations to the PhotonVision field layout object
    try {
      this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
        AprilTagFields.k2023ChargedUp.m_resourceFile);

      // Construct PhotonPoseEstimator
      this.photonFrontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, 
      frontCamera, visionConstants.kRobotToFrontCamMeters);

      photonFrontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
      
    } catch (IOException e) {
      DriverStation.reportError("Error loading AprilTagFieldLayout in Sensors:" + e.getMessage(), true);
      photonFrontPoseEstimator = null;
    }
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
  }


  // public double armNavxPitchdeg() {
  //   return -armNavX.getRoll();
  // }

  public Rotation2d navXRotation2d() {
    return navX.getRotation2d();
  }

  public double navXYawDeg() {
    return navX.getYaw();
  }

  public double wrappedNavXHeading() {
    return MathUtil.inputModulus(-navX.getYaw(), 0, 360);
  }

  public void setFusedHeadingOffset(double offset) {
    m_fusedHeadingOffset = offset;
  }

  public double NavXFusedHeading() {
    return MathUtil.inputModulus(navX.getFusedHeading() + 
      m_fusedHeadingOffset, 0, 360);
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

  public Transform2d navXdisplacement() {
    return new Transform2d(new Translation2d(
      navX.getDisplacementX(), navX.getDisplacementY()), 
        new Rotation2d());

  }

  public void resetNavxDisplacement() {
    navX.resetDisplacement();
    
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      if (photonFrontPoseEstimator == null) {
        return Optional.empty();
      }

    photonFrontPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonFrontPoseEstimator.update();
  }

}
