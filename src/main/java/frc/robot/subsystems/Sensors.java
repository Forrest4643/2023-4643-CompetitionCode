// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors extends SubsystemBase {

  //creating AHRS device "navX"
  AHRS navX;

  //defining onboard I2C port
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //defining I2C port on NavX
  private final I2C.Port mXpPorti2c = I2C.Port.kMXP;
  //defining top and bottom REV V3 color sensors
  private final ColorSensorV3 m_topV3ColorSensor = new ColorSensorV3(i2cPort);
  private final ColorSensorV3 m_bottomV3ColorSensor = new ColorSensorV3(mXpPorti2c);

  //simulated yaw angle, for simulation.
  SimDouble m_simYawAngle;

  //creating color objects for the top and bottom color sensors
  Color m_topColor;

  Color m_bottomColor;

  /** Creates a new IndexSensors. */
  public Sensors() {

    // instantiate navx over USB
    try {
      navX = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-USB: " + ex.getMessage(), true);
    }

  }

  @Override
  public void periodic() {

    m_topColor = m_topV3ColorSensor.getColor();

    m_bottomColor = m_bottomV3ColorSensor.getColor();

    //sending info for debugging to the SmartDashboard
    SmartDashboard.putNumber("topProx", m_topV3ColorSensor.getProximity());

    SmartDashboard.putNumber("bottomProx", m_bottomV3ColorSensor.getProximity());

    SmartDashboard.putNumber("topRedValue", m_topColor.red);
    SmartDashboard.putNumber("topBlueValue", m_topColor.blue);

    SmartDashboard.putNumber("bottomRedValue", m_bottomColor.red);
    SmartDashboard.putNumber("bottomBlueValue", m_bottomColor.blue);

    SmartDashboard.putNumber("Yaw", navXYaw());
    SmartDashboard.putNumber("Pitch", navXPitch());
    SmartDashboard.putNumber("Roll", navXRoll());

  }

  @Override
  public void simulationPeriodic() {
    //getting simulated device handle for navX
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[1]");
    //setting simulated yaw angle to the JNI angle
    m_simYawAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  }

  public Rotation2d navXRotation2d() {
    return navX.getRotation2d();
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

  public boolean topV3ColorSensor() {
    return (m_topV3ColorSensor.getProximity() > IndexerConstants.topThresh);
  }

  public boolean bottomV3ColorSensor() {
    return (m_bottomV3ColorSensor.getProximity() > IndexerConstants.bottomThresh);
  }

  public boolean topCargoBlue() {
    if (m_topColor.blue >= IndexerConstants.blueThresh) {
      return true;
    } else {
      return false;
    }
  }

  public boolean bottomCargoBlue() {
    if (m_bottomColor.blue >= IndexerConstants.blueThresh) {
      return true;
    } else {
      return false;
    }
  }
}
