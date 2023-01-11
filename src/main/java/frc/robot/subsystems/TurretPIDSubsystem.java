// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;

public class TurretPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
  private final Sensors m_sensors;
  
  private final VisionSubsystem m_visionsubsystem;
  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem(VisionSubsystem m_visionsubsystem, Sensors m_sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));

        getController().setTolerance(TurretConstants.tolerance);
        turretMotor.setInverted(true);
        turretMotor.setIdleMode(IdleMode.kBrake);
        turretEncoder.setPositionConversionFactor(TurretConstants.turretTicksToDegrees);

        this.m_visionsubsystem = m_visionsubsystem;
        this.m_sensors = m_sensors;
  }

  public void setMotor(double speed) {
    
    double limitedOutput;
    //System.out.println("Pos: " + turretPositionDEG());
    if (turretPositionDEG() >= TurretConstants.turretForwardLimit) {
      limitedOutput = MathUtil.clamp(speed, -1, 0);
      System.out.println("TRT-FWD-LIMIT");
    } else if (turretPositionDEG() <= TurretConstants.turretReverseLimit) {
      limitedOutput = MathUtil.clamp(speed, 0, 1);
      System.out.println("TRT-REV-LIMIT");
    } else {
      limitedOutput = speed;
    }    
    //System.out.println("Spd: " + limitedOutput);

    turretMotor.set(limitedOutput);
    System.out.println("turretLimOutput: " + limitedOutput);
  }

  public void neg() {
    setMotor(-.5);
  }

  public void pos() {
    setMotor(.5);
  }
  
  @Override
  public void useOutput(double output, double setpoint) {
    setMotor(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return -m_visionsubsystem.getTargetYaw();
  }

  public double turretPositionDEG() {
    SmartDashboard.putNumber("turretPositionDEG", turretEncoder.getPosition());
    return turretEncoder.getPosition();
  }

}

