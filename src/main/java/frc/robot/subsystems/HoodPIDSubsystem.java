// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.HoodConstants;

public class HoodPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax hoodMotor = new CANSparkMax(HoodConstants.hoodID, MotorType.kBrushless);

  // ticks to inches conversion factor
  private final RelativeEncoder hoodEncoder = hoodMotor.getAlternateEncoder(
      SparkMaxAlternateEncoder.Type.kQuadrature,
      8192);

  /** Creates a new HoodSubsystem. */
  public HoodPIDSubsystem() {
    super(new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD));

    getController().setTolerance(HoodConstants.PIDtolerance);

    hoodMotor.setInverted(false);
    hoodEncoder.setInverted(false);
    hoodEncoder.setPositionConversionFactor(HoodConstants.conversionFactor);
    getController().disableContinuousInput();
   
    }

    public void hoodClosed() {
    getController().setSetpoint(74);  
    }

    public void hoodOpen() {
    getController().setSetpoint(52.5);  
    }
    public void positionUpdate() {
      SmartDashboard.putNumber("HoodPosition", getHoodPositionDEG());
    }

    public void zeroHood() {
      hoodEncoder.setPosition(0);
    }



  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  public double getHoodPositionDEG() {
    //encoder * hoodtravelIN/hoodtravelDEG + hood lowest position
    double DEG = (hoodEncoder.getPosition() * 4.65805632013) + 74.429;
    return DEG;
  }

  public double getHoodPositionIN() {
    return hoodEncoder.getPosition();
  }

  public void setHoodMotor(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    double limitedOutput;
    if (getHoodPositionDEG() <= HoodConstants.highAngleLimit) {
      limitedOutput = MathUtil.clamp(output, 0, 1);
      System.out.println("FWDLIMIT");
    } else if (getHoodPositionIN() >= HoodConstants.lowAngleLimit) {
      limitedOutput = MathUtil.clamp(output, -1, 0);
      System.out.println("REVLIMIT");
    } else {
      limitedOutput = output;
    }
    hoodMotor.set(MathUtil.clamp(limitedOutput, -.5, .5));

  }

  @Override
  protected double getMeasurement() {
    return getHoodPositionDEG();
  }

  
}
