// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.leftMotorID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.rightMotorID, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kS,
      ShooterConstants.kV);

  /** Creates a new ShooterSubsystem. */
  public ShooterPIDSubsystem() {

    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.PIDtolerance);

    rightEncoder.setVelocityConversionFactor(ShooterConstants.efficiencyConversion);
    leftEncoder.setVelocityConversionFactor(ShooterConstants.efficiencyConversion);
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

  }

  public void idleShooter() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public double leftAmps() {
    return leftMotor.getOutputCurrent();
  }

  public double rightAmps() {
    return rightMotor.getOutputCurrent();
  }

  public void backDrive() {
    setShooterVolts(-4);
  }

  public void setShooterVolts(double volts) {
    leftMotor.setVoltage(MathUtil.clamp(volts, -11, 11));
    rightMotor.setVoltage(MathUtil.clamp(volts, -11, 11));
  }

  public double getShooterRPM() {
    return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
      setShooterVolts(output + shooterFeedforward.calculate(setpoint));
      SmartDashboard.putNumber("shooterOutput", output);
      SmartDashboard.putNumber("shooterSetpoint", setpoint);


  }

  @Override
  protected double getMeasurement() {
    return getShooterRPM();
  }
}
