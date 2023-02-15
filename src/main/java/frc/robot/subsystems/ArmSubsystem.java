// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.aConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem {

  private final CANSparkMax armMotor = new CANSparkMax(aConstants.armID, MotorType.kBrushless);

  public RelativeEncoder armEncoder = armMotor.getEncoder();

  public ArmFeedforward armFFcontroller = new ArmFeedforward(0, 0.37, 3.63, 0.03);


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));

        armEncoder.setPositionConversionFactor(0.00537109374);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    armMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
