//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    public final CANSparkMax Front = new CANSparkMax(IntakeConstants.frontID, MotorType.kBrushless);
    public final CANSparkMax Rear = new CANSparkMax(IntakeConstants.rearID, MotorType.kBrushless);

    private RelativeEncoder fEncoder = Front.getEncoder();
    private RelativeEncoder rEncoder = Rear.getEncoder();

    public void frontWheelsOn() {
        Front.set(1);
    }

    public void frontWheelsOff() {
        Front.set(0);
    }

    public void frontWheelsReverse() {
        Front.set(-1);
    }

    public double getFrontIntakeSpeed() {
        return fEncoder.getVelocity();
    }

    public double frontIntakeCurrent() {
        return Front.getOutputCurrent();
    }

    public void rearWheelsOn() {
        Rear.set(1);
    }

    public void rearWheelsOff() {
        Rear.set(0);
    }

    public void rearWheelsReverse() {
        Rear.set(-1);
    }

    public double getRearIntakeSpeed() {
        return rEncoder.getVelocity();
        
    }

    public double rearIntakeCurrent() {
        return Rear.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontIntakeRPM", fEncoder.getVelocity());
        SmartDashboard.putNumber("rearIntakeRPM", rEncoder.getVelocity());
    }

}
