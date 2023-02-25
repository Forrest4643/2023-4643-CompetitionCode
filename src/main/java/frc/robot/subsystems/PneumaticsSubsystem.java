//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import frc.robot.Constants.pConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
    Compressor phCompressor = new Compressor(pConstants.compressorID, PneumaticsModuleType.REVPH);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pswitch Value", phCompressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Compressor Enabled?", phCompressor.isEnabled());
        SmartDashboard.putNumber("Compressor Current", phCompressor.getCurrent());
    }

    public boolean getPswitch() {
        return phCompressor.getPressureSwitchValue();
    }

    public boolean getCompressorStatus() {
        return phCompressor.isEnabled();
    }

    public double getCompressorCurrent() {
        return phCompressor.getCurrent();
    }

    public void compOff() {
        phCompressor.disable();
    }

    public void compOn() {
        phCompressor.enableDigital();
    }
}
