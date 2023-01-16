// //Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

// package frc.robot.subsystems;

// import frc.robot.Constants.PNConstants;
// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class PneumaticsSubsystem extends SubsystemBase {
//     Compressor pcmCompressor = new Compressor(PNConstants.compressorID, PneumaticsModuleType.CTREPCM);

//     DoubleSolenoid frontIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PNConstants.frontForwardID,
//             PNConstants.frontReverseID);
//     DoubleSolenoid rearIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PNConstants.rearForwardID,
//             PNConstants.rearReverseID);

//     @Override
//     public void periodic() {
//         SmartDashboard.putBoolean("Pswitch Value", pcmCompressor.getPressureSwitchValue());
//         SmartDashboard.putBoolean("Compressor Enabled?", pcmCompressor.enabled());
//         SmartDashboard.putNumber("Compressor Current", pcmCompressor.getCurrent());
//     }

//     public boolean getPswitch() {
//         return pcmCompressor.getPressureSwitchValue();
//     }

//     public boolean getCompressorStatus() {
//         return pcmCompressor.enabled();
//     }

//     public double getCompressorCurrent() {
//         return pcmCompressor.getCurrent();
//     }

//     public boolean frontIntakeStatus() {
//         if (frontIntake.get() == Value.kForward) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public boolean rearIntakeStatus() {
//         if (rearIntake.get() == Value.kForward) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public void frontIntakeOpen() {
//         frontIntake.set(Value.kForward);
//     }

//     public void frontIntakeClosed() {
//         frontIntake.set(Value.kReverse);
//     }

//     public void rearIntakeOpen() {
//         rearIntake.set(Value.kForward);
//     }

//     public void rearIntakeClosed() {
//         rearIntake.set(Value.kReverse);
//     }

//     public void compOff() {
//         pcmCompressor.disable();
//     }

//     public void compOn() {
//         pcmCompressor.enableDigital();
//     }
// }
