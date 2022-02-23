package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;


public class LimelightAim extends CommandBase {

    private static final double kP = 1;
    private static final double kI = 0.005;
    private static final double kD = 0;
    private static final double timeDiff = 0.02;

    private double xError;
    private double integralSumX;
    private double lastError;

    private DriveSubsystem m_drive;
    private LimeLightSubsystem m_limelight;

    public LimelightAim(DriveSubsystem driveSubsystem, LimeLightSubsystem limelight) {
        m_drive = driveSubsystem;
        m_limelight = limelight;
        addRequirements(driveSubsystem);
        addRequirements(limelight);
    }

    public void initialize() {
        xError = m_limelight.getX()*Math.PI/180;
    }

    public void execute() {
        xError = m_limelight.getX()*Math.PI/180;
        if (Math.abs(integralSumX) < 100) {
            integralSumX += xError;
        }

        double derivative = (xError - lastError) / timeDiff;

        double output = kP * xError + kI * integralSumX + kD * derivative;

        m_drive.arcadeDrive(0, output);

        lastError = xError;
        SmartDashboard.putNumber("output",output);
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            m_drive.arcadeDrive(0, 0);
        }
    }

    public boolean isFinished() {
        if (Math.abs(xError) < 0.1){
            return true;
        }
        integralSumX = 0;
        return false;
    }
}
