package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.robot;



public class autoAlignCmd extends Command{
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_vision;

    public autoAlignCmd(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_vision = visionSubsystem;
    }

    
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        // double xvspeed = (m_vision.getYaw())*0.1*robot.kPhysicalMaxSpeedMetersPerSecond;
        // double yvspeed = (m_vision.getTargetRange())*0.1*robot.kPhysicalMaxSpeedMetersPerSecond;

        // SmartDashboard.putNumber("xvspeed", m_vision.getYaw());
        // SmartDashboard.putNumber("yvspeed", m_vision.getTargetRange());

        // m_driveSubsystem.drive(m_vision.getYaw(), m_vision.getTargetRange(), 0, false);
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;

    }
}