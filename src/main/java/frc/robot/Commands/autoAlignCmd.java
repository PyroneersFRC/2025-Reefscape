package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.robot;



public class autoAlignCmd extends Command{
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_vision;
    private final Pose3d m_targetPose3d;

    public autoAlignCmd(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose3d targetPose3d){
        m_driveSubsystem = driveSubsystem;
        m_vision = visionSubsystem;
        m_targetPose3d = targetPose3d;
    }

    
    @Override
    public void initialize(){
        

    }
    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;

    }
}