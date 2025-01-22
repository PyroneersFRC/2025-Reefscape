package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.robot;


public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(CANids.kFrontLeftTurningCanId,CANids.kFrontLeftTurningCanId);
    private final SwerveModule m_frontRight = new SwerveModule(CANids.kFrontRightTurningCanId,CANids.kFrontRightTurningCanId);
    private final SwerveModule m_RearLeft = new SwerveModule(CANids.kRearLeftTurningCanId,CANids.kRearLeftTurningCanId);
    private final SwerveModule m_RearRight = new SwerveModule(CANids.kRearRightTurningCanId,CANids.kRearRightTurningCanId);

    private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);   // TODO we put random value

    public DriveSubsystem() {
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch (Exception e){}
        }).start();
    }
        

    public void zeroHeading(){
        m_gyro.reset();
    }


    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }


    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }
    public void stopModules() {
        m_RearLeft.stop();
        m_RearRight.stop();
        m_frontLeft.stop();
        m_frontRight.stop();

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, robot.kPhysicalMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_RearLeft.setDesiredState(desiredStates[2]);
        m_RearRight.setDesiredState(desiredStates[3]);
    }


}
