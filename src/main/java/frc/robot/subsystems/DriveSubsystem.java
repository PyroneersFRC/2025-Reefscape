package frc.robot.subsystems;

import java.time.Period;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.CANids;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.robot;


public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(CANids.kFrontLeftDrivingCanId,CANids.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(CANids.kFrontRightDrivingCanId,CANids.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_rearLeft = new SwerveModule(CANids.kRearLeftDrivingCanId,CANids.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_rearRight = new SwerveModule(CANids.kRearRightDrivingCanId,CANids.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

    SlewRateLimiter xlimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
    SlewRateLimiter ylimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
    SlewRateLimiter rotationlimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);

    private final VisionSubsystem m_vision = new VisionSubsystem();
    

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);   // TODO we put random value
    private final SwerveDriveKinematics m_driveKinematics = robot.kDriveKinematics;
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_driveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

    
    public DriveSubsystem() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            // Configure AutoBuilder
            AutoBuilder.configure(
                this::getPose2d, 
                this::resetPose2d, 
                this::getCurrentSpeeds,
                this::driveRobotRelative, 
                new PPHolonomicDriveController(
                robot.kPIDDrive,
                robot.kPIDTurning
                ), config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                } , this);
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }
    public Command goToPose() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    0.1*robot.kPhysicalMaxSpeedMetersPerSecond,
                    0.1*robot.kTeleDriveAccelerationUnitsPerSecond)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(robot.kDriveKinematics);
    
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                m_vision.getRobotPoseTag(),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                m_vision.getTargetPose(),
                config);
    
        var thetaController =
            new ProfiledPIDController(
                0.4, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                m_vision::getRobotPoseTag, // Functional interface to feed supplier
                robot.kDriveKinematics,
    
                // Position controllers
                new PIDController(0.2, 0, 0),
                new PIDController(0.2, 0, 0),
                thetaController,
                this::setModuleStates,
                this);
    
        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.sequence(
            new InstantCommand(() -> this.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> this.drive(0, 0, 0, false)));
    }
    
        

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){
        //System.out.println("xSpeed: " + xSpeed + " ySpeed " + ySpeed + " rotation " + rotation);
        SmartDashboard.putNumber("DriveSubsystem/drive/xSpeed", xSpeed);
        SmartDashboard.putNumber("DriveSubsystem/drive/ySpeed", ySpeed);
        SmartDashboard.putNumber("DriveSubsystem/drive/rotation", rotation);
        SmartDashboard.putBoolean("DriveSubsystem/drive/fieldRelative", fieldRelative);

        double xSpeedDelivered = 0.15*xSpeed * robot.kPhysicalMaxSpeedMetersPerSecond;
        double ySpeedDelivered = 0.15*ySpeed * robot.kPhysicalMaxSpeedMetersPerSecond;
        double rotationDelivered = 0.15*rotation * robot.kPhysicalMaxAngularSpeedRadiansPerSecond;

        double xSpeedLimited = xlimiter.calculate(xSpeedDelivered);
        double ySpeedLimited = ylimiter.calculate(ySpeedDelivered);
        double rotationSpeedLimited = rotationlimiter.calculate(rotationDelivered);

        SmartDashboard.putNumber("DriveSubsystem/drive/xSpeed limited", xSpeedLimited);
        SmartDashboard.putNumber("DriveSubsystem/drive/ySpeed limited", ySpeedLimited);
        SmartDashboard.putNumber("DriveSubsystem/drive/rotation limited", rotationSpeedLimited);
       
        var swerveModuleStates = robot.kDriveKinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedLimited, ySpeedLimited, rotationSpeedLimited, Rotation2d.fromDegrees(-m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedLimited, ySpeedLimited, rotationSpeedLimited)
        );

        this.setModuleStates(swerveModuleStates);
    }

    @Override
    public void periodic(){
        //System.out.println(m_gyro.getAngle());
        SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
        m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    }
    public void zeroHeading(){
        m_gyro.reset();
    }

    public Command resetGyro(){
        return this.runOnce(this::zeroHeading);
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }
    public Rotation2d getRotation2dZero(){
        return Rotation2d.fromDegrees(0);
    }
    public Pose2d getPose2d(){
        return m_odometry.getPoseMeters();
    }
    public void resetPose2d(Pose2d pose2d){
        m_odometry.resetPose(pose2d);
    }


    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return m_driveKinematics.toChassisSpeeds(new SwerveModuleState[]{m_frontLeft.getState(),m_frontRight.getState(),m_rearLeft.getState(),m_rearRight.getState()});
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = m_driveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }

    public void stopModules() {
        m_rearLeft.stop();
        m_rearRight.stop();
        m_frontLeft.stop();
        m_frontRight.stop();

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, robot.kPhysicalMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
        
    }
}
