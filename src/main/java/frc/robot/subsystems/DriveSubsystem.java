
package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CANids;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.robot;
import frc.robot.Constants.xboxConstants;


public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(CANids.kFrontLeftDrivingCanId,CANids.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(CANids.kFrontRightDrivingCanId,CANids.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_rearLeft = new SwerveModule(CANids.kRearLeftDrivingCanId,CANids.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_rearRight = new SwerveModule(CANids.kRearRightDrivingCanId,CANids.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

    SlewRateLimiter xlimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
    SlewRateLimiter ylimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
    SlewRateLimiter rotationlimiter = new SlewRateLimiter(40);


    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);   // TODO we put random value

    private final SwerveDriveKinematics m_driveKinematics = robot.kDriveKinematics;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_driveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle()), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

    private String SMART_DASHBOARD_PREFIX = "Drive Subsystem/";
    
    public DriveSubsystem() {
        pathplannerConfig();
    }
    
    private void pathplannerConfig(){
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
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
                    robot.kPhysicalMaxSpeedMetersPerSecond,
                    robot.kTeleDriveAccelerationUnitsPerSecond)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(robot.kDriveKinematics);
    
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,0,new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                //new Pose2d(getPose2d().getX()+1, getPose2d().getY(),getRotation2d().plus(new Rotation2d(Math.PI/2))),
                new Pose2d(1,0,new Rotation2d(0)),
                config);
    
        var thetaController =
            new ProfiledPIDController(
                0.4, 0, 0.2, new TrapezoidProfile.Constraints(0.5, 0.5));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                this::getPose2d, // Functional interface to feed supplier
                robot.kDriveKinematics,
    
                // Position controllers
                new PIDController(3, 0.35, 0),
                new PIDController(3, 0.35, 0),
                thetaController,
                this::setModuleStates,
                this);
    
        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.sequence(
            new InstantCommand(() -> this.resetOdometry(exampleTrajectory.getInitialPose())),
            new InstantCommand(() -> System.out.println("BEFORE TRAJECTORY"+System.currentTimeMillis())),
            swerveControllerCommand,
            new InstantCommand(() -> System.out.println("AFTER TRAJECTORY"+System.currentTimeMillis())),
            new InstantCommand(() -> this.drive(0, 0, 0, true)));
    }
    
    

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){
        xSpeed = xlimiter.calculate(xSpeed);
        ySpeed = ylimiter.calculate(ySpeed);
        rotation = rotationlimiter.calculate(rotation);
       
        var swerveModuleStates = robot.kDriveKinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(-m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );

        this.setModuleStates(swerveModuleStates);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "Gyro", m_gyro.getAngle());
        SmartDashboard.putString(SMART_DASHBOARD_PREFIX + "Pose", getPose2d().toString());

        m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()), new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    }


    public void zeroHeading(){
        m_gyro.reset();
        m_odometry.resetPose(getPose2d());
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
        return m_driveKinematics.toChassisSpeeds(new SwerveModuleState[]{
            m_frontLeft.getState(), 
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        });
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

    public Command driveWithJoystickCmd(CommandXboxController xboxController){
        return this.run(
                () ->
                    this.drive(
                        - robot.kPhysicalMaxSpeedMetersPerSecond * MathUtil.applyDeadband(
                            xboxController.getLeftY(), xboxConstants.kDeadband),
                        - robot.kPhysicalMaxSpeedMetersPerSecond * MathUtil.applyDeadband(
                            xboxController.getLeftX(), xboxConstants.kDeadband),
                        - robot.kPhysicalMaxAngularSpeedRadiansPerSecond * MathUtil.applyDeadband(
                            xboxController.getRightX(), xboxConstants.kDeadband),
                        true)
                );
    }

}