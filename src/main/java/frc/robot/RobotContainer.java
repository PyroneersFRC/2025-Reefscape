package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.xboxConstants;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {


  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.xboxConstants.kDriverControllerPort);

  // public RobotContainer() {
  //     swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
  //             swerveSubsystem,
  //             () -> -driverJoytick.getRawAxis(Constants.xboxConstants.kDriverYAxis),
  //             () -> driverJoytick.getRawAxis(Constants.xboxConstants.kDriverXAxis),
  //             () -> driverJoytick.getRawAxis(Constants.xboxConstants.kDriverRotAxis),
  //             () -> !driverJoytick.getRawButton(Constants.xboxConstants.kDriverFieldOrientedButtonIdx)));


    public RobotContainer() {
      m_driveSubsystem.setDefaultCommand(Commands.run(
            () ->
                m_driveSubsystem.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), xboxConstants.kDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), xboxConstants.kDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), xboxConstants.kDeadband),
                      false),
            m_driveSubsystem));
    }
      // configureButtonBindings();

  // private void configureButtonBindings() {
  //     new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  // }

  public Command getAutonomousCommand() {
    return null;
  }
}