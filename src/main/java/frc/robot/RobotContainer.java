package frc.robot;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.xboxConstants;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {


  private final DriveSubsystem swerveSubsystem = new DriveSubsystem();

  private final Joystick driverJoytick = new Joystick(Constants.xboxConstants.kDriverControllerPort);

  public RobotContainer() {
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoytick.getRawAxis(Constants.xboxConstants.kDriverYAxis),
              () -> driverJoytick.getRawAxis(Constants.xboxConstants.kDriverXAxis),
              () -> driverJoytick.getRawAxis(Constants.xboxConstants.kDriverRotAxis),
              () -> !driverJoytick.getRawButton(Constants.xboxConstants.kDriverFieldOrientedButtonIdx)));

      // configureButtonBindings();
  }

  // private void configureButtonBindings() {
  //     new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  // }

  public Command getAutonomousCommand() {
    return null;
  }
}