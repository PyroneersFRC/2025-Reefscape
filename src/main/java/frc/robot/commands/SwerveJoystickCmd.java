package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.robot;
import frc.robot.Constants.xboxConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveJoystickCmd extends Command{

    private final DriveSubsystem DriveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    public final SlewRateLimiter xLimiter,yLimiter,turnLimiter;
    
    
    public SwerveJoystickCmd(DriveSubsystem driveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.DriveSubsystem = driveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
        this.turnLimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
        addRequirements(driveSubsystem);
        }

    

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > xboxConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(xSpeed) > xboxConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > xboxConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turnLimiter.calculate(turningSpeed);
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
