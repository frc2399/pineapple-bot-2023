package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveTrain;
import java.util.function.Supplier;

public class CurvatureDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;
    private final Supplier<Boolean> quickTurn;

    /* This command does this (fill in)... */
    public CurvatureDriveCmd(DriveTrain driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction, Supplier<Boolean> quickTurn) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        this.quickTurn = quickTurn;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CurvatureDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed;
        double realTimeTurn;
        double right;
        double left;

        realTimeSpeed = speedFunction.get();
        realTimeTurn = -turnFunction.get();

        //Multiplied by realTimeSpeed to make turn speed proportional to straight speed
        //Speed and turn proportional so arc remains the same when the speed changes
        if(quickTurn.get() == true)
        {
            left = realTimeSpeed - realTimeTurn;
            right = realTimeSpeed + realTimeTurn;
        }
        else
        {
            left = realTimeSpeed - realTimeTurn * Math.abs(realTimeSpeed);
            right = realTimeSpeed + realTimeTurn * Math.abs(realTimeSpeed);
        }

        //Makes it so the ratio between speed and turn are still the same if turn * speed is >1
        double maxValue = Math.max(Math.max(Math.abs(left), Math.abs(right)), 1);
        left /= maxValue;
        right /= maxValue;
        this.driveSubsystem.setMotors(left, right);   
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("CurvatureDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}