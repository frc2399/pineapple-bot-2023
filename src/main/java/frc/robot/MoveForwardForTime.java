// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveForwardForTime extends CommandBase {
  private double time;
  private double motorRightSpeed;
  private double motorLeftSpeed;
  private DriveTrain driveTrain;
  private Timer timer;

  /** Creates a new MoveForwardForTime. */
  public MoveForwardForTime(double time1, double motorRightSpeed1, double motorLeftSpeed1, DriveTrain driveTrain1) {
  time=time1;
  motorRightSpeed=motorRightSpeed1;
  motorLeftSpeed=motorLeftSpeed1;
  driveTrain=driveTrain1;
  timer=new Timer();
  //adds the specific subsystem to the command
  addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
driveTrain.setMotors(motorLeftSpeed, motorRightSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setMotors(0, 0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get()>time){
      return true;
    }
    return false;
  }
}
