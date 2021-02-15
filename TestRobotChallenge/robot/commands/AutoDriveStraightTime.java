/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveStraightTime extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double speed;
  private double duration;
  
  private double startTime = 0.0;
  private int counter = 4;
  
  /**
   * Creates a new AutoDriveStraightTime.
   */
  public AutoDriveStraightTime(double speedIn, double timeIn) {
    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;
    this.duration = timeIn;   // in seconds

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();   // get start time
    System.out.println(MessageFormat.format("**Started {0}", this.getName()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      m_driveTrain.doTankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putString("Auto Command", "Finished AutoDriveStraightTime");
    System.out.println(MessageFormat.format("**Ended {0}", this.getName()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      double elapsedTime = Timer.getFPGATimestamp() - startTime;    // get elapsed time
    if (counter++ % 5 == 0) { System.out.println("**AutoDriveStraightTime  elapsed: "+String.format("%.3f", elapsedTime)+" duration: "+duration); }
    return (elapsedTime >= duration);   // check if time to end
  }
}
