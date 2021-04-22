/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoGoToBallOne extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double speed;
  
  private double travelDistance = 0.0;
  private double startTime = 0.0;
  private double travelTime = 0.0;    // in seconds

  private AutoGoBluePath1 autoGoBluePath;  // commands to execute after this one
  private AutoGoRedPath1 autoGoRedPath;

  /**
   * Creates a new AutoGoToBallOne.
   */
  public AutoGoToBallOne(double speedIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;

    this.autoGoBluePath = new AutoGoBluePath1(this.m_driveTrain);  // create instances of next commands to do
    this.autoGoRedPath = new AutoGoRedPath1(this.m_driveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveTrain.resetEncoders();   // reset the encoders to 0 position
    startTime = Timer.getFPGATimestamp();   // get start time
    travelTime = 0.0;

    System.out.println(MessageFormat.format("**Started {0}  start time: {1}", this.getName(), String.format("%.3f", startTime)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_driveTrain.doTankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (travelTime < 4.0) {
      autoGoBluePath.schedule();  // based on time to 1st ball, do either Blue or Red path
      System.out.println("**scheduling AutoGoBluePath command");
    } else {
      autoGoRedPath.schedule();
      System.out.println("**scheduling AutoGoRedPath command");
    }

    //if (travelDistance < 12.0) {
    //    autoGoBluePath.schedule();  // based on distance to 1st ball, do either Blue or Red path
    //    System.out.println("**scheduling AutoGoBluePath command");
    //} else {
    //    autoGoRedPath.schedule();
    //    System.out.println("**scheduling AutoGoRedPath command");
    //}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (m_driveTrain.checkBallSwitch()) {         // check if limit switch tripped
      travelTime = Timer.getFPGATimestamp() - startTime;    // get elapsed time
      travelDistance = m_driveTrain.getLeftDistanceInch();
      System.out.println(MessageFormat.format("**Ending {0}  travel time: {1}  distance: {2}", this.getName(),String.format("%.3f",travelTime),String.format("%.3f",travelDistance)));
      return true;
    } else {
      return false;       // if limit switch not tripped means not at ball, so return false
    }
  }
}
