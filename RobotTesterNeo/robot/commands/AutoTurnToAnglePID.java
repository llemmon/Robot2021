/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnToAnglePID extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double targetAngle;
  private double turnSpeed;
  private PIDController pid;

  private static final double kP = 0.03;
  private static final double kI = 0.00;
  private static final double kD = 0.00;
  //private static final double kF = 0.00;

  private static double currentHeading;
  private int counter = 4;

  /**
   * Creates a new AutoTurnToAnglePID.
   */
  public AutoTurnToAnglePID(double targetAngleIn, double turnSpeedIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.targetAngle = targetAngleIn;   // in degrees
    this.turnSpeed = turnSpeedIn;

    pid = new PIDController(kP, kI, kD);  // create PID object

    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveTrain.resetGyro();  // reset gyro to 0 heading
    pid.reset();              // clear internal state of PIDController
    pid.setTolerance(DriveConstants.kToleranceDegrees, 10.0);    // allow 2 degree tolerance around setpoint, targetAngle in this case
    System.out.println("**starting AutoTurnToAnglePID target angle: " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    currentHeading = m_driveTrain.getHeadingAngle();
    double pidValue = pid.calculate(currentHeading, targetAngle) / 100;
    //double pidValue1 = clampValue(pidValue, minOutputValue, maxOutputValue); // clamp pid value to valid range
    if (counter % 5 == 0) { System.out.println("**PID calc: "+String.format("%.3f", pidValue)+" curr / target: "+String.format("%.3f", currentHeading)+"~"+String.format("%.3f", targetAngle)); }

    double rotation;
    if (pidValue < 0) {
      rotation = turnSpeed * -1;
    } else {
      rotation = turnSpeed;
    }

    m_driveTrain.doArcadeDrive(0, rotation);  // turn using arcadeDrive(xSpeed, zRotation)
  }

  //public static double clampValue(double value, double min, double max) {
  //  return Math.max(min, Math.min(value, max));		// make sure within range
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("**ending AutoTurnToAnglePID command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pid.atSetpoint()) {   // see if time to quit
      System.out.println(MessageFormat.format("**Ending {0}, current heading: {1}", this.getName(), String.format("%.3f", currentHeading)));
      return true;
    }
    return false;
  }
}
