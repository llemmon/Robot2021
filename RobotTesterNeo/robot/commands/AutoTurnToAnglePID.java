/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
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

  private double currentHeading;
  private double currentDiff;
  private boolean finished = false;
  private int counter = 1;

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
    finished = false;
    System.out.println("**starting AutoTurnToAnglePID target angle: " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation;
    currentHeading = m_driveTrain.getHeadingAngle();
    currentDiff = targetAngle - Math.abs(currentHeading);

    // if in range done, else do pid calculation
    if (currentDiff  >= -DriveConstants.kToleranceDegrees && currentDiff  <= DriveConstants.kToleranceDegrees) {
      rotation = 0;
      finished = true;
      System.out.println("**done - diff: "+String.format("%.3f", currentDiff)+" heading: "+String.format("%.3f", currentHeading)+" target: "+targetAngle); 

    } else {
      double pidValue = MathUtil.clamp(pid.calculate(currentHeading, targetAngle), -turnSpeed, turnSpeed);
      if (targetAngle >= 0) {
        rotation = pidValue;
      } else {
        rotation = pidValue * -1;
      }
      if (counter++ % 2 == 0) { System.out.println("**PID calc: "+String.format("%.3f", pidValue)+" curr / target: "+String.format("%.3f", currentHeading)+" ~ "+String.format("%.3f", targetAngle)); }
    }
    m_driveTrain.doArcadeDrive(0, rotation);  // turn using arcadeDrive(xSpeed, zRotation)
  }

  //public static double clampValue(double value, double min, double max) {
  //  return Math.max(min, Math.min(value, max));		// make sure within range
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("**ending AutoTurnToAnglePID command  heading: +"+ String.format("%.3f", currentHeading));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (currentDiff  >= -DriveConstants.kToleranceDegrees && currentDiff  <= DriveConstants.kToleranceDegrees);
    return finished;    // checked and set in execute()
  }
}
