/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
// https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/gyrodrivecommands

/** A command that will turn the robot to the specified angle. */
public class AutoTurnToAnglePIDCmd extends PIDCommand {

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngle The angle to turn to
   * @param driveTrain The drive subsystem to use
   */
  public AutoTurnToAnglePIDCmd(double targetAngle, DriveTrain driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        // This should return the measurement
        () -> driveTrain.getHeadingAngle(),
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // send output to robot
        output -> driveTrain.doArcadeDrive(0, output),
        // Require the driveTrain
        driveTrain
        );

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);

        // Configure additional PID options by calling `getController` here.
        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);

        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
  }
}
