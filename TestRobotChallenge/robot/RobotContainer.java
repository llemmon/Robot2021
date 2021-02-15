/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.AutoRunChallenge3;
import frc.robot.autonomous.PickBlueBalls;
import frc.robot.autonomous.PickRedBalls;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoDriveStraightUnits;
import frc.robot.commands.AutoSpinToAngle;
import frc.robot.commands.AutoTurnToAngle;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // *** Robot subsystems and commands are defined here
  public static final DriveTrain m_driveTrain = new DriveTrain();

  // ** Controllers
  private static final XboxController xController = new XboxController(0);
  //private static final Joystick auxController = new Joystick(1);

  private static Command autoGoStraightCommand;
  private static Command autoGoUnitsCommand;
  private static Command autoSpinCommand;
  private static Command autoPickBlueBalls;
  private static Command autoPickRedBalls;
  private static Command autoDoChallenge3;

  // ** set up autonomous chooser
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // assign default command for drive train
    m_driveTrain.setDefaultCommand(
          new DriveCommand(() -> xController.getY(Hand.kLeft), () -> xController.getY(Hand.kRight)));

    buildAutonomousCommands();    // go create autonomous commands

    // load autonomous commands in autochooser and put on dashboard
    autoChooser.setDefaultOption("Auto Drive", autoGoStraightCommand );
    autoChooser.addOption("Auto Blue Balls", autoPickBlueBalls );
    autoChooser.addOption("Auto Red Balls", autoPickRedBalls );
    autoChooser.addOption("Auto To Angle", autoSpinCommand );
    autoChooser.addOption("Auto Units", autoGoUnitsCommand );
    autoChooser.addOption("Auto Challenge3", autoDoChallenge3 );
    SmartDashboard.putData("Auto Choices", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //bind XBox controller buttons
    // start driveTrain when XBox buttonA pressed, stop driveTrain when released
    final JoystickButton xButtonA = new JoystickButton(xController, OIConstants.kXBoxButtonA);
    xButtonA.whenHeld(new AutoDriveStraightTime(0.55, 3.0));     // check .whenHeld() vs .whileHeld()
    //xButtonA.whileHeld(new AutoDriveStraightTime(0.55, 3.0));  // whileHeld() reschedules if finished, whenHeld() command one time only
    //xButtonA.whenPressed(new PickBlueBalls(m_driveTrain));

    // start driveTrain when XBox buttonB pressed, stop driveTrain when pressed again
    final JoystickButton xButtonB = new JoystickButton(xController, OIConstants.kXBoxButtonB);
    xButtonB.toggleWhenPressed(new StartEndCommand(() -> m_driveTrain.doTankDrive(0.55, 0.55), 
          () -> m_driveTrain.stop(), m_driveTrain));

    // turn to angle when XBox buttonY pressed
    final JoystickButton xButtonY = new JoystickButton(xController, OIConstants.kXBoxButtonY);
    xButtonY.whenPressed(new AutoSpinToAngle(45.0, 0.55))
          .whenReleased(() -> m_driveTrain.stop(), m_driveTrain);

    // spin to angle when XBox buttonX pressed
    final JoystickButton xButtonX = new JoystickButton(xController, OIConstants.kXBoxButtonX);
    xButtonX.whenPressed(new AutoTurnToAngle(45.0, 0.55))
          .whenReleased(() -> m_driveTrain.stop(), m_driveTrain);

    // bind XBox buttonX to a sequentialCommandGroup (do multiple commands in sequence)
    //final JoystickButton xButtonX = new JoystickButton(xController, OIConstants.kXBoxButtonX);
    //xButtonX.whenPressed(new SequentialCommandGroup(
    //            new AutoDriveStraightTime(0.55, 3.0),
    //            new WaitCommand(2.0),
    //            new InstantCommand(m_driveTrain::stop, m_driveTrain)
    // ));

    /**
    // bind Logitech controller buttons
    // start driveTrain when Logitech buttonA pressed, stop driveTrain when released
    final JoystickButton auxButtonA = new JoystickButton(auxController, OIConstants.kLogiTechButtonA);
    auxButtonA.whenPressed(new AutoDriveStraightTime(0.55, 3.0))
          .whenReleased(() -> m_driveTrain.stop(), m_driveTrain);

    // start driveTrain when Logitech buttonB pressed, stop driveTrain when released
    final JoystickButton auxButtonB = new JoystickButton(auxController, OIConstants.kXBoxButtonB);
    auxButtonB.whenPressed(() -> m_driveTrain.doTankDrive(0.55, 0.55), m_driveTrain)
          .whenReleased(() -> m_driveTrain.stop(), m_driveTrain);

    // stop driveTrain when Logitech buttonX pressed
    final JoystickButton auxButtonX = new JoystickButton(auxController, OIConstants.kLogiTechButtonX);
    auxButtonX.whenPressed(m_driveTrain::stop, m_driveTrain);
    */

    // bind Logitech buttonX to a sequentialCommandGroup (do multiple commands in sequence)
    //final JoystickButton auxButtonX = new JoystickButton(auxController, OIConstants.kLogiTechButtonX);
    //auxButtonX.whenPressed(new SequentialCommandGroup(
    //            new AutoDriveStraightTime(0.55, 3.0),
    //            new WaitCommand(2.0),
    //            new InstantCommand(m_driveTrain::stop, m_driveTrain)
    //  ));
  }

  private void buildAutonomousCommands() {

     // create autonomous command to drive forward
     autoGoStraightCommand = new AutoDriveStraightTime(0.45, 2.0);
     autoGoUnitsCommand = new AutoDriveStraightUnits(0.45, 2.0);
     autoSpinCommand = new AutoSpinToAngle(45, 0.40);

     autoPickBlueBalls = new PickBlueBalls(m_driveTrain);
     autoPickRedBalls = new PickRedBalls(m_driveTrain);
     autoDoChallenge3 = new AutoRunChallenge3(m_driveTrain);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // get Command to run in autonomous
    System.out.println("***getting Autonomous command");
    //Command autoSelected = autoChooser.getSelected();
    //return autoSelected;
    return autoGoUnitsCommand;
  }
}
