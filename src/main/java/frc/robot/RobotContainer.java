// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive2.auto.AutoMap;
import frc.robot.commands.swervedrive2.auto.GoToLoadingZone;
import frc.robot.commands.swervedrive2.auto.GoToLoadingZone.LOADING_SIDE;
import frc.robot.commands.swervedrive2.auto.GoToScoring;
import frc.robot.commands.swervedrive2.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive2.auto.PathBuilder;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();
  private SendableChooser<Double> spdLimit = new SendableChooser<>();

  // The robot's subsystems
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public static final Arm arm = new Arm();
  public static final Intake intake = new Intake();
  // private final Limelight vision = new Limelight(drivebase);

  private final AutoMap autoMap = new AutoMap(intake, arm);
  private final PathBuilder builder = new PathBuilder(drivebase, autoMap.getMap());

  private final CommandXboxController drv = new CommandXboxController(OIConstants.driverID);
  private final CommandXboxController op = new CommandXboxController(OIConstants.operatorID);
  private final CommandGenericHID btn = new CommandGenericHID(OIConstants.buttonsID);

  // Grid Selection Variables
  private int column = 0;
  private String level = "ArmHigh";
  private double limit = 0.75;

  // the default commands
  private final TeleopDrive closedFieldRel =
      new TeleopDrive(
          drivebase,
          () -> getLimitedSpeed(drv.getLeftY()),
          () -> getLimitedSpeed(drv.getLeftX()),
          () -> getDeadband(drv.getRightX(), OIConstants.radDeadband) * OIConstants.radLimiter,
          () -> true,
          false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the button bindings
    configureBindings();
    initializeChooser();
  }

  private void initializeChooser() {

    chooser.setDefaultOption(
        "Default Test",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "Test Path", new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));

    chooser.addOption(
        "3 Score T1",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "3 Score T1", new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));

    chooser.addOption(
        "1 Score + Dock T2",
        builder
            .getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "1 Score + Dock T2",
                    new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS)))
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0)));

    SmartDashboard.putData("CHOOSE", chooser);

    spdLimit.addOption("100%", 1.0);
    spdLimit.setDefaultOption("75%", 0.75);
    spdLimit.addOption("50%", 0.5);
    spdLimit.addOption("25%", 0.25);
    SmartDashboard.putData("Speed Limit", spdLimit);
  }

  private double getLimitedSpeed(double axis) {
    return limit * getDeadband(axis, OIConstants.xyDeadband);
  }

  private double getDeadband(double axis, double deadband) {
    if (Math.abs(axis) > deadband) {
      return axis;
    } else {
      return 0;
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {
    // Set up the normal drive control
    drivebase.setDefaultCommand(closedFieldRel);

    // This is the backup manual control of the Arm
    arm.setDefaultCommand(new RunCommand(() -> arm.setArmPower(-op.getRightY()), arm));

    // Left Bumper slows the drive way down for fine positioning
    drv.leftBumper().onTrue(Commands.runOnce(() -> limit = 0.2));
    drv.leftBumper().onFalse(Commands.runOnce(() -> limit = spdLimit.getSelected()));

    // Buttons automatically drive a corridor / charge station
    drv.x()
        .whileTrue(
            new ProxyCommand(() -> new GoToScoring(drivebase, POSITION.LEFT).getCommand())
                .alongWith(autoMap.getCommandInMap(level)));

    drv.a()
        .whileTrue(
            new ProxyCommand(() -> new GoToScoring(drivebase, POSITION.MIDDLE).getCommand())
                .alongWith(autoMap.getCommandInMap(level)));

    drv.b()
        .whileTrue(
            new ProxyCommand(() -> new GoToScoring(drivebase, POSITION.RIGHT).getCommand())
                .alongWith(autoMap.getCommandInMap(level)));

    drv.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.5)
        .whileTrue(
            new ProxyCommand(() -> new GoToLoadingZone(LOADING_SIDE.LEFT, drivebase).getCommand())
                .alongWith(autoMap.getCommandInMap("ArmHigh")));

    drv.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.5)
        .whileTrue(
            new ProxyCommand(() -> new GoToLoadingZone(LOADING_SIDE.RIGHT, drivebase).getCommand())
                .alongWith(autoMap.getCommandInMap("ArmHigh")));

    // Zero the Gyro, should only be used during practice
    drv.start().onTrue(new InstantCommand(drivebase::zeroGyro));

    // Teleop AutoBalance will test if better than manual
    drv.back()
        .whileTrue(
            Commands.run(
                    () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                    drivebase)
                .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0));

    // Manual Arm High
    op.y().onTrue(Commands.run(() -> arm.extendArmHigh(), arm).until(() -> arm.isAtSetpoint()));

    // Manual Arm Mid
    op.x().onTrue(Commands.run(() -> arm.extendArmMid(), arm).until(() -> arm.isAtSetpoint()));

    // Manual Arm Low
    op.a().onTrue(Commands.run(() -> arm.extendArmLow(), arm).until(() -> arm.isAtSetpoint()));

    // Stow Arm
    op.b().onTrue(Commands.run(() -> arm.stowArm(), arm).until(() -> arm.isAtSetpoint()));

    // While left bumper is pressed intake the cone then minimal power to hold it
    op.leftBumper().whileTrue(Commands.runOnce(() -> intake.intakeCone(), intake));
    op.leftBumper().onFalse(Commands.runOnce(() -> intake.holdCone(), intake));

    // While right bumper is pressed intake the cube then minimal power to hold it
    op.rightBumper().whileTrue(Commands.runOnce(() -> intake.intakeCube(), intake));
    op.rightBumper().onFalse(Commands.runOnce(() -> intake.holdCube(), intake));

    // Zero Arm Encoder shouldn't be needed unless turned on without arm stowed.
    op.start().onTrue(Commands.runOnce(() -> arm.zeroArm(), arm));

    // Button Board setting the level and column to be placed
    btn.button(1).onTrue(Commands.runOnce(() -> level = "ArmLow"));
    btn.button(2).onTrue(Commands.runOnce(() -> level = "ArmMid"));
    btn.button(3).onTrue(Commands.runOnce(() -> level = "ArmHigh"));
    btn.button(4).onTrue(Commands.runOnce(() -> column = 1));
    btn.button(5).onTrue(Commands.runOnce(() -> column = 2));
    btn.button(6).onTrue(Commands.runOnce(() -> column = 3));
    btn.button(7).onTrue(Commands.runOnce(() -> column = 4));
    btn.button(8).onTrue(Commands.runOnce(() -> column = 5));
    btn.button(9).onTrue(Commands.runOnce(() -> column = 6));
    btn.button(10).onTrue(Commands.runOnce(() -> column = 7));
    btn.button(11).onTrue(Commands.runOnce(() -> column = 8));
    btn.button(12).onTrue(Commands.runOnce(() -> column = 9));
    btn.button(13).whileTrue(Commands.runOnce(() -> intake.intakeCone(), intake));
    btn.button(13).onFalse(Commands.runOnce(() -> intake.holdCone(), intake));
    btn.button(14).whileTrue(Commands.runOnce(() -> intake.intakeCube(), intake));
    btn.button(14).onFalse(Commands.runOnce(() -> intake.holdCube(), intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    return chooser.getSelected();
  }
}
