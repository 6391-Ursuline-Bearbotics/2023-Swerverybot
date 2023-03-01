// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive2.auto.AutoMap;
import frc.robot.commands.swervedrive2.auto.GoToLoadingZone;
import frc.robot.commands.swervedrive2.auto.GoToLoadingZone.LOADING_SIDE;
import frc.robot.commands.swervedrive2.auto.GoToScoring;
import frc.robot.commands.swervedrive2.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive2.auto.GoToScoring.SCORING_SIDE;
import frc.robot.commands.swervedrive2.auto.PathBuilder;
import frc.robot.commands.swervedrive2.auto.TeleopBuilder;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.Vision.Limelight;
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
  private final Limelight vision = new Limelight(drivebase);
  public final LEDSubsystem led = new LEDSubsystem();

  private final AutoMap autoMap = new AutoMap(intake, arm);
  private final PathBuilder builder = new PathBuilder(drivebase, autoMap.getMap());
  private final TeleopBuilder teleopBuilder = new TeleopBuilder(drivebase, autoMap.getMap());

  private final CommandXboxController drv = new CommandXboxController(OIConstants.driverID);
  private final CommandXboxController op = new CommandXboxController(OIConstants.operatorID);
  private final CommandGenericHID btn = new CommandGenericHID(OIConstants.buttonsID);

  // Grid Selection Variables
  private int column = 1;
  private String level = "ArmHigh";
  private double limit = 0.75;
  private Alliance color = Alliance.Invalid;

  // the default commands
  private final TeleopDrive closedFieldRel =
      new TeleopDrive(
          drivebase,
          () -> getLimitedSpeed(-drv.getLeftY()),
          () -> getLimitedSpeed(-drv.getLeftX()),
          () -> getDeadband(-drv.getRightX(), OIConstants.radDeadband) * OIConstants.radLimiter,
          () -> true,
          false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the button bindings
    configureBindings();
  }

  private void initializeChooser() {

    chooser.setDefaultOption("Default Test", teleopBuilder.getSwerveCommand("ConeStationRail"));

    chooser.addOption(
        "Cube Mobility Dock",
        builder
            .getSwerveCommand("CubeMobilityDock")
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0)));

    chooser.addOption(
        "Cone Mobility Dock",
        builder
            .getSwerveCommand("ConeMobilityDock")
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0)));

    SmartDashboard.putData("CHOOSE", chooser);

    spdLimit.addOption("100%", 1.0);
    spdLimit.setDefaultOption("75%", 0.75);
    spdLimit.addOption("50%", 0.5);
    spdLimit.addOption("35%", 0.35);
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

  private LOADING_SIDE getLoadingSide(Boolean left) {
    if (color == Alliance.Blue) {
      if (left) {
        return LOADING_SIDE.RAIL;
      } else {
        return LOADING_SIDE.BARRIER;
      }
    } else {
      if (left) {
        return LOADING_SIDE.BARRIER;
      } else {
        return LOADING_SIDE.RAIL;
      }
    }
  }

  private String getStationPath(Boolean left) {
    if (color == Alliance.Blue) {
      if (left) {
        if (gamePiece()) {
          return "CubeStationRail";
        } else {
          return "ConeStationRail";
        }
      } else {
        if (gamePiece()) {
          return "CubeStationBarrier";
        } else {
          return "ConeStationBarrier";
        }
      }
    } else {
      if (left) {
        if (gamePiece()) {
          return "CubeStationBarrier";
        } else {
          return "ConeStationBarrier";
        }
      } else {
        if (gamePiece()) {
          return "CubeStationRail";
        } else {
          return "ConeStationRail";
        }
      }
    }
  }

  private SCORING_SIDE getCorridor(POSITION pos) {
    if (color == Alliance.Blue) {
      if (pos == POSITION.LEFT) {
        return SCORING_SIDE.BARRIER;
      } else if (pos == POSITION.MIDDLE) {
        return SCORING_SIDE.MIDDLE;
      } else {
        return SCORING_SIDE.BUMP;
      }
    } else {
      if (pos == POSITION.LEFT) {
        return SCORING_SIDE.BUMP;
      } else if (pos == POSITION.MIDDLE) {
        return SCORING_SIDE.MIDDLE;
      } else {
        return SCORING_SIDE.BARRIER;
      }
    }
  }

  public Boolean gamePiece() {
    if (column == 2 || column == 5 || column == 8 || level == "ArmLow") {
      return true;
    } else {
      return false;
    }
  }

  public void setAllianceColor(Alliance color) {
    this.color = color;
    vision.setAlliance(color);
    builder.loadAllPaths();
    teleopBuilder.loadPath("ConeStationRail");
    initializeChooser();
  }

  private void setColumn(int col) {
    if (color == Alliance.Blue) {
      column = col;
    } else if (color == Alliance.Red) {
      column = 10 - col;
    }
    SmartDashboard.putNumber("Column", column);
    SmartDashboard.putString("Level", level);
    if (gamePiece()) {
      led.setAll(Color.kPurple);
    } else {
      led.setAll(Color.kOrange);
    }
  }

  public void setSpeedLimit(double lim) {
    if (lim == 0.0) {
      limit = spdLimit.getSelected();
    } else {
      limit = lim;
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
    drv.leftBumper().onTrue(Commands.runOnce(() -> limit = 0.35));
    drv.leftBumper().onFalse(Commands.runOnce(() -> setSpeedLimit(0.0)));

    // Buttons automatically drive a corridor / charge station
    drv.x()
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToScoring(drivebase, getCorridor(POSITION.LEFT), column).getCommand())
                .andThen(autoMap.getCommandInMap(level)));

    drv.a()
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToScoring(drivebase, getCorridor(POSITION.MIDDLE), column)
                            .getCommand())
                .andThen(autoMap.getCommandInMap(level)));

    drv.b()
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToScoring(drivebase, getCorridor(POSITION.RIGHT), column)
                            .getCommand())
                .andThen(autoMap.getCommandInMap(level)));

    drv.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.5)
        .whileTrue(
            new ProxyCommand(
                    () -> new GoToLoadingZone(getLoadingSide(true), drivebase, color).getCommand())
                .andThen(() -> builder.getSwerveCommand(getStationPath(true))));

    drv.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.5)
        .whileTrue(
            new ProxyCommand(
                    () -> new GoToLoadingZone(getLoadingSide(false), drivebase, color).getCommand())
                .andThen(() -> builder.getSwerveCommand(getStationPath(false))));

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
    op.y().onTrue(autoMap.getCommandInMap("ArmHigh"));

    // Manual Arm Mid
    op.x().onTrue(autoMap.getCommandInMap("ArmMid"));

    // Manual Arm Low
    op.a().onTrue(autoMap.getCommandInMap("ArmLow"));

    // Stow Arm
    op.b().onTrue(autoMap.getCommandInMap("ArmStow"));

    // While left bumper is pressed intake the cone then minimal power to hold it
    op.leftBumper().whileTrue(Commands.runOnce(() -> intake.intakeCone(), intake));
    op.leftBumper().onFalse(Commands.runOnce(() -> intake.holdCone(), intake));

    // While right bumper is pressed intake the cube then minimal power to hold it
    op.rightBumper().whileTrue(Commands.runOnce(() -> intake.intakeCube(), intake));
    op.rightBumper().onFalse(Commands.runOnce(() -> intake.holdCube(), intake));

    // Zero Arm Encoder shouldn't be needed unless turned on without arm stowed.
    op.start().onTrue(Commands.runOnce(() -> arm.zeroArm(), arm));

    // Button Board setting the level and column to be placed

    btn.button(1).onTrue(Commands.runOnce(() -> setColumn(1)));
    btn.button(2).onTrue(Commands.runOnce(() -> setColumn(2)));
    btn.button(3).onTrue(Commands.runOnce(() -> setColumn(3)));
    btn.button(4).onTrue(Commands.runOnce(() -> setColumn(4)));
    btn.button(5).onTrue(Commands.runOnce(() -> setColumn(5)));
    btn.button(6).onTrue(Commands.runOnce(() -> setColumn(6)));
    btn.button(7).onTrue(Commands.runOnce(() -> setColumn(7)));
    btn.button(8).onTrue(Commands.runOnce(() -> setColumn(8)));
    btn.button(9).onTrue(Commands.runOnce(() -> setColumn(9)));
    btn.button(10).onTrue(Commands.runOnce(() -> level = "ArmLow"));
    btn.button(11).onTrue(Commands.runOnce(() -> level = "ArmMid"));
    btn.button(12).onTrue(Commands.runOnce(() -> level = "ArmHigh"));
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
