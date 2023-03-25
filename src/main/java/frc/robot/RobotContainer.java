// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive2.auto.AutoMap;
import frc.robot.commands.swervedrive2.auto.GoToLoadingZone;
import frc.robot.commands.swervedrive2.auto.GoToScoring;
import frc.robot.commands.swervedrive2.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive2.auto.GoToScoring.SCORING_SIDE;
import frc.robot.commands.swervedrive2.auto.PathBuilder;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Ground.Ground;
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
  public final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final Arm arm = new Arm();
  public final Intake intake = new Intake();
  public final Limelight vision = new Limelight(drivebase);
  public final LEDSubsystem led = new LEDSubsystem();
  public final Ground ground = new Ground();

  private final AutoMap autoMap = new AutoMap(intake, arm, ground);
  private final PathBuilder builder = new PathBuilder(drivebase, autoMap.getMap());

  private final CommandXboxController drv = new CommandXboxController(OIConstants.driverID);
  private final CommandXboxController op = new CommandXboxController(OIConstants.operatorID);
  private final CommandGenericHID btn = new CommandGenericHID(OIConstants.buttonsID);

  // Grid Selection Variables
  private int column = 1;
  private String level = "ArmHigh";
  private double limit = 0.75;
  private double radLimiter = OIConstants.radLimiter;
  private Alliance color = Alliance.Invalid;

  // the default commands
  private final TeleopDrive closedFieldRel =
      new TeleopDrive(
          drivebase,
          () -> getLimitedSpeed(-drv.getLeftY()),
          () -> getLimitedSpeed(-drv.getLeftX()),
          () -> Math.pow(getDeadband(-drv.getRightX(), OIConstants.radDeadband), 3) * radLimiter,
          () -> true,
          false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the button bindings
    configureBindings();
    SmartDashboard.putBoolean("ArmHigh", level == "ArmHigh");
    SmartDashboard.putBoolean("ArmMid", level == "ArmMid");
    SmartDashboard.putBoolean("ArmLow", level == "ArmLow");
    SmartDashboard.putNumber("Column", column);
    SmartDashboard.putNumber("limit", limit);
  }

  private void initializeChooser() {
    chooser.setDefaultOption(
        "Cone Only",
        autoMap
            .getCommandInMap("IntakeHigh")
            .andThen(autoMap.getCommandInMap("OuttakeStow"))
            .andThen(Commands.runOnce(() -> drivebase.swerveDrive.setGyro(180.0))));
    chooser.addOption(
        "6 - Cone Engage",
        builder
            .getSwerveCommand("6 - Cone Engage")
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(
                        () ->
                            Math.abs(drivebase.getPlaneInclination().getDegrees())
                                < Auton.balanceLimitDeg)));
    chooser.addOption("3 - Cone 2 Piece", builder.getSwerveCommand("ConeBarrier2"));
    chooser.addOption("9 - Cone 2 Piece", builder.getSwerveCommand("ConeBump2"));
    chooser.addOption(
        "3 - Cone Mobility Loading", builder.getSwerveCommand("3 - Cone Mobility Loading"));
    chooser.addOption(
        "9 - Cone Mobility Loading",
        Commands.runOnce(() -> vision.useLimelight(true), vision)
            .andThen(builder.getSwerveCommand("9 - Cone Mobility Loading")));
    chooser.addOption(
        "3 - Cone Mobility Straight", builder.getSwerveCommand("3 - Cone Mobility Straight"));
    chooser.addOption(
        "6 - Cone Mobility Straight", builder.getSwerveCommand("6 - Cone Mobility Straight"));
    chooser.addOption(
        "9 - Cone Mobility Straight", builder.getSwerveCommand("9 - Cone Mobility Straight"));
    chooser.addOption(
        "6 - Cone Mobility Engage",
        builder
            .getSwerveCommand("ConeMobilityDock")
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(
                        () ->
                            Math.abs(drivebase.getPlaneInclination().getDegrees())
                                < Auton.balanceLimitDeg)));

    chooser.addOption(
        "5 - Cube Mobility Engage",
        builder
            .getSwerveCommand("CubeMobilityDock")
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(
                        () ->
                            Math.abs(drivebase.getPlaneInclination().getDegrees())
                                < Auton.balanceLimitDeg)));
    chooser.addOption("Do Nothing", Commands.runOnce(() -> drivebase.swerveDrive.setGyro(180.0)));
    SmartDashboard.putData("Auto choices", chooser);

    spdLimit.addOption("100%", 1.0);
    spdLimit.addOption("95%", 0.95);
    spdLimit.addOption("90%", 0.9);
    spdLimit.addOption("85%", 0.85);
    spdLimit.addOption("80%", 0.8);
    spdLimit.addOption("75%", 0.75);
    spdLimit.addOption("70%", 0.7);
    spdLimit.setDefaultOption("65%", 0.65);
    spdLimit.addOption("60%", 0.6);
    spdLimit.addOption("55%", 0.55);
    spdLimit.addOption("50%", 0.5);
    spdLimit.addOption("35%", 0.35);
    SmartDashboard.putData("Speed Limit", spdLimit);
  }

  private double getLimitedSpeed(double axis) {
    return limit * Math.pow(getDeadband(axis, OIConstants.xyDeadband), 3);
  }

  private double getDeadband(double axis, double deadband) {
    if (Math.abs(axis) > deadband) {
      return axis;
    } else {
      return 0;
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

  private String gamePieceString() {
    if (gamePiece()) {
      return "CubeGrab";
    } else {
      return "ConeGrab";
    }
  }

  public void setAllianceColor(Alliance color) {
    this.color = color;
    vision.setAlliance(color);
    builder.loadAllPaths();
    initializeChooser();
  }

  private void setColumn(int col) {
    if (color == Alliance.Blue) {
      column = col;
    } else if (color == Alliance.Red) {
      column = 10 - col;
    }
    SmartDashboard.putNumber("Column", col);
    if (gamePiece()) {
      led.setAll(Color.kPurple);
    } else {
      led.setAll(Color.kOrange);
    }
  }

  private void setLevel(String lev) {
    level = lev;
    SmartDashboard.putString("Level", level);
    SmartDashboard.putBoolean("ArmHigh", lev == "ArmHigh");
    SmartDashboard.putBoolean("ArmMid", lev == "ArmMid");
    SmartDashboard.putBoolean("ArmLow", lev == "ArmLow");
  }

  public void setSpeedLimit(double lim) {
    if (lim == 0.0) {
      limit = spdLimit.getSelected();
      radLimiter = OIConstants.radLimiter;
    } else {
      limit = lim;
      radLimiter = OIConstants.radSlow;
    }
    SmartDashboard.putNumber("limit", limit);
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
    arm.setDefaultCommand(Commands.run(() -> arm.setArmPower(-op.getRightY()), arm));

    // Manual Control of the ground intake
    ground.setDefaultCommand(Commands.run(() -> ground.setArmPower(-op.getLeftY()), ground));

    // Left Bumper slows the drive way down for fine positioning
    drv.leftBumper().onTrue(new ConditionalCommand(runOnce(() -> setSpeedLimit(0.10)), runOnce(() -> setSpeedLimit(0.0)), () -> limit > 0.4));

    // Right Bumper trusts the Limelight regardless of robot pose
    drv.rightBumper().onTrue(runOnce(() -> vision.trustLL(true)));
    drv.rightBumper().onFalse(runOnce(() -> vision.trustLL(false)));

    // Buttons automatically drive a corridor / charge station
    drv.x()
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToScoring(
                                drivebase, getCorridor(POSITION.LEFT), column, level, autoMap)
                            .getCommand())
                .andThen(new ProxyCommand(() -> autoMap.getCommandInMap(level))));

    drv.a()
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToScoring(
                                drivebase, getCorridor(POSITION.MIDDLE), column, level, autoMap)
                            .getCommand())
                .andThen(new ProxyCommand(() -> autoMap.getCommandInMap(level))));

    drv.b()
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToScoring(
                                drivebase, getCorridor(POSITION.RIGHT), column, level, autoMap)
                            .getCommand())
                .andThen(new ProxyCommand(() -> autoMap.getCommandInMap(level))));

    drv.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.5)
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToLoadingZone(true, drivebase, color, autoMap, gamePieceString())
                            .getCommand())
                .andThen(new ProxyCommand(autoMap.getCommandInMap("ArmHigh"))));
    drv.rightTrigger().onFalse(new ProxyCommand(runOnce(() -> intake.holdCone(), intake)));

    drv.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.5)
        .whileTrue(
            new ProxyCommand(
                    () ->
                        new GoToLoadingZone(false, drivebase, color, autoMap, gamePieceString())
                            .getCommand())
                .andThen(new ProxyCommand(autoMap.getCommandInMap("ArmHigh"))));
    drv.leftTrigger().onFalse(new ProxyCommand(runOnce(() -> intake.holdCone(), intake)));

    // Zero the Gyro, should only be used during practice
    drv.start().onTrue(runOnce(drivebase::zeroGyro));

    // Set drive to brake mode
    drv.back()
        .onTrue(
            runOnce(() -> drivebase.setMotorIdleMode(true))
                .andThen(Commands.runOnce(() -> drivebase.lockPose())));

    // Enable the Limelight
    drv.povUp().onTrue(runOnce(() -> vision.useLimelight(true)));

    // Disable the Limelight
    drv.povDown().onTrue(runOnce(() -> vision.useLimelight(false)));

    // Teleop AutoBalance test
    drv.povLeft()
        .onTrue(
            Commands.run(
                    () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                    drivebase)
                .until(
                    () ->
                        Math.abs(drivebase.getPlaneInclination().getDegrees())
                            < Auton.balanceLimitDeg)
                .alongWith(
                    Commands.run(
                        () ->
                            SmartDashboard.putNumber(
                                "Plane Angle", drivebase.getPlaneInclination().getDegrees()))));

    // Reflective fine alignment
    drv.povRight()
        .whileTrue(
            Commands.runOnce(
                () -> drivebase.drive(new Translation2d(0, vision.getFineAlign()), 0, false, false),
                drivebase));
    drv.povRight().onFalse(Commands.runOnce(() -> vision.fineAlignment(false)));

    // Manual Arm High
    op.y().onTrue(autoMap.getCommandInMap("ArmHigh"));

    // Manual Arm Mid
    op.x().onTrue(autoMap.getCommandInMap("ArmMid"));

    // Manual Arm Low
    op.a().onTrue(autoMap.getCommandInMap("ArmLow"));

    // Stow Arm
    op.b().onTrue(autoMap.getCommandInMap("ArmStow"));

    // While left bumper is pressed intake the cone then minimal power to hold it
    op.leftBumper().whileTrue(runOnce(() -> intake.intakeCone(), intake));
    op.leftBumper().onFalse(runOnce(() -> intake.holdCone(), intake));

    // While right bumper is pressed intake the cube then minimal power to hold it
    op.rightBumper().whileTrue(runOnce(() -> intake.intakeCube(), intake));
    op.rightBumper().onFalse(runOnce(() -> intake.holdCube(), intake));

    // Zero Arm Encoder shouldn't be needed unless turned on without arm stowed.
    op.start().onTrue(runOnce(() -> arm.zeroArm(), arm));

    // Zero Ground Arm Encoder
    op.back().onTrue(runOnce(() -> ground.zeroArm(), ground));

    op.povUp().onTrue(autoMap.getCommandInMap("GroundDeploy"));

    op.povDown().onTrue(autoMap.getCommandInMap("GroundStow"));

    op.povLeft().onTrue(autoMap.getCommandInMap("RetractIntake"));

    op.povRight().onTrue(autoMap.getCommandInMap("GroundOuttake"));

    // Right trigger trusts the Limelight regardless of robot pose
    op.rightTrigger().onTrue(runOnce(() -> vision.trustLL(true)));
    op.rightTrigger().onFalse(runOnce(() -> vision.trustLL(false)));

    // Button Board setting the level and column to be placed
    btn.button(1).onTrue(runOnce(() -> setColumn(1)));
    btn.button(2).onTrue(runOnce(() -> setColumn(2)));
    btn.button(3).onTrue(runOnce(() -> setColumn(3)));
    btn.button(4).onTrue(runOnce(() -> setColumn(4)));
    btn.button(5).onTrue(runOnce(() -> setColumn(5)));
    btn.button(6).onTrue(runOnce(() -> setColumn(6)));
    btn.button(7).onTrue(runOnce(() -> setColumn(7)));
    btn.button(8).onTrue(runOnce(() -> setColumn(8)));
    btn.button(9).onTrue(runOnce(() -> setColumn(9)));
    btn.button(10).onTrue(runOnce(() -> setLevel("ArmLow")));
    btn.button(11).onTrue(runOnce(() -> setLevel("ArmMid")));
    btn.button(12).onTrue(runOnce(() -> setLevel("ArmHigh")));
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
