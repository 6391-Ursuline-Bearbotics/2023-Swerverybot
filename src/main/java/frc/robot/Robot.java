// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.TimeUnit;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private boolean isAutoDone;
  private RobotContainer robotContainer;
  private Command autonomousCommand;
  public static Robot instance;
  private Alliance ally = Alliance.Invalid;
  private double lastpos = 0;
  private Timer time = new Timer();

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
      TimeUnit.SECONDS.sleep(1);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    robotContainer = new RobotContainer();
    checkDSUpdate();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // Shuffleboard.update();
    // SmartDashboard.updateValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    checkDSUpdate();
  }

  @Override
  public void disabledPeriodic() {
    checkDSUpdate();
    if (ally == Alliance.Red) {
      robotContainer.led.setAll(Color.kRed);
    } else if (ally == Alliance.Blue) {
      robotContainer.led.setAll(Color.kBlue);
    } else {
      robotContainer.led.rainbow();
    }
  }

  @Override
  public void autonomousInit() {
    checkDSUpdate();
    robotContainer.drivebase.setMotorIdleMode(true);
    robotContainer.vision.useLimelight(false);
    robotContainer.arm.zeroArm();
    autonomousCommand = robotContainer.getAuto();
    System.out.println("AUTO START");
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousPeriodic() {
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)

    if (autonomousCommand != null && !isAutoDone) {
      autonomousCommand.schedule();
      isAutoDone = true;
      System.out.println("AUTO SCHEDULE");
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    isAutoDone = false;
    checkDSUpdate();
    robotContainer.drivebase.setMotorIdleMode(false);
    robotContainer.vision.useLimelight(true);
    robotContainer.intake.stop();
    robotContainer.ground.stopIntake();
    time.reset();
    time.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var pos = robotContainer.arm.getArmPosition();
    if (pos > 5000) {
      robotContainer.setSpeedLimit(0.3);
    } else if (pos < 5000 && lastpos > 5000) {
      robotContainer.setSpeedLimit(0.0);
    }
    lastpos = pos;

    // This is forcing garbage collection to avoid the std::bad_alloc problem
    if (time.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  void checkDSUpdate() {
    Alliance currentAlliance = DriverStation.getAlliance();

    // If we have data, and have a new alliance from last time
    if (DriverStation.isDSAttached() && currentAlliance != ally) {
      ally = currentAlliance;
      robotContainer.setAllianceColor(ally);
    }
  }
}
