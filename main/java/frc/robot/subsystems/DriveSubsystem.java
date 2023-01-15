// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX gyroTalonSRX;
  private final WPI_PigeonIMU gyro;
  private final SwerveModule[] swerveModules;
  private ChassisSpeeds speeds;
  private SwerveModuleState[] desiredModuleStates;
  private SwerveDriveOdometry swerveOdometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyroTalonSRX =  new WPI_TalonSRX(Swerve.gyroID);
    gyro = new WPI_PigeonIMU(gyroTalonSRX);

    swerveOdometry = new SwerveDriveOdometry(Swerve.KINEMATICS, getGyroRotation2d(), getSwervePositions());
    speeds = new ChassisSpeeds();
    desiredModuleStates = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
    

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Swerve.Mod0.constants),
      new SwerveModule(1, Swerve.Mod1.constants),
      new SwerveModule(2, Swerve.Mod2.constants),
      new SwerveModule(3, Swerve.Mod3.constants),
    };
    
    initializeTelemetry();
    gyro.reset();
  }

  private GenericEntry gyroRotEntry;
  private GenericEntry gyroPitchEntry;
  private GenericEntry odometryXEntry;
  private GenericEntry odometryYEntry;
  private GenericEntry odometryDegEntry;
  private GenericEntry[] cancoderEntries;
  private GenericEntry[] integratedEntries;
  private GenericEntry[] velEntries;
  /**
   * initializes telemetry
   */
  private void initializeTelemetry() {
    ShuffleboardTab teleTab = Shuffleboard.getTab("Telemetry");
    ShuffleboardLayout gyroLayout = teleTab
      .getLayout("gyro", BuiltInLayouts.kList)
      .withSize(1, 3);
    gyroLayout.add("reset gyro position", new InstantCommand(() -> gyro.reset()));
    gyroRotEntry = gyroLayout.add("gyro rotation deg", gyro.getRotation2d().getDegrees()).getEntry();
    gyroPitchEntry = gyroLayout.add("gyro pitch deg", gyro.getPitch()).getEntry();

    ShuffleboardLayout odometryLayout = teleTab
      .getLayout("odometry", BuiltInLayouts.kList)
      .withSize(1, 3);
    odometryXEntry = odometryLayout.add("x", swerveOdometry.getPoseMeters().getX()).getEntry();
    odometryYEntry = odometryLayout.add("y", swerveOdometry.getPoseMeters().getY()).getEntry();
    odometryDegEntry = odometryLayout.add("deg", swerveOdometry.getPoseMeters().getRotation().getDegrees()).getEntry();

    cancoderEntries = new GenericEntry[4];
    integratedEntries = new GenericEntry[4];
    velEntries = new GenericEntry[4];
    for(SwerveModule module : swerveModules){
      ShuffleboardLayout encoderLayout = teleTab
        .getLayout("Module " + module.moduleNumber + " Encoders", BuiltInLayouts.kList)
        .withSize(1, 3);
      cancoderEntries[module.moduleNumber] = encoderLayout.add("Cancoder", module.getCancoderAngle().getDegrees()).getEntry();
      integratedEntries[module.moduleNumber] = encoderLayout.add("Integrated", module.getPosition().angle.getDegrees()).getEntry();
      velEntries[module.moduleNumber] = encoderLayout.add("Velocity", module.getState().speedMetersPerSecond).getEntry();    
    }
    
  }

  /**
   * updates telemetry of modules
   */
  public void updateModuleTelemetry() {
    for(SwerveModule module : swerveModules){ 
      cancoderEntries[module.moduleNumber].setDouble(module.getCancoderAngle().getDegrees());
      integratedEntries[module.moduleNumber].setDouble(module.getPosition().angle.getDegrees());
      velEntries[module.moduleNumber].setDouble(module.getState().speedMetersPerSecond);
    }
  }
  
  /**
   * @return Rotation2d of gyro
   */
  public Rotation2d getGyroRotation2d() {
    gyroRotEntry.setDouble(gyro.getRotation2d().getDegrees());
    gyroPitchEntry.setDouble(gyro.getPitch());
    return gyro.getRotation2d();
  }

    /**
     * resets gyro
     */
    public void resetGyro() {
      gyro.reset();
    }

  /**
   * 
   * @return Pose2d of the robot from odometry
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * reset odometry of the robot from a given pose
   * @param pose Pose2d that the robot is at
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroRotation2d(), getSwervePositions(), pose);
  }

  /**
   * reset modules to their absolute position
   */
  public void resetModulesToAbsolute() {
    for(SwerveModule modules : swerveModules){
      modules.resetToAbsolute();
  }
  }

  /**
   * drive from module states list
   * @param states list of SwerveModuleStates that correspond to the robot
   */
  public void setDesiredModuleStates(SwerveModuleState[] states) {
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(states[module.moduleNumber]);
    }
  }

  /**
   * update the odometry of the robot with current pose of the robot
   */
  public void updateRobotPose() {
    swerveOdometry.update(
      getGyroRotation2d(),
      getSwervePositions()
      );
    odometryXEntry.setDouble(swerveOdometry.getPoseMeters().getX());
    odometryYEntry.setDouble(swerveOdometry.getPoseMeters().getY());
    odometryDegEntry.setDouble(swerveOdometry.getPoseMeters().getRotation().getDegrees());
  }

  /**
   * drive robot from current module states in the class
   */
  public void drive() {
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredModuleStates[module.moduleNumber]);
    }
  }

  /**
   * drive a specific swerve module only by position (only front left)
   * @param i double from [-1, 1]
   */
  public void drivePosSpecificModule(double i) {
    swerveModules[0].setDrivePosition(i);
  }

  /**
   * set modude positions to a locked position with vel pid set to 0 to attempt to brake
   */
  public void driveFromStopped() {
    SwerveModuleState stopped = new SwerveModuleState(0, new Rotation2d(Math.PI/2));
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(stopped);
    }
  }

  /**
   * update speeds kinematics class 
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void setSpeeds(double rad, double vx, double vy) {
    speeds.omegaRadiansPerSecond = rad;
    speeds.vxMetersPerSecond = vx;
    speeds.vyMetersPerSecond = vy;
  }

  /**
   * update speeds from field relative setup
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void setSpeedsFieldRelative(double rad, double vx, double vy) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rad, getGyroRotation2d());
  }

  /**
   * update normal moduleStates
   */
  public void updateModuleStates() {
    desiredModuleStates = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
  }

  /**
   * update PID in the module substates from constants
   */
  public void updatePIDConfigs() {
    for (SwerveModule module : swerveModules) {
      module.updatePID();;
    }
  }

  /**
   * @return SwerveModuleState[] of all modules, calculated from drive velocity and cancoders
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule module : swerveModules){
        states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * @return SwerveModulePosition[] of all modules, calculated from drive position and cancoders
   */
  public SwerveModulePosition[] getSwervePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule module : swerveModules){
        positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }


  @Override
  public void periodic() {
    if (DriverStation.isDisabled()){
      resetModulesToAbsolute();
    }
    updateRobotPose();
    updateModuleTelemetry();
  }

}