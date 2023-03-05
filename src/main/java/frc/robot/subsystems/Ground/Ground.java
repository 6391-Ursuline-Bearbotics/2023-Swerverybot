// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Ground;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ground extends SubsystemBase {
  private final WPI_TalonFX armMotor;
  private final CANSparkMax intakeMotor;
  TalonFXConfiguration config = new TalonFXConfiguration();
  /** How many amps the arm motor can use. */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /** Percent output to run the arm up/down at */
  static final double ARM_OUTPUT_POWER = 0.4;

  /** Time to extend or retract arm in auto */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /** Stow position in Falcon units */
  static final double STOW = 0.0;

  /** Extend position in Falcon units */
  static final double EXTEND = 68000.0;

  /** Proportional term of PID for arm position control */
  static final double kP = 0.1;

  /** Threshold to determine how close we need to be to our position target in encoder ticks */
  static final double ERROR_THRESHOLD = 100;

  /** How many amps the intake can use while picking up cube */
  static final int CUBE_CURRENT_LIMIT_A = 25;

  /** Percent output for intaking cube */
  static final double CUBE_OUTPUT_POWER = 0.5;

  private int armCounter = 0;

  /** Creates a new Ground Intake. */
  public Ground() {
    armMotor = new WPI_TalonFX(4);
    armMotor.configFactoryDefault();
    armMotor.setInverted(false);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configVoltageCompSaturation(12.0);
    armMotor.enableVoltageCompensation(true);
    config.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true, ARM_CURRENT_LIMIT_A, ARM_CURRENT_LIMIT_A + 10, .1);
    config.slot0.kP = kP;
    config.closedloopRamp = 0.25;
    config.openloopRamp = 0.25;
    config.peakOutputForward = 0.3;
    config.peakOutputReverse = -0.3;
    armMotor.configAllSettings(config);
    intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {}

  public void stowArm() {
    setArmPosition(STOW);
  }

  public void extendArm() {
    setArmPosition(EXTEND);
  }

  public void stopArm() {
    armMotor.set(0);
  }

  public boolean isAtSetpoint() {
    var error = armMotor.getClosedLoopError();
    var noErr = Math.abs(error) < ERROR_THRESHOLD;

    if (noErr) {
      armCounter++;
    } else {
      armCounter = 0;
    }
    return armCounter > 4;
  }

  public void setArmPower(double percent) {
    armMotor.set(percent);
  }

  private void setArmPosition(double position) {
    armMotor.set(TalonFXControlMode.Position, position);
  }

  public void zeroArm() {
    armMotor.setSelectedSensorPosition(0);
  }

  public double getArmPosition() {
    var pos = armMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("Arm Position", pos);
    return pos;
  }

  public void intakeCube() {
    setIntakeMotor(CUBE_OUTPUT_POWER, CUBE_CURRENT_LIMIT_A);
  }

  public void outtakeCube() {
    setIntakeMotor(-CUBE_OUTPUT_POWER, CUBE_CURRENT_LIMIT_A);
  }

  public void stopIntake() {
    setIntakeMotor(0, 0);
  }

  public void deployGround() {
    intakeCube();
    extendArm();
  }

  public void retractGround() {
    stopIntake();
    stowArm();
  }

  private void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
  }
}
