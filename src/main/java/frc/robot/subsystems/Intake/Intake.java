// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;

  private enum gamepiece {
    cone(1),
    cube(-1),
    nothing(0);

    private int numVal;

    gamepiece(int numVal) {
      this.numVal = numVal;
    }

    public int getNumVal() {
      return numVal;
    }
  }

  private gamepiece piece = gamepiece.cone;

  /** How many amps the intake can use while picking up cube */
  static final int CUBE_CURRENT_LIMIT_A = 25;

  /** How many amps the intake can use while picking up cone */
  static final int CONE_CURRENT_LIMIT_A = 35;

  /** How many amps the intake can use while holding */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /** Percent output for intaking cube */
  static final double CUBE_OUTPUT_POWER = 0.5;

  /** Percent output for intaking cone */
  static final double CONE_OUTPUT_POWER = 0.8;

  /** Percent output for holding */
  static final double INTAKE_HOLD_POWER = 0.07;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(7, MotorType.kBrushless);
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCone() {
    piece = gamepiece.cone;
  }

  public void setCube() {
    piece = gamepiece.cube;
  }

  public void intakeCube() {
    setIntakeMotor(CUBE_OUTPUT_POWER, CUBE_CURRENT_LIMIT_A);
  }

  public void intakeCone() {
    setIntakeMotor(-CONE_OUTPUT_POWER, CONE_CURRENT_LIMIT_A);
  }

  public void outtakeCube() {
    setIntakeMotor(-CUBE_OUTPUT_POWER, CUBE_CURRENT_LIMIT_A);
  }

  public void outtakeCone() {
    setIntakeMotor(CONE_OUTPUT_POWER, CONE_CURRENT_LIMIT_A);
  }

  public void holdCube() {
    setIntakeMotor(INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public void holdCone() {
    setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public void pickup() {
    if (piece.getNumVal() > 0) {
      setIntakeMotor(CUBE_OUTPUT_POWER, CUBE_CURRENT_LIMIT_A);
    } else {
      setIntakeMotor(-CONE_OUTPUT_POWER, CONE_CURRENT_LIMIT_A);
    }
  }

  public void place() {
    if (piece.getNumVal() > 0) {
      setIntakeMotor(-CUBE_OUTPUT_POWER, CUBE_CURRENT_LIMIT_A);
    } else {
      setIntakeMotor(CONE_OUTPUT_POWER, CONE_CURRENT_LIMIT_A);
    }
  }

  public void stop() {
    setIntakeMotor(0, 0);
  }

  private void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
  }
}
