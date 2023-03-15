package frc.robot.commands.swervedrive2.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Ground.Ground;
import frc.robot.subsystems.Intake.Intake;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoMap {
  private HashMap<String, Command> eventMap = new HashMap<>();
  private HashMap<String, Supplier<Command>> eventMapGetter = new HashMap<>();

  public AutoMap(Intake intake, Arm arm, Ground ground) {

    eventMapGetter.put(
        "ArmLow",
        () ->
            Commands.run(() -> arm.extendArmLow(), arm)
                .withTimeout(2)
                .andThen(() -> arm.stopArm(), arm));
    eventMapGetter.put(
        "ArmHigh",
        () ->
            Commands.run(() -> arm.extendArmHigh(), arm)
                .withTimeout(1.7)
                .andThen(() -> arm.stopArm(), arm));
    eventMapGetter.put(
        "ArmStow",
        () ->
            Commands.run(() -> arm.stowArm(), arm)
                .withTimeout(2)
                .andThen(() -> arm.stopArm(), arm));
    eventMapGetter.put(
        "ArmMid",
        () ->
            Commands.run(() -> arm.extendArmMid(), arm)
                .withTimeout(2)
                .andThen(() -> arm.stopArm(), arm));

    eventMapGetter.put(
        "CubeGrab", () -> Commands.run(() -> intake.intakeCube(), intake).withTimeout(5));

    eventMapGetter.put(
        "ConeGrab", () -> Commands.run(() -> intake.intakeCone(), intake).withTimeout(5));

    eventMapGetter.put(
        "IntakeCone", () -> Commands.run(() -> intake.intakeCone(), intake).withTimeout(0.25));

    eventMapGetter.put(
        "OuttakeCube", () -> Commands.run(() -> intake.outtakeCube(), intake).withTimeout(0.1));

    eventMapGetter.put(
        "OuttakeCone", () -> Commands.run(() -> intake.outtakeCone(), intake).withTimeout(0.1));

    eventMapGetter.put(
        "DropIntake", () -> Commands.run(() -> ground.deployGround(), ground).withTimeout(0.5));

    eventMapGetter.put(
        "RetractIntake", () -> Commands.run(() -> ground.retractGround(), ground).withTimeout(0.5));

    eventMapGetter.put("GroundIntake", () -> Commands.run(() -> ground.intakeCube(), ground));

    eventMapGetter.put("GroundOuttake", () -> Commands.run(() -> ground.outtakeCube(), ground));

    eventMapGetter.put(
        "IntakeHigh", () -> getCommandInMap("IntakeCone").alongWith(getCommandInMap("ArmHigh")));

    eventMapGetter.put(
        "OuttakeStow",
        () ->
            getCommandInMap("OuttakeCone")
                .alongWith(getCommandInMap("ArmStow"))
                .andThen(() -> intake.stop(), intake));

    eventMapGetter.put("GroundDeploy", () -> getCommandInMap("DropIntake"));

    eventMapGetter.put("GroundStow", () -> getCommandInMap("RetractIntake"));

    eventMapGetter.forEach(
        (key, val) -> {
          eventMap.put(key, val.get());
        });
  }

  public HashMap<String, Command> getMap() {
    return eventMap;
  }

  public Command getCommandInMap(String key) {
    return eventMapGetter.get(key).get();
  }
}
