package frc.robot.commands.swervedrive2.auto;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;
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
            run(() -> arm.extendArmLow(), arm).withTimeout(1.2).andThen(() -> arm.stopArm(), arm));
    eventMapGetter.put(
        "ArmHigh",
        () ->
            run(() -> arm.extendArmHigh(), arm).withTimeout(1.7).andThen(() -> arm.stopArm(), arm));
    eventMapGetter.put(
        "ArmStow",
        () -> run(() -> arm.stowArm(), arm).withTimeout(1.7).andThen(() -> arm.stopArm(), arm));
    eventMapGetter.put(
        "ArmMid",
        () ->
            run(() -> arm.extendArmMid(), arm).withTimeout(1.2).andThen(() -> arm.stopArm(), arm));

    // For grabbing from the loading station
    eventMapGetter.put(
        "CubeGrab",
        () ->
            run(() -> intake.intakeCube(), intake)
                .withTimeout(5)
                .andThen(runOnce(() -> intake.stop(), intake)));

    eventMapGetter.put(
        "ConeGrab",
        () ->
            run(() -> intake.intakeCone(), intake)
                .withTimeout(5)
                .andThen(runOnce(() -> intake.stop(), intake)));

    // Only used to pull cone in tighter at beginning of auto
    eventMapGetter.put(
        "IntakeCone", () -> run(() -> intake.intakeCone(), intake).withTimeout(0.25));

    // For handoff from ground to intake
    eventMapGetter.put(
        "IntakeCube",
        () ->
            run(() -> intake.intakeCube(), intake)
                .withTimeout(0.5)
                .andThen(runOnce(() -> intake.stop(), intake)));

    // For placing both type of game pieces on the grid
    eventMapGetter.put(
        "OuttakeCube", () -> run(() -> intake.outtakeCube(), intake).withTimeout(0.1));

    eventMapGetter.put(
        "OuttakeCone",
        () ->
            run(() -> intake.outtakeCone(), intake)
                .withTimeout(0.2)
                .andThen(runOnce(() -> intake.stop(), intake)));

    // Automatic control of the ground arm
    eventMapGetter.put(
        "DropIntake",
        () ->
            run(() -> ground.extendArm(), ground)
                .withTimeout(0.9)
                .andThen(runOnce(() -> ground.stopArm(), ground)));

    eventMapGetter.put(
        "RetractIntake",
        () ->
            run(() -> ground.stowArm(), ground)
                .withTimeout(0.9)
                .andThen(runOnce(() -> ground.stopArm(), ground)));

    eventMapGetter.put(
        "GroundOuttake",
        () ->
            run(() -> ground.outtakeCube(), ground)
                .withTimeout(0.5)
                .andThen(runOnce(() -> ground.stopIntake(), ground)));

    eventMapGetter.put(
        "IntakeHigh", () -> getCommandInMap("IntakeCone").alongWith(getCommandInMap("ArmHigh")));

    eventMapGetter.put(
        "OuttakeStow",
        () ->
            getCommandInMap("OuttakeCone")
                .alongWith(getCommandInMap("ArmStow"))
                .andThen(() -> intake.stop(), intake));

    eventMapGetter.put(
        "GroundDeploy",
        () ->
            // Bring the Arm in first as this can take a bit
            getCommandInMap("ArmStow")
                // Meanwhile start the intake sucking in and drop it
                .alongWith(
                    runOnce(() -> ground.intakeCube()).andThen(getCommandInMap("DropIntake"))));

    eventMapGetter.put(
        "GroundStow",
        () ->
            // Bring the Arm in first as this can take a bit
            getCommandInMap("ArmStow")
                // Stop Intake and bring it in
                .alongWith(
                    runOnce(() -> ground.stopIntake()).andThen(getCommandInMap("RetractIntake")))
                // Run intake cube and Ground Outake
                .andThen(
                    getCommandInMap("IntakeCube").alongWith(getCommandInMap("GroundOuttake"))));

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
