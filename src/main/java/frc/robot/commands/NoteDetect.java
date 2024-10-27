package frc.robot.commands;

import static frc.robot.RobotContainer.driveSubsystem;
import static frc.robot.RobotContainer.elevatorSubsystem;
import static frc.robot.RobotContainer.innerIntakeSubsystem;
import static frc.robot.RobotContainer.notedLoadedSubsystem;
import static frc.robot.RobotContainer.outerIntakeSubsystem;
import static frc.robot.RobotContainer.shooterTiltSubsystem;
import static frc.robot.RobotContainer.singulatorSubsystem;
import static frc.robot.RobotContainer.telemetrySubsystem;
import static frc.robot.RobotContainer.transportSubsystem;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public abstract class NoteDetect {

        public static final Supplier<Command> create = () -> {
                Command setObjectDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableObjectDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command waitUntilNoteDetected = Commands.waitUntil(() -> {
                        return telemetrySubsystem.objectPositions.get(0).getSecond().get().isPresent();
                });

                Command goToNoteCommand = Commands.deferredProxy(DriveCommands.goToNoteCommandSupplier);

                Command setFieldDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command pickupDetectedNote = Commands
                .race(goToNoteCommand, Ground.groundCommandThenLevel.get())
                .until(notedLoadedSubsystem.hasNote);

                Command command = Commands.sequence(setObjectDetectModeCommand, waitUntilNoteDetected,
                                pickupDetectedNote,
                                setFieldDetectModeCommand);

                command.setName("Note Detect");
                return command;
        };

        public static final Supplier<Command> createForAuto = () -> {
                Command setObjectDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableObjectDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command waitUntilNoteDetected = Commands.waitUntil(() -> {
                        return telemetrySubsystem.objectPositions.get(0).getSecond().get().isPresent();
                });

                Command waitDelay = Commands.waitSeconds(0.3);
                Command delay = Commands.race(waitUntilNoteDetected, waitDelay);

                Set<Subsystem> set = Set.of(
                        driveSubsystem,
                        innerIntakeSubsystem,
                        outerIntakeSubsystem,
                        transportSubsystem,
                        elevatorSubsystem,
                        shooterTiltSubsystem,
                        singulatorSubsystem
                );

                Command goToNoteCommand = Commands.defer(DriveCommands.goToNoteCommandSupplier, set);
                ;

                Command setFieldDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command command = Commands.sequence(setObjectDetectModeCommand, delay,
                                goToNoteCommand,
                                setFieldDetectModeCommand);

                command.setName("Note Detect");
                return command;
        };

}
