// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.driver;
import static frc.robot.RobotContainer.operator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ControllerCommands {
    public static final Supplier<Command> createOperatorRumble = () -> {
        Runnable rumble = () -> {
            operator.rumbleConsumer.accept(RumbleType.kBothRumble, 0.5);

        };
        Runnable stop = () -> {
            operator.rumbleConsumer.accept(RumbleType.kBothRumble, 0.0);
        };
        Command rumbleCommand = Commands.race(Commands.run(rumble), Commands.waitSeconds(1.5));
        Command stopRumbleCommand = Commands.runOnce(stop);
        Command command = rumbleCommand.andThen(stopRumbleCommand);
        command.setName("Operator Rumble");
        return command;
    };

    public static final Supplier<Command> createDriverRumble = () -> {
        Runnable rumble = () -> {
            driver.rumbleConsumer.accept(RumbleType.kBothRumble, 0.35);

        };
        Runnable stop = () -> {
            driver.rumbleConsumer.accept(RumbleType.kBothRumble, 0.0);
        };
        Command rumbleCommand = Commands.race(Commands.run(rumble), Commands.waitSeconds(0.5));
        Command stopRumbleCommand = Commands.runOnce(stop);
        Command command = rumbleCommand.andThen(stopRumbleCommand);
        command.setName("Driver Rumble");
        return command;
    };

}
