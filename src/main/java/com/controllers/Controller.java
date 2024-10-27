package com.controllers;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller implements Sendable {
        public final BiConsumer<RumbleType, Double> rumbleConsumer;
        public final DoubleSupplier leftXValue;
        public final DoubleSupplier leftYValue;
        public final DoubleSupplier rightXValue;
        public final DoubleSupplier rightYValue;
        public final DoubleSupplier leftTriggerValue;
        public final DoubleSupplier rightTriggerValue;
        public final DoubleSupplier fullTriggerValue;
        public final Trigger leftX;
        public final Trigger leftY;
        public final Trigger rightX;
        public final Trigger rightY;
        public final Trigger leftTrigger;
        public final Trigger rightTrigger;
        public final Trigger fullTrigger;
        public final Trigger a;
        public final Trigger b;
        public final Trigger x;
        public final Trigger y;
        public final Trigger leftBumper;
        public final Trigger rightBumper;
        public final Trigger back;
        public final Trigger start;
        public final Trigger padUp;
        public final Trigger padRight;
        public final Trigger padDown;
        public final Trigger padLeft;

        private Controller(
                        BiConsumer<RumbleType, Double> rumbleConsumer,
                        DoubleSupplier leftXValue,
                        DoubleSupplier leftYValue,
                        DoubleSupplier rightXValue,
                        DoubleSupplier rightYValue,
                        DoubleSupplier leftTriggerValue,
                        DoubleSupplier rightTriggerValue,
                        DoubleSupplier fullTriggerValue,
                        Trigger leftX,
                        Trigger leftY,
                        Trigger rightX,
                        Trigger rightY,
                        Trigger leftTrigger,
                        Trigger rightTrigger,
                        Trigger fullTrigger,
                        Trigger a,
                        Trigger b,
                        Trigger x,
                        Trigger y,
                        Trigger leftBumper,
                        Trigger rightBumper,
                        Trigger back,
                        Trigger start,
                        Trigger padUp,
                        Trigger padRight,
                        Trigger padDown,
                        Trigger padLeft) {

                this.rumbleConsumer = rumbleConsumer;
                this.leftXValue = leftXValue;
                this.leftYValue = leftYValue;
                this.rightXValue = rightXValue;
                this.rightYValue = rightYValue;
                this.leftTriggerValue = leftTriggerValue;
                this.rightTriggerValue = rightTriggerValue;
                this.fullTriggerValue = fullTriggerValue;
                this.leftX = leftX;
                this.leftY = leftY;
                this.rightX = rightX;
                this.rightY = rightY;
                this.leftTrigger = leftTrigger;
                this.rightTrigger = rightTrigger;
                this.fullTrigger = fullTrigger;
                this.a = a;
                this.b = b;
                this.x = x;
                this.y = y;
                this.leftBumper = leftBumper;
                this.rightBumper = rightBumper;
                this.back = back;
                this.start = start;
                this.padUp = padUp;
                this.padRight = padRight;
                this.padDown = padDown;
                this.padLeft = padLeft;

        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty(
                                "LEFT X Value",
                                leftXValue,
                                null);
                builder.addDoubleProperty(
                                "LEFT Y Value",
                                leftYValue,
                                null);
                builder.addDoubleProperty(
                                "RIGHT X Value",
                                rightXValue,
                                null);
                builder.addDoubleProperty(
                                "RIGHT Y Value",
                                rightYValue,
                                null);
                builder.addDoubleProperty(
                                "LEFT TRIGGER Value",
                                leftTriggerValue,
                                null);
                builder.addDoubleProperty(
                                "RIGHT TRIGGER Value",
                                rightTriggerValue,
                                null);
                builder.addDoubleProperty(
                                "FULL TRIGGER Value",
                                fullTriggerValue,
                                null);
                builder.addBooleanProperty(
                                "LEFT X",
                                leftX,
                                null);
                builder.addBooleanProperty(
                                "LEFT Y",
                                leftY,
                                null);
                builder.addBooleanProperty(
                                "RIGHT X",
                                rightX,
                                null);
                builder.addBooleanProperty(
                                "RIGHT Y",
                                rightY,
                                null);
                builder.addBooleanProperty(
                                "LEFT TRIGGER",
                                leftTrigger,
                                null);
                builder.addBooleanProperty(
                                "RIGHT TRIGGER",
                                rightTrigger,
                                null);
                builder.addBooleanProperty(
                                "FULL TRIGGER",
                                fullTrigger,
                                null);
                builder.addBooleanProperty(
                                "A",
                                a,
                                null);
                builder.addBooleanProperty(
                                "B",
                                b,
                                null);
                builder.addBooleanProperty(
                                "X",
                                x,
                                null);
                builder.addBooleanProperty(
                                "Y",
                                y,
                                null);
                builder.addBooleanProperty(
                                "LEFT BUMPER",
                                leftBumper,
                                null);
                builder.addBooleanProperty(
                                "RIGHT BUMPER",
                                rightBumper,
                                null);
                builder.addBooleanProperty(
                                "UP",
                                padUp,
                                null);
                builder.addBooleanProperty(
                                "DOWN",
                                padDown,
                                null);
                builder.addBooleanProperty(
                                "LEFT",
                                padLeft,
                                null);
                builder.addBooleanProperty(
                                "RIGHT",
                                padRight,
                                null);
                builder.addBooleanProperty(
                                "BACK",
                                back,
                                null);
                builder.addBooleanProperty(
                                "START",
                                start,
                                null);
        }

        public static final Function<Integer, Function<Double, Function<Boolean, Controller>>> createFromXBox = (
                        port) -> (deadband) -> (hasRumble) -> {
                                CommandXboxController commandXboxController = new CommandXboxController(port);
                                BiConsumer<RumbleType, Double> rumbleConsumer = (type, value) -> {
                                        if (hasRumble && RobotState.isTeleop() && RobotState.isEnabled()) {
                                                commandXboxController
                                                                .getHID().setRumble(type, value);
                                        }else{
                                                commandXboxController.getHID().setRumble(type, 0.0);
                                        }


                                };

                                DoubleSupplier leftXValue = () -> {
                                        double leftXRawValue = -MathUtil.applyDeadband(commandXboxController.getLeftX(),
                                                        deadband);
                                        double leftYRawValue = -MathUtil.applyDeadband(commandXboxController.getLeftY(),
                                                        deadband);
                                        double mag = Math.hypot(leftXRawValue, leftYRawValue);
                                        return mag > 1 ? leftXRawValue / mag : leftXRawValue;
                                };
                                DoubleSupplier leftYValue = () -> {
                                        double leftXRawValue = -MathUtil.applyDeadband(commandXboxController.getLeftX(),
                                                        deadband);
                                        double leftYRawValue = -MathUtil.applyDeadband(commandXboxController.getLeftY(),
                                                        deadband);
                                        double mag = Math.hypot(leftXRawValue, leftYRawValue);
                                        return mag > 1 ? leftYRawValue / mag : leftYRawValue;
                                };
                                DoubleSupplier rightXValue = () -> {
                                        double rightXRawValue = -MathUtil
                                                        .applyDeadband(commandXboxController.getRightX(), deadband);
                                        double rightYRawValue = -MathUtil
                                                        .applyDeadband(commandXboxController.getRightY(), deadband);
                                        double mag = Math.hypot(rightXRawValue, rightYRawValue);
                                        return mag > 1 ? rightXRawValue / mag : rightXRawValue;
                                };
                                DoubleSupplier rightYValue = () -> {
                                        double rightXRawValue = -MathUtil
                                                        .applyDeadband(commandXboxController.getRightX(), deadband);
                                        double rightYRawValue = -MathUtil
                                                        .applyDeadband(commandXboxController.getRightY(), deadband);
                                        double mag = Math.hypot(rightXRawValue, rightYRawValue);
                                        return mag > 1 ? rightYRawValue / mag : rightYRawValue;
                                };

                                DoubleSupplier leftTriggerValue = () -> MathUtil.applyDeadband(
                                                commandXboxController.getLeftTriggerAxis(),
                                                deadband);
                                DoubleSupplier rightTriggerValue = () -> MathUtil.applyDeadband(
                                                commandXboxController.getRightTriggerAxis(),
                                                deadband);

                                DoubleSupplier fullTriggerValue = () -> leftTriggerValue.getAsDouble()
                                                - rightTriggerValue.getAsDouble();

                                Trigger leftX = new Trigger(() -> leftXValue.getAsDouble() != 0);
                                Trigger leftY = new Trigger(() -> leftYValue.getAsDouble() != 0);
                                Trigger rightX = new Trigger(() -> rightXValue.getAsDouble() != 0);
                                Trigger rightY = new Trigger(() -> rightYValue.getAsDouble() != 0);
                                Trigger leftTrigger = new Trigger(() -> leftTriggerValue.getAsDouble() != 0);
                                Trigger rightTrigger = new Trigger(() -> rightTriggerValue.getAsDouble() != 0);
                                Trigger fullTrigger = new Trigger(() -> {
                                        return leftTrigger.getAsBoolean() || rightTrigger.getAsBoolean();
                                });
                                return new Controller(
                                                rumbleConsumer,
                                                leftXValue,
                                                leftYValue,
                                                rightXValue,
                                                rightYValue,
                                                leftTriggerValue,
                                                rightTriggerValue,
                                                fullTriggerValue,
                                                leftX,
                                                leftY,
                                                rightX,
                                                rightY,
                                                leftTrigger,
                                                rightTrigger,
                                                fullTrigger,
                                                commandXboxController.a(),
                                                commandXboxController.b(),
                                                commandXboxController.x(),
                                                commandXboxController.y(),
                                                commandXboxController.leftBumper(),
                                                commandXboxController.rightBumper(),
                                                commandXboxController.back(),
                                                commandXboxController.start(),
                                                commandXboxController.pov(0),
                                                commandXboxController.pov(90),
                                                commandXboxController.pov(180),
                                                commandXboxController.pov(270));
                        };

}