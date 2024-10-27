package com.compLevel1;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import java.util.function.Consumer;
import com.compLevel0.Motor;
import com.compLevel0.Sensor;
import com.types.SixFunction;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class SwerveModule {
        public final Measure<Voltage> steerVoltage;
        public final Measure<Voltage> wheelVoltage;
        public final Measure<Voltage> angleSensorVoltage;
        public final Measure<Angle> angle;
        public final Measure<Angle> angleFromSensor;
        public final Measure<Distance> wheelPosition;
        public final Measure<Velocity<Distance>> wheelVelocity;
        public final Consumer<SwerveModuleState> controlState;
        public final Runnable resetSteerEncoderFromAbsolute;
        public final Runnable stopAndHold;
        public final Runnable update;

        private SwerveModule(
                        Measure<Voltage> steerVoltage,
                        Measure<Voltage> wheelVoltage,
                        Measure<Voltage> angleSensorVoltage,
                        Measure<Angle> angle,
                        Measure<Angle> angleFromSensor,
                        Measure<Distance> wheelPosition,
                        Measure<Velocity<Distance>> wheelVelocity,
                        Consumer<SwerveModuleState> controlState,
                        Runnable resetSteerEncoderFromAbsolute,
                        Runnable stopAndHold,
                        Runnable update) {
                this.steerVoltage = steerVoltage;
                this.wheelVoltage = wheelVoltage;
                this.angleSensorVoltage = angleSensorVoltage;
                this.angle = angle;
                this.angleFromSensor = angleFromSensor;
                this.wheelPosition = wheelPosition;
                this.wheelVelocity = wheelVelocity;
                this.controlState = controlState;
                this.resetSteerEncoderFromAbsolute = resetSteerEncoderFromAbsolute;
                this.stopAndHold = stopAndHold;
                this.update = update;

                resetSteerEncoderFromAbsolute.run();
        }

        public static final SixFunction<Motor, Motor, Double, Double , Measure<Distance>, Sensor<Angle>, SwerveModule> create = (
                        steerMotor, wheelMotor, steerGearing, wheelGearing, wheelRadius, angleSensor) -> {
                                                                MutableMeasure<Angle> angle = MutableMeasure
                                                                                .zero(Rotations);
                                                                MutableMeasure<Angle> absoluteAngle = MutableMeasure
                                                                                .zero(Rotations);
                                                                MutableMeasure<Distance> wheelPosition = MutableMeasure
                                                                                .zero(Meters);
                                                                MutableMeasure<Velocity<Distance>> wheelVelocity = MutableMeasure
                                                                                .zero(MetersPerSecond);

                                                                MutableMeasure<Angle> steerSetpoint = MutableMeasure
                                                                                .zero(Radians);
                                                                MutableMeasure<Velocity<Angle>> wheelSetpoint = MutableMeasure
                                                                                .zero(RadiansPerSecond);
                                                                Consumer<SwerveModuleState> controlState = (state) -> {
                                                                        steerSetpoint.mut_setMagnitude(
                                                                                        state.angle.getRadians());
                                                                        steerSetpoint.mut_times(steerGearing);
                                                                        wheelSetpoint.mut_setMagnitude(
                                                                                        state.speedMetersPerSecond);
                                                                        wheelSetpoint.mut_divide(
                                                                                        wheelRadius.in(Meters));
                                                                        wheelSetpoint.mut_times(wheelGearing);
                                                                        steerMotor.turn.accept(steerSetpoint);
                                                                        wheelMotor.spin.accept(wheelSetpoint);
                                                                };

                                                                MutableMeasure<Angle> resetSetpoint = MutableMeasure
                                                                                .zero(Rotations);
                                                                Runnable resetSteerEncoderFromAbsolute = () -> {
                                                                        steerMotor.setRelativeEncoderAngle
                                                                                        .accept(Rotations.of(
                                                                                                        angleSensor.measure
                                                                                                                        .in(Rotations)
                                                                                                                        * steerGearing));


                                                                };

                                                                Runnable stopAndHold = () -> {
                                                                        steerMotor.stop.run();
                                                                        wheelMotor.stop.run();
                                                                };

                                                                Runnable update = () -> {
                                                                        steerMotor.update.run();
                                                                        wheelMotor.update.run();
                                                                        absoluteAngle.mut_setMagnitude(
                                                                                        steerMotor.angle.in(Rotations)
                                                                                                        / steerGearing);
                                                                        angleSensor.update.run();
                                                                        resetSetpoint.mut_setMagnitude(
                                                                                        angleSensor.measure
                                                                                                        .in(Rotations)
                                                                                                        * steerGearing);
                                                                        angle.mut_setMagnitude(
                                                                                        steerMotor.angle.in(Rotations)
                                                                                                        / steerGearing);
                                                                        wheelPosition.mut_setMagnitude(
                                                                                        wheelMotor.angle.in(Radians)
                                                                                                        * wheelRadius.in(
                                                                                                                        Meters)
                                                                                                        / wheelGearing);
                                                                        wheelVelocity.mut_setMagnitude(
                                                                                        wheelMotor.angularVelocity.in(
                                                                                                        RadiansPerSecond)
                                                                                                        * wheelRadius.in(
                                                                                                                        Meters)
                                                                                                        / wheelGearing);

                                                                };

                                                                return new SwerveModule(
                                                                                steerMotor.voltage,
                                                                                wheelMotor.voltage,
                                                                                angleSensor.voltage,
                                                                                angle,
                                                                                angleSensor.measure,
                                                                                wheelPosition,
                                                                                wheelVelocity,
                                                                                controlState,
                                                                                resetSteerEncoderFromAbsolute,
                                                                                stopAndHold,
                                                                                update);
                                                        };
}
