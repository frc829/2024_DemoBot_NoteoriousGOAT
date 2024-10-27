package com.compLevel0;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class Motor {
        public final MutableMeasure<Voltage> voltage;
        public final MutableMeasure<Angle> angle;
        public final MutableMeasure<Angle> absoluteAngle;
        public final MutableMeasure<Velocity<Angle>> angularVelocity;
        public final Measure<Velocity<Angle>> maxAngularVelocity;
        public final Consumer<Measure<Angle>> setRelativeEncoderAngle;
        public final Consumer<Measure<Angle>> turn;
        public final Consumer<Measure<Velocity<Angle>>> spin;
        public final Consumer<Measure<Voltage>> setVoltage;
        public final Runnable stop;
        public final Runnable update;

        private Motor(
                        MutableMeasure<Voltage> voltage,
                        MutableMeasure<Angle> angle,
                        MutableMeasure<Angle> absoluteAngle,
                        MutableMeasure<Velocity<Angle>> angularVelocity,
                        Measure<Velocity<Angle>> maxAngularVelocity,
                        Consumer<Measure<Angle>> setRelativeEncoderAngle,
                        Consumer<Measure<Angle>> turn,
                        Consumer<Measure<Velocity<Angle>>> spin,
                        Consumer<Measure<Voltage>> setVoltage,
                        Runnable stop,
                        Runnable update) {
                this.voltage = voltage;
                this.angle = angle;
                this.absoluteAngle = absoluteAngle;
                this.angularVelocity = angularVelocity;
                this.maxAngularVelocity = maxAngularVelocity;
                this.setRelativeEncoderAngle = setRelativeEncoderAngle;
                this.turn = turn;
                this.spin = spin;
                this.setVoltage = setVoltage;
                this.stop = stop;
                this.update = update;
        }

        public static final class REV {

                // #region createMotorFromCANSparkMax
                public static final TriFunction<CANSparkBase, Measure<Velocity<Angle>>, Measure<Voltage>, Motor> createMotorFromCANSparkBase = (
                                canSparkBase, maxAngularVelocity, kS) -> {
                        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                        MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure.zero(RPM);
                        Consumer<Measure<Angle>> setRelativeEncoderAngle = (setpoint) -> {
                                canSparkBase
                                                .getEncoder()
                                                .setPosition(setpoint.in(Rotations));
                        };

                        PIDController pidController = new PIDController(
                                        canSparkBase.getPIDController().getP(1),
                                        canSparkBase.getPIDController().getI(1),
                                        canSparkBase.getPIDController().getD(1));
                        if (canSparkBase.getPIDController().getPositionPIDWrappingEnabled()) {
                                pidController.enableContinuousInput(
                                                canSparkBase.getPIDController()
                                                                .getPositionPIDWrappingMinInput(),
                                                canSparkBase.getPIDController()
                                                                .getPositionPIDWrappingMaxInput());
                        }

                        Consumer<Measure<Angle>> turn = (setpoint) -> {
                                if (RobotBase.isSimulation()) {

                                        MutableMeasure<Voltage> voltageSetpoint = MutableMeasure.zero(Volts);
                                        double measurement = canSparkBase.getEncoder().getPosition();
                                        double goal = setpoint.in(Rotations);
                                        double volts = pidController.calculate(measurement,
                                                        goal);
                                        volts = MathUtil.clamp(volts, -12.0, 12.0);
                                        voltageSetpoint.mut_setMagnitude(volts / 12.0);
                                        canSparkBase.getPIDController().setReference(voltageSetpoint.in(Volts),
                                                        ControlType.kVoltage);
                                } else {
                                        canSparkBase.getPIDController()
                                                        .setReference(setpoint.in(Rotations),
                                                                        ControlType.kPosition, 1);
                                }

                        };
                        Consumer<Measure<Velocity<Angle>>> spin = (setpoint) -> {
                                if (RobotBase.isSimulation()) {
                                        double rpm = setpoint.in(RPM);
                                        rpm = MathUtil.clamp(
                                                        rpm,
                                                        -maxAngularVelocity.in(RPM),
                                                        maxAngularVelocity.in(RPM));
                                        if (rpm != 0) {
                                                double arb = kS.in(Volts);
                                                if (rpm < 0) {
                                                        arb *= -1;
                                                }
                                                canSparkBase
                                                                .getPIDController()
                                                                .setReference(rpm, ControlType.kVelocity, 0,
                                                                                arb,
                                                                                ArbFFUnits.kVoltage);
                                        } else {
                                                canSparkBase
                                                                .getPIDController()
                                                                .setReference(rpm, ControlType.kVelocity, 0);
                                        }

                                } else {
                                        double rpm = setpoint.in(RPM);
                                        double arb = kS.in(Volts);
                                        if (rpm < 0) {
                                                arb *= -1;
                                        }
                                        if (rpm != 0) {
                                                canSparkBase
                                                                .getPIDController()
                                                                .setReference(rpm, ControlType.kVelocity, 0,
                                                                                arb,
                                                                                ArbFFUnits.kVoltage);
                                        } else {
                                                canSparkBase
                                                                .getPIDController()
                                                                .setReference(rpm, ControlType.kVelocity, 0);
                                        }
                                }
                        };
                        Consumer<Measure<Voltage>> setVoltage = (setpoint) -> {
                                canSparkBase.getPIDController().setReference(setpoint.in(Volts),
                                                ControlType.kVoltage);
                        };
                        Runnable stop = () -> canSparkBase.setVoltage(0);
                        Runnable update = () -> {
                                voltage.mut_setMagnitude(
                                                canSparkBase.getAppliedOutput()
                                                                * canSparkBase.getBusVoltage());
                                angle.mut_setMagnitude(canSparkBase.getEncoder().getPosition());

                                angularVelocity.mut_setMagnitude(
                                                canSparkBase.getEncoder().getVelocity());

                                if (RobotBase.isSimulation()) {
                                        absoluteAngle.mut_setMagnitude(0);
                                } else {
                                        double absoluteAngleValue = canSparkBase.getAbsoluteEncoder(Type.kDutyCycle)
                                                        .getPosition();
                                        absoluteAngle.mut_setMagnitude(absoluteAngleValue);
                                }

                        };

                        return new Motor(voltage, angle, absoluteAngle, angularVelocity, maxAngularVelocity,
                                        setRelativeEncoderAngle, turn, spin, setVoltage, stop, update);
                };
                // #endregion

                public static final BiFunction<CANSparkBase, Measure<Voltage>, Motor> createNEOMotor = (
                                canSparkBase, kS) -> createMotorFromCANSparkBase.apply(canSparkBase,
                                                RadiansPerSecond.of(DCMotor.getNEO(1).freeSpeedRadPerSec),
                                                kS);

                public static final BiFunction<CANSparkBase, Measure<Voltage>, Motor> createNEOVortexMotor = (
                                canSparkBase, kS) -> createMotorFromCANSparkBase.apply(canSparkBase,
                                                RadiansPerSecond.of(DCMotor.getNeoVortex(1).freeSpeedRadPerSec),
                                                kS);

                public static final BiFunction<CANSparkBase, Measure<Voltage>, Motor> createNEO550Motor = (
                                canSparkBase, kS) -> createMotorFromCANSparkBase.apply(canSparkBase,
                                                RadiansPerSecond.of(DCMotor.getNeo550(1).freeSpeedRadPerSec),
                                                kS);
        };

        public static final class CTRE {

                // #region createMotorFromTalonFX
                public static final BiFunction<TalonFX, Measure<Velocity<Angle>>, Motor> createMotorFromTalonFX = (
                                talonFX, maxAngularVelocity) -> {

                        TalonFXSimState talonFXSimState = talonFX.getSimState();
                        DCMotorSim dcMotorSim = new DCMotorSim(
                                        DCMotor.getKrakenX60(1),
                                        1,
                                        0.00001);

                        MutableMeasure<Time> lastTime = MutableMeasure
                                        .ofBaseUnits(Timer.getFPGATimestamp(), Seconds);
                        MutableMeasure<Time> currentTime = MutableMeasure
                                        .ofBaseUnits(Timer.getFPGATimestamp(), Seconds);

                        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                        MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure
                                        .zero(RotationsPerSecond);

                        Consumer<Measure<Angle>> setRelativeEncoder = (setpoint) -> {
                        };
                        Consumer<Measure<Angle>> turn = (setpoint) -> {
                        };
                        VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(
                                        0,
                                        0,
                                        0,
                                        0,
                                        false,
                                        false,
                                        false);
                        Consumer<Measure<Velocity<Angle>>> spin = (setpoint) -> {
                                if (setpoint.in(RotationsPerSecond) > 0) {
                                        talonFX.setControl(velocityTorqueCurrentFOC
                                                        .withVelocity(setpoint.in(RotationsPerSecond))
                                                        .withFeedForward(1));
                                } else if (setpoint.in(RotationsPerSecond) < 0) {
                                        talonFX.setControl(velocityTorqueCurrentFOC
                                                        .withVelocity(setpoint.in(RotationsPerSecond))
                                                        .withFeedForward(-1));
                                } else {
                                        talonFX.setControl(velocityTorqueCurrentFOC
                                                        .withVelocity(setpoint.in(RotationsPerSecond))
                                                        .withFeedForward(0));
                                }
                        };
                        Consumer<Measure<Voltage>> setVoltage = (setpoint) -> {
                        };
                        NeutralOut brake = new NeutralOut();
                        Runnable stop = () -> {
                                if (RobotBase.isSimulation()) {
                                        talonFX.setControl(brake);
                                } else {
                                        talonFX.setControl(brake);
                                }
                        };
                        Runnable update = () -> {
                                if (RobotBase.isSimulation()) {
                                        currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
                                        double deltaTime = currentTime.in(Seconds)
                                                        - lastTime.in(Seconds);
                                        dcMotorSim.setInputVoltage(talonFXSimState.getMotorVoltage());
                                        dcMotorSim.update(deltaTime);
                                        talonFXSimState.setRawRotorPosition(
                                                        dcMotorSim.getAngularPositionRotations());
                                        talonFXSimState.setRotorVelocity(
                                                        dcMotorSim.getAngularVelocityRPM() / 60.0);
                                        talonFXSimState.setSupplyVoltage(12
                                                        - talonFXSimState.getSupplyCurrent() * 0.002);
                                        lastTime.mut_setMagnitude(currentTime.in(Seconds));
                                }
                                voltage.mut_setMagnitude(talonFX.getSupplyVoltage().getValueAsDouble());
                                angle.mut_setMagnitude(talonFX.getPosition().getValueAsDouble());
                                angularVelocity.mut_setMagnitude(
                                                talonFX.getVelocity().getValueAsDouble());
                        };

                        return new Motor(voltage, angle, absoluteAngle, angularVelocity,
                                        maxAngularVelocity,
                                        setRelativeEncoder, turn, spin, setVoltage, stop, update);
                };

                public static final Function<TalonFX, Motor> createKrakenX60FOCMotor = (talonFX) -> {
                        return createMotorFromTalonFX.apply(
                                        talonFX,
                                        RadiansPerSecond.of(DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec));
                };
                // #endregion
        };

}
