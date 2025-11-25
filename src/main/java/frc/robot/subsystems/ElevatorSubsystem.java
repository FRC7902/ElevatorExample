// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /**
   * Leader TalonFX motor
   */
  private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_CAN_ID);

  /**
   * Follower TalonFX motor
   */
  private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_CAN_ID);

  /**
   * Motion magic request object
   */
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);

  /**
   * Mechanism2d instance for the elevator
   */
  private final Mechanism2d m_mech2d = new Mechanism2d(
          Units.inchesToMeters(50),
          Units.inchesToMeters(100)
  );

  /**
   * Elevator base mechanism root
   */
  private final MechanismRoot2d m_elevatorBase2d = m_mech2d.getRoot(
          "ElevatorRoot",
          Units.inchesToMeters(25),
          Units.inchesToMeters(1)
  );

  /**
   * Main elevator ligament, represents the whole elevator
   */
  private final MechanismLigament2d m_elevator2d = m_elevatorBase2d.append(
          new MechanismLigament2d(
                  "Elevator",
                  ElevatorConstants.HEIGHT_METERS,
                  90,
                  7,
                  new Color8Bit(Color.kAntiqueWhite)
          )
  );

  /**
   * Simulated elevator physics simulation instance
   */
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
          DCMotor.getKrakenX60(2),
          ElevatorConstants.GEARING,
          ElevatorConstants.CARRIAGE_MASS,
          ElevatorConstants.DRUM_RADIUS,
          ElevatorConstants.MIN_HEIGHT_METERS,
          ElevatorConstants.MAX_HEIGHT_METERS,
          true,
          ElevatorConstants.HEIGHT_METERS,
          0, 0
  );

  private double m_elevatorSetpointMeters = ElevatorConstants.MIN_HEIGHT_METERS;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Configure motors
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Configure slot 0
    // PID
    configs.Slot0.kP = ElevatorConstants.PID_P;
    configs.Slot0.kI = ElevatorConstants.PID_I;
    configs.Slot0.kD = ElevatorConstants.PID_D;

    // Gravity
    configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // MotionMagic
    configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY;
    configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCEL;

    // Set current limits
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;

    // Set follower control
    m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), true));

    // Apply configuration to motors
    m_leaderMotor.getConfigurator().apply(configs);
    m_followerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator/info/Position (m)", getElevatorPositionMeters());
    SmartDashboard.putNumber("elevator/info/Setpoint (m)", m_elevatorSetpointMeters);
    SmartDashboard.putBoolean("elevator/info/At Setpoint", hasReachedSetpoint());
    SmartDashboard.putData("elevator/sim/mech2d",  m_mech2d);
  }

  @Override
  public void simulationPeriodic() {
    m_leaderMotor.getSimState().setSupplyVoltage(12);

    // Provide the physics sim with data
    m_elevatorSim.setInputVoltage(m_leaderMotor.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(0.02);

    // Set the simulation state of the Motors
    final double positionRotations = m_elevatorSim.getPositionMeters() / ElevatorConstants.METERS_PER_MOTOR_ROTATION;
    final double velocityRPS = m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_MOTOR_ROTATION;
    m_leaderMotor.getSimState().setRawRotorPosition(positionRotations);
    m_leaderMotor.getSimState().setRotorVelocity(velocityRPS);

    // Set the length of the elevator
    m_elevator2d.setLength(getElevatorPositionMeters());
  }

  /**
   * Method that gets the position of the elevator in meters
   * @return the position of the elevator in meters
   */
  public double getElevatorPositionMeters() {
    double rotations = m_leaderMotor.getPosition().getValueAsDouble();
    return rotations*ElevatorConstants.METERS_PER_MOTOR_ROTATION;
  }

  /**
   * Determine whether the elevator has reached its setpoint with reasonable accuracy
   * @return if the elevator has reached its setpoint or not
   */
  public boolean hasReachedSetpoint() {
    double position = getElevatorPositionMeters();
    return Math.abs(position - m_elevatorSetpointMeters) < ElevatorConstants.ERROR_METERS;
  }

  /**
   * Set the desired height of the elevator in meters. The elevator will attempt to move to this height
   * @param positionMeters
   */
  public void setElevatorPositionMeters(double positionMeters) {
    // Clamp the input
    if (positionMeters > ElevatorConstants.MAX_HEIGHT_METERS) {
      positionMeters = ElevatorConstants.MAX_HEIGHT_METERS;
    } else if (positionMeters < ElevatorConstants.MIN_HEIGHT_METERS) {
      positionMeters = ElevatorConstants.MIN_HEIGHT_METERS;
    }
    m_elevatorSetpointMeters = positionMeters;

    // Convert meters to rotations
    double rotations = m_elevatorSetpointMeters / ElevatorConstants.METERS_PER_MOTOR_ROTATION;

    // Set control
    m_motionMagicRequest.Slot = 0;
    m_motionMagicRequest.Position = rotations;
    m_leaderMotor.setControl(m_motionMagicRequest);
  }
}
