// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States;

import static frc.robot.Constants.ElevatorConstants.*;

//import org.apache.commons.collections4.map.HashedMap;

public class Elevator extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController heightPID;
  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private double currentPosition = 0.0;

  public Elevator(int motorCANID) {
    // The motor controlling the angle of the assembly
    motor = new SparkMax(motorCANID, SparkMax.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(kHeightPositionFactor) 
        .velocityConversionFactor(kHeightPositionFactor);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kFF);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // angleMotor.setSmartCurrentLimit(CurrentLimit.kIntakeAngle);
    // angleMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    
    // angleMotor.setSoftLimit(SoftLimitDirection.kForward, intakeStates.get(true).floatValue());
    // angleMotor.setSoftLimit(SoftLimitDirection.kReverse, intakeStates.get(false).floatValue());

    encoder = motor.getEncoder();
    heightPID = motor.getClosedLoopController();

  }

  @Override
  public void periodic() {

    // If the intake is within tolerance of it's desired angle, set this variable to
    // true so other files can act

    goToHeight(currentPosition);

    // Putting data on SmartDashboard
    SmartDashboard.putNumber("Height", encoder.getPosition());
    SmartDashboard.putNumber("Desired Height", currentPosition);
  }

  //public boolean getIntakeArrived() {
  //  return intakeArrived;
  //}

  public Command zeroIntake() {
    return runOnce(() -> {
      encoder.setPosition(0);
    });
  }

  public double getHeight() {
    return encoder.getPosition();
  }

  public Command setDesiredHeight(States.PositionState state) {
    return runOnce(() -> {
      currentPosition = States.DesiredHeightMap.get(state).floatValue();
    });
  }

  public void goToHeight(double goal_position) {
    m_goal = new TrapezoidProfile.State(goal_position, 0);
    m_setpoint = profile.calculate(0.02, m_setpoint, m_goal);
    // Set the PID Controller to the corresponding angle for in/out
    heightPID.setReference(m_setpoint.position, ControlType.kPosition);
  }
}
