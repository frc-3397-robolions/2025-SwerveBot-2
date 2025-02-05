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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController pid;
  private double desiredVelocity = 0;

  public Intake() {

    motor = new SparkMax(kCANID, SparkMax.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .velocityConversionFactor(1);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0, 0, 0, 0.000175);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake", encoder.getVelocity());
    pid.setReference(desiredVelocity, ControlType.kVelocity);
  }

  public Command Intake() {
    return runEnd(() -> {
      desiredVelocity = 5820;
    }, () -> {
      desiredVelocity = 0;
    });
  }

  public Command Outtake() {
    return runEnd(() -> {
      desiredVelocity = -5820;
    }, () -> {
      desiredVelocity = 0;
    });
  }

  public Command autoShoot(double time) {
    return runEnd(() -> {
      desiredVelocity = 5820 * 0.75;
    }, () -> {
      desiredVelocity = 0;
    }).withTimeout(time);
  }
}
