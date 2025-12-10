package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.random.RandomGenerator.LeapableGenerator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import static frc.robot.subsystems.Elevator.ledModifier;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 0;
  private static final int kLength = 142;

  private final double sideLength = 55;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final double percentEffective = sideLength/kLength;
  

  Distance ledSpacing = Meters.of(1 / 120.0);
  Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, percentEffective*ledModifier, Color.kBlack, 1-percentEffective*ledModifier, Color.kWhite);
  //Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.0, Color.kBlack, 1-0.0, Color.kWhite);
  LEDPattern mask = LEDPattern.steps(maskSteps);

  LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kAliceBlue, 0.25*percentEffective, Color.kAliceBlue, .5*percentEffective, Color.kAliceBlue, .75*percentEffective, Color.kAliceBlue, percentEffective, Color.kAliceBlue, 1-percentEffective, Color.kAliceBlue, 100-.75*percentEffective, Color.kAliceBlue, 100-.5*percentEffective, Color.kAliceBlue, 100-.25*percentEffective, Color.kAliceBlue));
  //LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kAliceBlue));
  LEDPattern pattern = steps;
  
  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    //setDefaultCommand(runPattern(LEDPattern.solid(Color.kGreen)).withName("Off"));
    // Apply the LED pattern to the data buffer
 
  }

  @Override
  public void periodic() {
    
    // Periodically send the latest LED color data to the LED strip for it to display
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kRed, 0.36, Color.kBlack, .575, Color.kRed);
    SmartDashboard.putNumber("ledModifier", ledModifier);
    //Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.0, Color.kBlack, 1-0.0, Color.kWhite);
    LEDPattern mask = LEDPattern.steps(maskSteps);

   //LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kAliceBlue, 0.25*percentEffective, Color.kYellowGreen, .5*percentEffective, Color.kCrimson, .75*percentEffective, Color.kDarkOrange, percentEffective, Color.kYellow, 1-percentEffective, Color.kRed, 100-.75*percentEffective, Color.kYellow, 100-.5*percentEffective, Color.kMagenta, 100-.25*percentEffective, Color.kPaleGreen));
   //LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kCrimson));
    //LEDPattern pattern = steps.mask(mask);
    mask.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}