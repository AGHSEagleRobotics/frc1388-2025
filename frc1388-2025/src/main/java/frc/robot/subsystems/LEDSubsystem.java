package frc.robot.subsystems;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle m_candle; // CANdle canid is 42
  // private final boolean m_isOnRed;

  // LED Constants

  private static final int kSlotStart = 16;
  private static final int kSlotEnd = 46;
  CANdleConfiguration configOn;
  CANdleConfiguration configOff;

  /** Creates a new LEDSubsystem. */  
  public LEDSubsystem(CANdle candle) {

    m_candle = candle;

    // m_isOnRed = (DriverStation.getAlliance().get() == Alliance.Red);

    configOn = new CANdleConfiguration();
    configOn.withLED(new LEDConfigs().withStripType(StripTypeValue.RGB).withBrightnessScalar(2));
    

    configOff = new CANdleConfiguration();
    configOff.withLED(new LEDConfigs().withStripType(StripTypeValue.RGB).withBrightnessScalar(0));
    

    // setSolidWhite();

  } // end LEDSubsystem() constructor

  public void setSolidWhite(){
    m_candle.getConfigurator().apply(configOn);
    m_candle.setControl(
        new SolidColor(kSlotStart, kSlotEnd)
          .withColor(new RGBWColor(Color.kWhite).scaleBrightness(1))
          );
  }

  public void setShooterStrobe(){
      StrobeAnimation shooterStrobe = new StrobeAnimation(kSlotStart, kSlotEnd).withSlot(kSlotStart)
          .withColor(new RGBWColor(255, 255, 255, 255).scaleBrightness(.7)).withFrameRate(0.2);
      m_candle.setControl(shooterStrobe);
    }

  public void turnOffLEDS(){
    m_candle.getConfigurator().apply(configOff);
  } 


  // public void setFireAnimation(){
  //   // Fire animation
  //   m_candle.setControl(
  //       new FireAnimation(kSlotStart, kSlotEnd).withSlot(1)
  //           .withDirection(AnimationDirectionValue.Backward)
  //           .withCooling(0.4)
  //           .withSparking(0.5));
  // }

  // public void setRainbowAnimation(){
  //   m_candle.setControl(
  //       new RainbowAnimation(kSlotStart, kSlotEnd).withSlot(2)
  //           .withDirection(AnimationDirectionValue.Forward)
  //           .withFrameRate(200));
  // }

  // public void setStrobeAnimation(){

  // }

  // public void setFadeAnimation(){
  //   // Yellow-Green fade in animation
  //   SingleFadeAnimation fades = new SingleFadeAnimation(8, 46).withSlot(0)
  //       .withColor(new RGBWColor(247, 233, 0, 0).scaleBrightness(.5)).withFrameRate(0.2);
  //   m_candle.setControl(fades);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}