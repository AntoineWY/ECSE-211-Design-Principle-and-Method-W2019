package ca.mcgill.ecse211.lab5;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

public class Lab5 {

  public static final double WHEEL_RAD = 2.16;
  public static final double TRACK = 13.4;
  public static final double TILE = 30.48;
  
  // r, g, b in order
  public static final double[] BLUE_COLOR = {0.19, 0.40, 0.87}; // value of blue colour
  public static final double[] GREEN_COLOR = {0.35, 0.85, 0.39}; // value of green colour
  public static final double[] YELLOW_COLOR = {0.85, 0.52, 0.09}; // value of yellow colour
  public static final double[] RED_COLOR = {0.98, 0.24, 0.07}; // value of red colour

  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  public static boolean isColorDetection;
  
  
  /**
   * The instance of the medium EV3 motor that controls the turning of the ultrasonic sensor. The
   * motor is connected to port C on the EV3 brick.
   */

  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  private static final Port US_PORT = LocalEV3.get().getPort("S2");

  private static final Port CS_PORT = LocalEV3.get().getPort("S4");

  private static final Port COLOR_PORT = LocalEV3.get().getPort("S3");

  private static final TextLCD LCD = LocalEV3.get().getTextLCD();

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  @SuppressWarnings("deprecation")
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice; // variable to record button clicked by user

    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    Display EV3Display = new Display(LCD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    @SuppressWarnings("resource")
    SensorModes lightSensor = new EV3ColorSensor(COLOR_PORT);
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[] lightData = new float[3];

    // CS sensor initialization
    @SuppressWarnings("resource")
    SensorModes csSensor = new EV3ColorSensor(CS_PORT);
    SampleProvider cs = csSensor.getMode("Red");
    float[] csData = new float[cs.sampleSize()];

    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("   LS   |  Start  ", 0, 2);
      LCD.drawString("  TEST  |  Search ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);

    if (buttonChoice == Button.ID_LEFT) {
      // LS TESTING
/*
      isColorDetection = true;
      
      Thread odoThread = new Thread(odometer);
      Thread displayThread = new Thread(EV3Display);
      CanCalibrator cc = new CanCalibrator(lightColor, lightData);
      ColorDetection cd = new ColorDetection(usDistance, usData, lightColor, lightData, LCD);

      odoThread.start();
      displayThread.start();
      cd.start();

      while (buttonChoice != Button.ID_ESCAPE) {
        cc.Calibrate(); // take readings
      }
*/
      UltrasonicLocalizer UL = new UltrasonicLocalizer(buttonChoice, usDistance, usData, odometer);
      
      Navigation nav = new Navigation(odometer, usDistance, usData);
      nav.turnTo(90);
      System.exit(0); // terminate program

    } else {

      isColorDetection = false;
      
      Thread odoThread = new Thread(odometer);
      Thread displayThread = new Thread(EV3Display);
      
      do {
        LCD.clear();
        LCD.drawString("< Left  |  Right >", 0, 0);
        LCD.drawString("        |         ", 0, 1);
        LCD.drawString("Falling |  Rising ", 0, 2);
        LCD.drawString(" Edge   |   Edge  ", 0, 3);
        buttonChoice = Button.waitForAnyPress();
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
          && buttonChoice != Button.ID_ESCAPE);
      
      LCD.clear();
      odoThread.start();
      //displayThread.start();
      
      UltrasonicLocalizer UL = new UltrasonicLocalizer(buttonChoice, usDistance, usData, odometer);
      UL.localize();
      
      LightLocalizer LL = new LightLocalizer(cs, csData, usDistance, usData, odometer);
      LL.localize();
      
      Search search = new Search(odometer, usDistance, usData, lightColor, lightData,cs, csData);
      Thread searchThread = new Thread(search);
      searchThread.start();

      while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      }
      System.exit(0); // exit program after esc pressed
    }
  }
}
