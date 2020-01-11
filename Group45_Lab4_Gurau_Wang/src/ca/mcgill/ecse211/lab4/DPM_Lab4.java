package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
//import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.localization.*;


/**
 * 
 * @author Tudor Gurau
 * @author Antoine Wang
 *
 */

public class DPM_Lab4 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static boolean isRisingEdge = true;

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.19;
	public static final double TRACK = 15.35;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising |Falling ", 0, 2);
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// Select which routine to find theta.
		if (buttonChoice == Button.ID_LEFT) {
			isRisingEdge = true;
		} else {
			isRisingEdge = false;
		}

		// Start odometer and display threads and correction Threads.
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Create ultrasonic and light localizer objects.
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, isRisingEdge, usDistance);
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor);

		// perform the ultrasonic localization
		USLocalizer.localize();
		// Press escape button if early exit is wanted
		if(Button.waitForAnyPress() == Button.ID_ESCAPE) {
			System.exit(0);
		}
		// wait for down button to be pressed to perform light localization
		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		// perform the light sensor localization
		lightLocatizer.localize();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}