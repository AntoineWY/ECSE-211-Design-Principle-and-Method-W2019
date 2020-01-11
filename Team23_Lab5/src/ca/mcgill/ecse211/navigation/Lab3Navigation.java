package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class is used to start lab 3 program, runs the entirety of navigation with
 * and without obstacle avoidance 
 * @author Antoine Wang
 * @author Tudor Gurau
 *
 */
public class Lab3Navigation {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.18;
	public static final double TRACK = 15.40;
	public static final double Tile_Size = 30.48;

	public static NavigationUS navig;
	public static Odometer odometer;
	public static Thread nav;
	
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		// implementation
		Display odometryDisplay = new Display(lcd); // No need to change

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Nav    | Nav    ", 0, 2);
			lcd.drawString("W/out  | With   ", 0, 3);
			lcd.drawString("Obstac | Obstac ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// The left and the right class basically uses the same navigation class
		// left option actually means right option without obstacle
		if (buttonChoice == Button.ID_LEFT) {

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			// Initiate navigation class and allocate a new thread to it
			navig=new NavigationUS (Tile_Size, leftMotor, rightMotor);
			nav=new Thread(navig);
			nav.start();
		}else if(buttonChoice == Button.ID_RIGHT) {
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			// Initiate navigation class and allocate a new thread to it
			navig=new NavigationUS (Tile_Size, leftMotor, rightMotor);
			nav=new Thread(navig);
			nav.start();
			
			
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
