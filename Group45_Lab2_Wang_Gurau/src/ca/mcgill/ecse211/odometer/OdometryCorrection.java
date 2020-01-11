/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class contains the algorithm to adjust the odometer
 * During the square driving by resetting the values in the odometer
 * @author Yinuo A Wang
 * @author Tudor Gurau
 */
public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_LENGTH = 30.48;
	private static int horizontalL; // counter for black line in x-axis
	private static int verticalL; // counter for black line in y-axis
	

	private Odometer odometer;
	
	// Below are four variable associate with light sensor
	private static Port lsPort = LocalEV3.get().getPort("S1"); //create a port instance
    private SensorModes lightSensor;                           //Sensor instance                  
	private SampleProvider color;                              //Sample provider to get sample of data from sensor
	private float[] lsData;                                    //Data retrieved from sensor 

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	//Below are 4 constant describe how much the cart goes off from the a straight line
	//at positive y direction, positive x direction and so on
	//Note that on Y direction it corrects X value and on X direction it corrects Y
	//Numbers are experimentally determined
	private static final double POSY_COE = 0.98;
	private static final double POSX_COE = 0.98;
	private static final double NEGY_COE = 0.974;
	private static final double NEGX_COE = 0.94;

	//Below are two variables used for light sensor filtering
	private double preInten = 5;             
	private static final double RANGE = 0.5;
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.lightSensor = new EV3ColorSensor(lsPort);
	this.color = lightSensor.getMode("Red");
	this.lsData = new float[lightSensor.sampleSize()];
	horizontalL = 0; 
	verticalL = 0;

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

	float colorIntensity;
    odometer.setXYT(-(TILE_LENGTH)/2, -(TILE_LENGTH)/2, 0);

    while (true) {
    	
    	

    	//odometer.setXYT(0, 0, 0);
		correctionStart = System.currentTimeMillis();

		// TODO Trigger correction (When do I have information to correct?)
		// TODO Calculate new (accurate) robot position
		// TODO Update odometer with new calculated (and more accurate) values

		color.fetchSample(lsData, 0);
		colorIntensity = lsData[0];

		double intenDiff = Math.abs(colorIntensity - preInten); // Simple filter to correct outliers of light sensor
		//if(intenDiff<RANGE){
			preInten = colorIntensity;
		//}
		if (colorIntensity < 0.30 && intenDiff < RANGE) { // black lines reflect low intensity light
			
			Sound.beep();
		
			double[] odoData = odometer.getXYT(); // using get method from OdometerData
			double theta = odoData[2];		
		   
			
			if (theta < 20 || theta > 340) { // theta around 0 degrees; moving in positive y direction	
				verticalL ++; 
				odometer.setY((verticalL-1) * TILE_LENGTH ); 
				odometer.setX(odometer.getXYT()[0]* POSY_COE);
			}		
			if (theta < 100 && theta >= 80) { // theta around 90degrees; moving in positive x direction
				horizontalL ++;
				odometer.setX((horizontalL-1) * TILE_LENGTH  ); 
				odometer.setY(odometer.getXYT()[1]* POSX_COE);
			}
			if (theta < 200 && theta >= 160) { // theta around 180 degrees; moving in negative y direction
				
				verticalL --; 
				odometer.setY((verticalL) * TILE_LENGTH ); 
				odometer.setX(odometer.getXYT()[0]*NEGY_COE);              
			}

			if (theta < 285 && theta >= 255) { // theta around 270 degrees; moving in negative x direction				
				horizontalL = horizontalL - 1; 
				odometer.setTheta(odometer.getXYT()[2]+0.9);
				odometer.setX((horizontalL) * TILE_LENGTH +2); // Plus 2 here is taking in account of the distance sensor advances the car center
				odometer.setY(odometer.getXYT()[1]*NEGX_COE);
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
  }
}
