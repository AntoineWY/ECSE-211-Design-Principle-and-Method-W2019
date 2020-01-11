package ca.mcgill.ecse211.navigation;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;

/**
 * This class controls the sweeps of the sensor and the initialization of sensor to its starting position
 * @author Antoine Wang
 * @author Tudor Gurau
 *
 */
public class SweepUS extends Thread {
	private EV3MediumRegulatedMotor sensorMotor;
	private static boolean isRotating = true;
	private static boolean upright = false;
	private static final int SWEEP_ANGLE = 35;
	
	/**
	 * Class constructor, pass the sensor motor
	 * @param sensormotor2
	 */
	public SweepUS(EV3MediumRegulatedMotor sensormotor2){
		this.sensorMotor = sensormotor2;
	}
	
	/**
	 * set the flag controlling the sweeping to false, so the sweeping stops
	 */
	public void stopSensor(){
		isRotating = false;	
	}
	
	/**
	 * set the flag controlling the sweeping to true, so the sweeping starts
	 */
	public void startSensor(){
		isRotating = true;
	}
	
	/**
	 * This method defines the whether the sensor should be reset upright
	 */
	public void setPos(){
		upright = true;
	}
    
	
	public void run(){
		sensorMotor.setSpeed(50);
		while(true){
			while(isRotating){	   // this while loop controls whether the sensor is sweeping
				sensorMotor.rotate(SWEEP_ANGLE);		
				sensorMotor.rotate(-2*SWEEP_ANGLE);
				sensorMotor.rotate(SWEEP_ANGLE);
			}
			while(upright){       // this while loop set the sensor to face front only once
				
				sensorMotor.rotateTo(0, false);
				upright = false;
				
				}
		}
		
	}
}