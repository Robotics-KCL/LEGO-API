package main;

import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.RCXMotor;
import lejos.util.Delay;

public class Crane {
	
	// Sensor 1 Color sensor
	// 2 Sonar Vertical
	// 3 Sonar Horizontal
	// 4 Touch sensor
	
//	public NXTRegulatedMotor vMotor;
	public NXTRegulatedMotor horizontalMotor;
	public NXTRegulatedMotor gripperMotor;
	public RCXMotor vericalMotor;
	private static UltrasonicSensor heightSonar;

	
	public static void main(String[] args) {
		
		/* Example for controlling the crane */
		
		// Initialise crane
		Crane k = new Crane();
		
		// Move crane to a particular height
		k.moveHeightTo(24);
		
		// Slide the gripper under the plant and pick up
		k.moveGripper(-320);
		k.up(2000);
		k.moveGripper(320);
		
		// Move crane down to lowest level and drop plant
		k.moveHeightTo(9);
		k.moveGripper(-240*3);
		k.down(3000);
		k.moveGripper(240*3);
		
		
		// Collect another plant with the same process
		k.moveHeightTo(18);
		
		// Move gripper position horizontally to pick a different plant
		k.right(160);
		
		// Pick up and bring it down
		k.moveGripper(-320);
		k.up(2000);
		k.moveGripper(320);
		
		k.moveHeightTo(9);
		k.moveGripper(-240*3);
		k.down(3300);
		k.moveGripper(240*3);
	}
	
	/**
	 * Constructor to setup components for the crane
	 */
	public Crane(){
		vericalMotor = new RCXMotor(MotorPort.B);
		horizontalMotor = new NXTRegulatedMotor(MotorPort.A);
		gripperMotor= new NXTRegulatedMotor(MotorPort.C);
		heightSonar = new UltrasonicSensor(SensorPort.S2);
	}
	
	/**
	 * Moves the gripper back.
	 * positive degrees will move the arm back
	 * negative degrees will move the arm forward
	 * @param degrees
	 */
	public void moveGripper(int degrees){
		gripperMotor.rotate(degrees);
	}
	
	/**
	 * Moves the crane down for a set period(milliseconds)
	 * @param msTime
	 */
	public void down(int msTime){
		vericalMotor.setPower(100);
		vericalMotor.backward();
		Delay.msDelay(msTime);
		vericalMotor.stop();
	}
	
	/**
	 * Moves the crane up for a set period(milliseconds)
	 * @param msTime
	 */
	public void up(int msTime){
		vericalMotor.setPower(100);
		vericalMotor.forward();
		Delay.msDelay(msTime);
		vericalMotor.stop();
	}
	
	/**
	 * Moves the crane left by rotating the horizontal motor
	 * @param degrees degrees to turn motor
	 */
	public void left(int degrees){
		if (degrees > 0) 
			degrees *= -1;
		horizontalMotor.rotate(degrees);
	}
	
	/**
	 * Moves the crane right by rotating the horizontal motor
	 * @param degrees degrees to turn motor
	 */
	public void right(int degrees){
		if (degrees > 0) 
			degrees *= -1;
		horizontalMotor.rotate(-1 * degrees);
	}
	
	/**
	 * Move the crane position to the given height(cm)
	 * @param goal
	 */
	public void moveHeightTo(int goal){
		int error;
		int current = heightSonar.getDistance();
		  vericalMotor.setPower(100);
		  error = current - goal;

		  while(error !=  0){
			  current = heightSonar.getDistance();
			  error = current - goal;
			  System.out.println(error);
			  if(error<0){
				  vericalMotor.forward();
			  }
			  else if(error>0){
				  vericalMotor.backward();
			  }
		  }
		  vericalMotor.stop();		
	  
	}
	
	/**
	 * Get sonar reading of height sonar to find height
	 * @return height(cm)
	 */
	public int getSonar(){
		return heightSonar.getDistance();
	}
	
	
}
