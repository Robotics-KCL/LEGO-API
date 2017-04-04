import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;
import lejos.nxt.BasicMotorPort;
import lejos.nxt.BasicMotor;
import lejos.nxt.addon.RCXMotor;
import lejos.robotics.BaseMotor;
import lejos.robotics.DCMotor;
import lejos.nxt.TouchSensor;
import lejos.nxt.LightSensor;
import lejos.nxt.Sound;





public class Kinematics {
	//Height for Dropping off pot, approx 9
	//Height for Shelf 1, approx 20
	//Height for shelf 2, approx 25
	//Distance between horizontal pots, approx 130 degrees
	
	public RCXMotor vMotor; //parallel motor at each end of the crane for making it go up and down 
	//Note that the vMotor only has the methods: .forward(), .backward(), .setPower(0-100%)
	public NXTRegulatedMotor gMotor; //The Motor for making the griper slide in and out, all the typical methods are available
	public NXTRegulatedMotor hMotor; //Motor for making the gripper move horizontally (side to side) on the crane
	public UltrasonicSensor vSonar; //Vertical sonar for detecting the height of the robot
	public TouchSensor hButton; // horizontal sonar for detecting the position of the gripper and NXt unit along the crane
	public TouchSensor gripperButton; //Button is pressed when the gripper fully retracted
	public LightSensor light;
	
	public Kinematics(){
		vMotor = new RCXMotor(MotorPort.B);
		hMotor = new NXTRegulatedMotor(MotorPort.A);
		gMotor= new NXTRegulatedMotor(MotorPort.C);
		vSonar = new UltrasonicSensor(SensorPort.S2);
		hButton = new TouchSensor(SensorPort.S3);
		gripperButton = new TouchSensor(SensorPort.S4);
		light = new LightSensor(SensorPort.S1);
	}
	
	
	/**
	 * Moves the gripper by rotating the motor by an amount you select.
	 * positive degrees will move the arm back
	 * negative degrees will move the arm forward
	 * @param degrees rotation of motor
	 */
	public void moveGripper(int degrees){
		gMotor.rotate(degrees); 
	}
	
	/**
	 * Fully retract the gripper
	 * @param degrees
	 */
	public void retractGripper(){
		while(!gripperButton.isPressed()){
			gMotor.forward(); // Forward rotation of the motor causes the gripper to retract
		}
		gMotor.stop();
	}
	
	/**
	 * Makes Crane Beep
	 */
	public void beep(){
		Sound.beep();
	}
	
	/**
	 * Makes the Crane go to its home position
	 */
	public void home(){
		while(!hButton.isPressed()){
			hMotor.forward(); // Forward rotation of the motor causes the gripper to retract
		}
		hMotor.stop();
		vMove(8);
	}
	
	/**
	 * Moves the crane left by rotating the horizontal motor
	 * @param degrees degrees to turn motor
	 */
	public void down(int msTime){
		vMotor.setPower(100);
		vMotor.backward();
		Delay.msDelay(msTime);
		vMotor.stop();
	}
	
	
	/**
	 * Moves the crane up for a set period(milliseconds)
	 * @param msTime
	 */
	public void up(int msTime){
		vMotor.setPower(100);
		vMotor.forward();
		Delay.msDelay(msTime);
		vMotor.stop();
	}
	
	
	
	/**
	 * Moves the crane left by rotating the horizontal motor
	 * @param degrees degrees to turn motor
	 */
	public void left(int degrees){
		if (degrees > 0) 
			degrees *= -1;
		hMotor.rotate(degrees);
	}
	
	/**
	 * Moves the crane right by rotating the horizontal motor
	 * @param degrees degrees to turn motor
	 */
	public void right(int degrees){
		if (degrees > 0) 
			degrees *= -1;
		hMotor.rotate(-1 * degrees);
	}

	/**
	 * Move the crane position to the given height(cm)
	 * @param goal
	 */
	public void vMove(int goal){
		int error;
		int current = vSonar.getDistance();
		vMotor.setPower(100);
		error = current - goal;
			while(error !=  0){
				current = vSonar.getDistance();
				error = current - goal;
				System.out.println(error);
				if(error<0){
					vMotor.forward();
				}
				else if(error>0){
					vMotor.backward();
				}
		}
		vMotor.stop();
	}

	
	
	/**
	 * Get sonar reading of height sonar to find height
	 * @return height(cm)
	 */
	public int getVSonar(){
		return vSonar.getDistance();
	}
	

	/**
	 * Returns the value read by the light sensor 
	 * @return
	 */
	public int getValue(){
		return light.getLightValue();
	}
	
	
}
