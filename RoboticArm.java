
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

/**
 * 
 * A simple API designed for an easier manipulation of the LEGO NXT Robotic Arm.
 * 
 * @author KCL ROBOTICS
 *
 */

public final class RoboticArm {
	private static final int ONE_CM = 21;
	private static final int TOUCH_SENSOR = 1;
	private static final int SONAR = 2;
	private static final int LIGHT_SENSOR = 3;
	private static TouchSensor touch = new TouchSensor(SensorPort.S1);
	private static TouchSensor[] touchSensors = new TouchSensor[3];
	private static UltrasonicSensor[] sonars = new UltrasonicSensor[3];
	private static LightSensor[] lightSensors = new LightSensor[3];
	private static int[] sensors = new int[5];
	/**
	 * Add a touch sensor to a specified port.
	 * @param port port of the touch sensor
	 */
	public static void addTouchSensor(int port){
		if(port >= 2 && port <= 4 && sensors[port] == 0) {
			sensors[port] = TOUCH_SENSOR;
			if(port == 2) {
				touchSensors[port-2] = new TouchSensor(SensorPort.S2);
			}
			else if(port == 3) {
				touchSensors[port-2] = new TouchSensor(SensorPort.S3);
			}
			else if(port == 4) {
				touchSensors[port-2] = new TouchSensor(SensorPort.S4);
			}
		}
		else {
			System.out.println("Trying to add touch sensor on port " + port + " failed.");
			Delay.msDelay(2000);
		}
	}
	/**
	 * Add a sonar to a specified port.
	 * @param port port of the sonar
	 */
	public static void addSonar(int port) {
		if(port >= 2 && port <= 4 && sensors[port] == 0) {
			sensors[port] = SONAR;
			if(port == 2) {
				sonars[port-2] = new UltrasonicSensor(SensorPort.S2);
			}
			else if(port == 3) {
				sonars[port-2] = new UltrasonicSensor(SensorPort.S3);
			}
			else if(port == 4) {
				sonars[port-2] = new UltrasonicSensor(SensorPort.S4);
			}
		}
		else {
			System.out.println("Trying to add sonar on port " + port + " failed.");
			Delay.msDelay(2000);
		}
	}
	/**
	 * Add a light sensor to a specified port.
	 * @param port the port of the sensor
	 */
	public static void addLightSensor(int port) {
		if(port >= 2 && port <= 4 && sensors[port] == 0) {
			sensors[port] = LIGHT_SENSOR;
			if(port == 2) {
				lightSensors[port-2] = new LightSensor(SensorPort.S2);
			}
			else if(port == 3) {
				lightSensors[port-2] = new LightSensor(SensorPort.S3);
			}
			else if(port == 4) {
				lightSensors[port-2] = new LightSensor(SensorPort.S4);
			}
		}
		else {
			System.out.println("Trying to add light sensor on port " + port + " failed.");
			Delay.msDelay(2000);
		}
	}
	/**
	 * Gets the information about the state of the touch sensor.
	 * @param port the port of the touch sensor
	 * @return true, if the buttons is pressed, false, otherise
	 */
	public static boolean getTouchSensorPressed(int port) {
		if(port >= 2 && port <= 4 && sensors[port] == TOUCH_SENSOR) {
			return touchSensors[port-2].isPressed();
		}
		else {
			System.out.println("No touch sensor on port " + port);
			Delay.msDelay(2000);
		}
		return false;
	}
	/**
	 * Gets the distance read from the sonar.
	 * @param port the port of the sonar
	 * @return distance
	 */
	public static int getSonarDistance(int port) {
		if(port >= 2 && port <= 4 && sensors[port] == SONAR) {
			return sonars[port-2].getDistance();
		}
		else {
			System.out.println("No sonar on port " + port);
			Delay.msDelay(2000);
		}
		return 0;
	}
	/**
	 * Gets the light value of a sensor.
	 * @param port the port of the sensor
	 * @return light value
	 */
	public static int getLightValue(int port) {
		if(port >= 2 && port <= 4 && sensors[port] == LIGHT_SENSOR) {
			return lightSensors[port-2].readNormalizedValue();
		}
		else {
			System.out.println("No sonar on port " + port);
			Delay.msDelay(2000);
		}
		return 0;
	}
	/**
	 * Makes the robot go forward for a non-specified amount of time or distance.
	 */
	public static void goForward() {
		Motor.B.forward();
		Motor.C.forward();
	}
	/**
	 * Makes the robot go backward for a non-specified amount of time or distance.
	 */
	public static void goBackward() {
		Motor.B.backward();
		Motor.C.backward();
	}
	/**
	 * Stops the wheels.
	 */
	public static void stopWheels() {
		Motor.B.stop();
		Motor.C.stop();
	}
	/**
	 * Move the robot forward a given distance in centimetres.
	 * @param cm number of centimetres
	 */
	public static void moveRobotForward(int cm) {
		Motor.B.rotate(ONE_CM*cm,true);
		Motor.C.rotate(ONE_CM*cm);
	}
	/**
	 * Move the robot backward a given distance in centimetres.
	 * @param cm number of centimetres
	 */
	public static void moveRobotBackward(int cm) {
		Motor.B.rotate(-ONE_CM*cm,true);
		Motor.C.rotate(-ONE_CM*cm);
	}
	/**
	 * Set the wheel speed to a given speed.
	 * @param speed new speed for the wheels
	 */
	public static void setWheelSpeed(int speed) {
		Motor.B.setSpeed(speed);
		Motor.C.setSpeed(speed);
	}
	/**
	 * Set the arm speed to a given speed.
	 * @param speed new speed for the arm
	 */
	public static void setArmSpeed(int speed) {
		Motor.A.setSpeed(speed);
	}
	/**
	 * Rotate the robot to the left for a given number of degrees.
	 * @param degrees number of degrees to rotate
	 */
	public static void rotateRobotLeft(int degrees) {
		Motor.B.rotate(degrees*2,true);
		Motor.C.rotate(-degrees*2);
	}
	/**
	 * Rotate the robot to the right for a given number of degrees.
	 * @param degrees number of degrees to rotate
	 */
	public static void rotateRobotRight(int degrees) {
		Motor.C.rotate(degrees*2,true);
		Motor.B.rotate(-degrees*2);
	}
	/**
	 * Make the robot catch an object.
	 * The robot stops when the touch button is pressed or when 2 seconds are elapsed.
	 */
	public static void catchObject() {
		long oldTime = System.currentTimeMillis();
		Motor.A.forward();
		while(!touch.isPressed()) {
			Motor.A.forward();
			if(touch.isPressed() || System.currentTimeMillis() - oldTime > 2000) break;
		}
		Motor.A.stop();
	}
	/**
	 * Open the robotic arm.
	 */
	public static void openArm() {
		Motor.A.rotate(-500);
	}
}