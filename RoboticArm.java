import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;

/**
 * 
 * A simple API designed for an easier manipulation of the LEGO NXT Robotic Arm.
 * 
 * @author KCL ROBOTICS
 *
 */

public final class RoboticArm {
	private static final int ONE_CM = 21;
	private static TouchSensor touch = new TouchSensor(SensorPort.S1);
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
	 * The robot stops when the touch button is pressed or when 5 seconds are elapsed.
	 */
	public static void catchObject() {
		long oldTime = System.currentTimeMillis();
		Motor.A.forward();
		while(!touch.isPressed()) {
			Motor.A.forward();
			if(touch.isPressed() || System.currentTimeMillis() - oldTime > 5000) Motor.A.stop();
		}
	}
	/**
	 * Open the robotic arm.
	 */
	public static void openArm() {
		Motor.A.rotate(-500);
	}
}
