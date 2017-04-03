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

public class Kinematics {
	
	public NXTRegulatedMotor vMotor;
	public NXTRegulatedMotor hMotor;
	public NXTRegulatedMotor gMotor;
	public RCXMotor rcxMotor;
	private static UltrasonicSensor hSonar;

	
	public Kinematics(){
		rcxMotor = new RCXMotor(MotorPort.B);
		hMotor = new NXTRegulatedMotor(MotorPort.A);
		gMotor= new NXTRegulatedMotor(MotorPort.C);
		hSonar = new UltrasonicSensor(SensorPort.S2);
	}
	
	public void testGripper(int degrees){
		gMotor.rotate(degrees);
	}
	
	public void down(int msTime){
		rcxMotor.setPower(100);
		rcxMotor.backward();
		Delay.msDelay(msTime);
		rcxMotor.stop();
	}
	
	public void up(int msTime){
		rcxMotor.setPower(100);
		rcxMotor.forward();
		Delay.msDelay(msTime);
		rcxMotor.stop();
	}
	
	
	public void shift(int degrees){
		hMotor.rotate(degrees);
	}
	
	public void hMove(int goal){
		int error;
		int current = hSonar.getDistance();
		  rcxMotor.setPower(100);
	error = current - goal;

	  while(error !=  0){
		  current = hSonar.getDistance();
		  error = current - goal;
		  System.out.println(error);
		  if(error<0){
			  rcxMotor.forward();
		  }
		  else if(error>0){
			  rcxMotor.backward();
		  }
	  }
	  rcxMotor.stop();		
	  
	}
	
	public int getSonar(){
		return hSonar.getDistance();
	}
	
}
