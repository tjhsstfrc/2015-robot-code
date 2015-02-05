
package org.usfirst.frc.team3455.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
	RobotDrive robotDrive;
    Joystick leftStick;
    Joystick rightStick;

    // Channels for the wheels
//    final int frontLeftChannel	= 4;
//    final int rearLeftChannel	= 2;
//    final int frontRightChannel	= 3;
//    final int rearRightChannel	= 1;
    
    Talon frontLeft;
    Talon frontRight;
    Talon backLeft;
    Talon backRight;
    
    // The channel on the driver station that the joystick is connected to
    final int rightJoystickChannel	= 0;
    final int leftJoystickChannel	= 1;
    Compressor compressor;
    DoubleSolenoid solenoid;

    public Robot() {
    	frontLeft = new Talon(4);
    	frontRight = new Talon(2);
    	backLeft = new Talon(3);
    	backRight = new Talon(1);
        rightStick = new Joystick(rightJoystickChannel);
        leftStick = new Joystick(leftJoystickChannel);
        compressor = new Compressor(); //Plugged into Digital I/O 2nd # is pressure switch, 4th # is spike, other #s are module
        solenoid = new DoubleSolenoid(0,1);
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
        robotDrive.setSafetyEnabled(false);  
        System.out.println("Autonomous");
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
    	double ctrlThresh = .2;
    	double minPower = .05;
    	double maxPower = .75;
    	double recip = (1 - ctrlThresh);
    	double mult = (maxPower - minPower);
    	double forward;
    	double strafe;
    	double rotate;
    	double fAxis;
    	double sAxis;
    	double rAxis;
    	//boolean rotateLeft;
    	//boolean rotateRight;
    	double fDir;
    	double sDir;
    	double rDir;
    	double frontLeftPower;
    	double frontRightPower;
    	double backLeftPower;
    	double backRightPower;
    	
    	//System.out.println("operator control");
    	compressor.start();
    	
    	
    	while(isOperatorControl()&&isEnabled()){
    	
        //robotDrive.setSafetyEnabled(true);
    		ctrlThresh = .2;
      	     recip = 1-ctrlThresh;
      	     fAxis = rightStick.getRawAxis(0);
            sAxis = rightStick.getRawAxis(1);
            rAxis = -1 * leftStick.getRawAxis(0);
            
      	     fDir = fAxis/Math.abs(fAxis);
      	     sDir = sAxis/Math.abs(sAxis);
      	     rDir = rAxis/Math.abs(rAxis);
      	     
      	     forward = 0;
      	     rotate = 0;
      	     strafe = 0;
      	     
      	     if(Math.abs(fAxis) > ctrlThresh)
      		{forward = (fAxis-ctrlThresh*fDir)/recip;}
      	     if(Math.abs(sAxis) > ctrlThresh)
      		{strafe = (sAxis-ctrlThresh*sDir)/recip;}
      	     if(Math.abs(rAxis) > ctrlThresh)
      		{rotate = (rAxis-ctrlThresh*rDir)/recip;}
                 
      	     
      	     frontLeftPower = (-1*forward - rotate + strafe);
      	     fDir = frontLeftPower/Math.abs(frontLeftPower);
                   frontLeftPower = (Math.abs(frontLeftPower*mult) + minPower) * fDir;
                  
                   frontRightPower = (forward - rotate + strafe);
                   fDir = frontRightPower/Math.abs(frontRightPower);
                   frontRightPower = (Math.abs(frontRightPower*mult) + minPower) * fDir;

                   backLeftPower = (-1*forward - rotate - strafe);
                   fDir = backLeftPower/Math.abs(backLeftPower);
                   backLeftPower = (Math.abs(backLeftPower*mult) + minPower) * fDir;

                   backRightPower = (forward - rotate - strafe);
                   fDir = backRightPower/Math.abs(backRightPower);
                   backRightPower = (Math.abs(backRightPower*mult) + minPower) * fDir;

                   frontRight.set(frontRightPower);
                   frontLeft.set(frontLeftPower);
                  backRight.set(backRightPower);
                   backLeft.set(backLeftPower); 

      	    
    	
//    	System.out.println("Compressor Enabled: " + compressor.enabled());
//    	System.out.println("Compressor Pressure Gauge: " + compressor.getPressureSwitchValue());
    	if(rightStick.getRawButton(5)){
    		System.out.println("button 5");
 	       solenoid.set(DoubleSolenoid.Value.kForward);
    	}
    	else if(rightStick.getRawButton(4)){
 	    	System.out.println("button 4");
 	       solenoid.set(DoubleSolenoid.Value.kReverse);
 	    }
    	else{
    		solenoid.set(DoubleSolenoid.Value.kOff);
    	}
        Timer.delay(0.02);	// wait to avoid hogging CPU cycles
        
    	}
    }


}
