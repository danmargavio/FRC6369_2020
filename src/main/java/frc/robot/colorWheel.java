package frc.robot;

/*import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.revrobotics.ColorMatch;

/*
public class colorWheel {
	///**
	//* Change the I2C port below to match the connection of your color sensor
	//
	//private final I2C.Port i2cPort = I2C.Port.kOnboard;

	///**
	//* A Rev Color Sensor V3 object is constructed with an I2C port as a 
	//* parameter. The device will be automatically initialized with default 
	//* parameters.
	//
	//private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

	/**
	* A Rev Color Match object is used to register and detect known colors. This can 
	* be calibrated ahead of time or during operation.
	* 
	* This object uses a simple euclidian distance to estimate the closest match
	* with given confidence range.
	
	//private final ColorMatch m_colorMatcher = new ColorMatch();

	///**
	//* Note: Any example colors should be calibrated as the user needs, these
	//* are here as a basic example.
	//

	// Color wheel
	private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
	private byte numTimesTargetColorSeen;
	private byte timesToRotate;
	//private TalonSRX rotater = new TalonSRX(10);
	private String targetColor;
	private String currentColor;
	private String lastColor;
	private ColorMatch match;
	private String colorString;
	public colorWheel()
	{
		///**
    	//* The method GetColor() returns a normalized color value from the sensor and can be
    	//* useful if outputting the color to an RGB LED or similar. To
    	//* read the raw color, use GetRawColor().
    	//* 
    	//* The color sensor works best when within a few inches from an object in
    	//* well lit conditions (the built in LED is a big help here!). The farther
    	//* an object is the more light from the surroundings will bleed into the 
    	//* measurements and make it difficult to accurately determine its color.
    	//
		Color detectedColor = m_colorSensor.getColor();
    	///**
    	//* Run the color match algorithm on our detected color
    	//
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    	///*
    	if (match.color == kBlueTarget) 
    	{
    		colorString = "Blue";
      
    	} 
    	else if (match.color == kRedTarget) 
    	{
    		colorString = "Red";
    	} 
    	else if (match.color == kGreenTarget) 
    	{
    		colorString = "Green";
    	}
    	else if (match.color == kYellowTarget) 
    	{
    		colorString = "Yellow";
    	} 
    	else 
    	{
    		colorString = "Unknown";
    		//System.err.print("Error: color \"unknown\" read");
    	}
    	currentColor = colorString;
    	///*
    	if(!currentColor.equals(lastColor) && currentColor.equals(targetColor))
    	{
    		numTimesTargetColorSeen++;
    	}

    	if(numTimesTargetColorSeen/2 > timesToRotate)
    		rotater.set(ControlMode.PercentOutput, 0.30);
    	else
    		rotater.set(ControlMode.PercentOutput, 0);
    	///**
    	//* Open Smart Dashboard or Shuffleboard to see the color detected by the 
    	//* sensor.
    	//
    	//
    	SmartDashboard.putNumber("Red", detectedColor.red);
    	SmartDashboard.putNumber("Green", detectedColor.green);
    	SmartDashboard.putNumber("Blue", detectedColor.blue);
    	SmartDashboard.putNumber("Confidence", match.confidence);
    	SmartDashboard.putString("Detected Color", colorString);

    	SmartDashboard.putNumber("Times I've seen target color", numTimesTargetColorSeen);


    	lastColor = currentColor; //
    	//if(!colorString.equalsIgnoreCase("red"))
    		//rotater.set(ControlMode.PercentOutput, 0.00);
    	//else
    		//rotater.set(ControlMode.PercentOutput, 0.00);

      SmartDashboard.putString("Raw Color", ControlPanel.getRawColor());
	
		m_colorMatcher.addColorMatch(kBlueTarget);
    	m_colorMatcher.addColorMatch(kGreenTarget);
    	m_colorMatcher.addColorMatch(kRedTarget);
    	m_colorMatcher.addColorMatch(kYellowTarget);
    	numTimesTargetColorSeen = 0;
    	timesToRotate = 4;

		ControlPanel.init();
	}
}*/
