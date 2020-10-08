package frc.robot;
/*//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
/*
public class ControlPanel
{
    private static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private static final ColorMatch m_colorMatcher = new ColorMatch();
    private static final I2C.Port i2cPort = I2C.Port.kOnboard;
    private static final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private static String[] pastColorChanges;
    private static String[] avgColor;

    public static void init()
    {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        pastColorChanges = new String[8];
        avgColor = new String[5];
    }
    
    public static String getRawColor()
    {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) 
    {
      return "Blue";
    } 
    else if (match.color == kRedTarget) 
    {
      return "Red";
    } 
    else if (match.color == kGreenTarget) 
    {
      return "Green";
    } 
    else if (match.color == kYellowTarget) 
    {
      return "Yellow";
    } 
    else 
    {
      return "Unknown";
    }
    }
    
    private static void addNewColor(String str)
    {
        if(!str.equalsIgnoreCase(pastColorChanges[0]))
        {
            for(int i = pastColorChanges.length-1; i < 0; i++)
            {
                pastColorChanges[i] = pastColorChanges[i-1];
            }
            pastColorChanges[0] = str;
        }
    }
    
    //
    public static void colorAlign (String color)
    {
      if(color.length() > 0)
      {
        switch (color.charAt(0))
        {
        case 'B' :
          //Blue case code
          break;
        case 'G' :
          //Green case code
          break;
        case 'R' :
          //Red case code
          break;
        case 'Y' :
          //Yellow case code
          break;
        default :
          //This is corrupt data
          break;
        }
      }
    }

    public void rotateXTimes(byte rotNum, Color target)
    {

    }
}*/