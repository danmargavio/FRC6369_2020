package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.GenericHID.RumbleType;

class Control{
  //private boolean mode = true;
  public int dPad;
  private ArrayList<Boolean> buttons;
  private ArrayList<Double> Axis;
  private Joystick C = null;

  public Control(int portNumber){
    C = new Joystick(portNumber);
    startList();
  }

  public void startList(){
    //Start List of Buttons
    buttons = new ArrayList<Boolean>();
    for(int i = 0; i < 10; i++){
      buttons.add(false);
    }
    // Start a list of axes
    Axis = new ArrayList<>();
    for(int i = 0; i < 6; i++){
      Axis.add(0.0);
    }
    dPad = -1;
  }

  public void refresh(){
    for(int i = 1; i < buttons.size(); i++){
      if(i == 1 || i == 2 || i == 7|| i == 8)
        buttons.set(i - 1, C.getRawButtonPressed(i));
      else
        buttons.set(i - 1, C.getRawButton(i));
    }
    //
    for(int i = 0; i < Axis.size(); i++){
      Axis.set(i, C.getRawAxis(i));
    }
    //
    dPad = C.getPOV();
  }

  public boolean getButton(int pos){
    return buttons.get(pos - 1);
  }

  public double getAxis(int pos){
    try{
      return Axis.get(pos);
    }
    catch(Exception e){
      try{
        return Axis.get(pos - 1);
      }
      catch(Exception f){
        System.out.print("fix getAxis");
        return 0.0;
      }
    }
  }
}