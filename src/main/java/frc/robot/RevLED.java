package frc.robot;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RevLED{
    PWM ledController = new PWM(9);
    
    /**
     * Enables the PWM control
     */
    /*public void initLED(){
        //ledController.enablePWM(1965);
        ledController.
    }*/

    /**
     * Sets the Duty Cycle/Pulse Width of the PWM
     * @param dutyCycle LED Pattern Table can be found at: http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
     * Examples:
     * Red: 1805
     * Blue: 1935
     * Green: 1885
     * White: 1965
     * Breathe Blue: 1425
     * Breathe Gray: 1435
     */
    public void setDutyCycle(int dutyCycle){
        ledController.setRaw(dutyCycle);
        SmartDashboard.putNumber("LED DutyCycle", dutyCycle);
    }
}