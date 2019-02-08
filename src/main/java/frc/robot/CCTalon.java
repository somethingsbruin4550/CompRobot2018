package frc.robot;

import edu.wpi.first.wpilibj.Talon;

public class CCTalon extends Talon{
	private boolean _reverse;

	public CCTalon(int channel, boolean reverse) {
		super(channel);
		_reverse = reverse;
	}
	
	public void set(double speed) {
		if (_reverse) {
			super.set(-speed);
		}
		else {
			super.set(speed);
		}
	}
}
