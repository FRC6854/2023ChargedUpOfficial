package frc.lib.helpers;

public class ArmState {

	public double lowerarmPosition;
	public double upperarmPosition;
	public double grabberAnglePosition;

	/**
	 * Contain certain states the arm should be in for different positions
	 * 
	 * @param lowerarmPosition
	 * @param upperarmPosition
	 * @param grabberAnglePosition
	 */
	public ArmState(double lowerarmPosition, double upperarmPosition, double grabberAnglePosition) {
		this.lowerarmPosition = lowerarmPosition;
		this.upperarmPosition = upperarmPosition;
		this.grabberAnglePosition = grabberAnglePosition;
	}
}
