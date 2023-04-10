package frc.lib.helpers;

public class NeopixelState {
	public int red;
	public int green;
	public int blue;

	/**
	 * Contain RGB values for the neopixels
	 * @param red
	 * @param green
	 * @param blue
	 */
	public NeopixelState(int red, int green, int blue) {
		this.red = red;
		this.blue = blue;
		this.green = green;
	}

	public int total(){
		return (this.red+this.blue+this.green);
	}

	@Override
	public String toString(){
		return String.format("NeopixelState(R=%d, G=%d, B=%d)", this.red, this.green, this.blue);
	}
}
