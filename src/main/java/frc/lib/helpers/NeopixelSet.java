package frc.lib.helpers;

public class NeopixelSet {
	public NeopixelState allianceLED; // For a LED that will stay the alliance color
	public NeopixelState defaultLED; // Default LED strips

	/**
	 * Neopixel Sets for better presence
	 * @param allianceLED
	 * @param defaultLED
	*/
	public NeopixelSet(NeopixelState allianceLED, NeopixelState defaultLED){
		this.allianceLED = allianceLED;
		this.defaultLED = defaultLED;
	}
}