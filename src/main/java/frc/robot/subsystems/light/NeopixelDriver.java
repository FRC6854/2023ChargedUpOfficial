package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.NeopixelSet;
import frc.lib.helpers.NeopixelState;
import frc.lib.helpers.enums.TimeStates;
import frc.robot.Constants;

public class NeopixelDriver extends SubsystemBase {
	private final AddressableLED m_led; // LED (PWM) Object
	private final AddressableLEDBuffer m_ledBuffer; // Led Buffer
	public NeopixelState alliance_state; // Past Alliance colour
	public NeopixelState default_state; // Middle / Default status color
	private boolean blinked; // If has blinked will not blink next tick
	private int ticks; // # of ticks (resets to zero after Constants.NeopixelConstants.blinkWhen)
	public NeopixelState blinkColour = new NeopixelState(0, 0, 0);
	public boolean blinking; // Enable or disable blinking
	public TimeStates state; // Current Neopixel program state
	public TimeStates p_state; // Past Neopixel program state

	// INFO: Check docs
	// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

	/**
	 * Sets up Neopixel Subsystem
	 */
	public NeopixelDriver() {
		this.m_led = new AddressableLED(Constants.NeopixelConstants.pwmHeader);
		this.m_ledBuffer = new AddressableLEDBuffer(Constants.NeopixelConstants.pwmDrive);
		this.alliance_state = new NeopixelState(0, 0, 0);
		this.default_state = new NeopixelState(0, 0, 0);
		m_led.setLength(m_ledBuffer.getLength());
	}

	/**
	 * Start NeoPixel PWM and adds Green and Yellow to buffer and commits
	 */
	public void start() {
		m_led.start();
		System.out.println("-------------------");
		System.out.println(" NeoPixels Enabled ");
		System.out.println("-------------------");
	}

	/**
	 * Sets the two side alliance LEDS
	 * 
	 * @param allianceState
	 * @return if setAlliance has commited a new change
	 */
	public boolean setAlliance(NeopixelState allianceState) {
		for (var i = 0; i < Constants.NeopixelConstants.statusEnding; i++) { // Fill first alliance
			m_ledBuffer.setRGB(i, allianceState.red, allianceState.green, allianceState.blue);
		}
		for (var i = Constants.NeopixelConstants.firstEnding
				- Constants.NeopixelConstants.statusEnding; i < Constants.NeopixelConstants.firstEnding; i++) { // Fill
																												// second
																												// alliance
			m_ledBuffer.setRGB(i, allianceState.red, allianceState.green, allianceState.blue);
		}
		for (var i = Constants.NeopixelConstants.firstEnding
				+ Constants.NeopixelConstants.backBand; i < Constants.NeopixelConstants.pwmDrive; i++) { // Fill
																											// back
																											// alliance
			m_ledBuffer.setRGB(i, allianceState.red, allianceState.green, allianceState.blue);
		}
		alliance_state = allianceState;
		return true;
	}

	/**
	 * Sets the middle LEDs
	 * 
	 * @param allianceState
	 * @return if setDefault has commited a new change
	 */
	@Deprecated
	public boolean setDefault(NeopixelState defaultState){
		setDefaultNP(defaultState);
		default_state = defaultState;
		return true;
	}

	/* Sets Neopixel Default without setting history */
	private void setDefaultNP(NeopixelState defaultState) {
		for (var i = Constants.NeopixelConstants.statusEnding; i < Constants.NeopixelConstants.firstEnding
				- Constants.NeopixelConstants.statusEnding; i++) { // Fill front middle
			m_ledBuffer.setRGB(i, defaultState.red, defaultState.green, defaultState.blue);
		}
		for (var i = Constants.NeopixelConstants.firstEnding; i < Constants.NeopixelConstants.pwmDrive
				- Constants.NeopixelConstants.backBand; i++) { // Fill back middle
			m_ledBuffer.setRGB(i, defaultState.red, defaultState.green, defaultState.blue);
		}
	}

	/**
	 * Fills the whole strip with a single colour
	 * 
	 * @param fillState
	 */
	public void fillSolid(NeopixelState fillState) {
		setAlliance(fillState);
		setDefault(fillState);
	}

	/**
	 * Uses the older NeopixelSet to assign colours
	 * 
	 * @param set
	 */
	public void fillSet(NeopixelSet set) {
		setAlliance(set.allianceLED);
		setDefault(set.defaultLED);
	}

	/**
	 * Commits buffer to strip
	 */
	private void commit() {
		m_led.setData(m_ledBuffer);
	}

	/**
	 * Set a state for NeoPixel Properties to work with SubSystems
	 * 
	 * @param new_state
	 */
	public void setState(TimeStates new_state) {
		p_state = state;
		state = new_state;
	}

	/**
	 * Sets the Alliance LEDs to their proper alliance
	 */
	public void fillAlliance() {
		boolean commiter;
		switch (DriverStation.getAlliance()) {
			case Red:
				commiter = setAlliance(Constants.NeopixelConstants.redAllianceState);
				break;
			case Blue:
				commiter = setAlliance(Constants.NeopixelConstants.blueAllianceState);
				break;
			default:
				commiter = setAlliance(Constants.NeopixelConstants.errorState); // Give error message if something
																				// breaks
				break;
		}
		if (commiter) {
			commit();
		}
	}

	/**
	 * Sets the middle LEDs to their proper alliance
	 */
	public void fillDefault() {
		boolean commiter;
		switch (DriverStation.getAlliance()) {
			case Red:
				commiter = setDefault(Constants.NeopixelConstants.redAllianceState);
				break;
			case Blue:
				commiter = setDefault(Constants.NeopixelConstants.blueAllianceState);
				break;
			default:
				commiter = setAlliance(Constants.NeopixelConstants.errorState); // Give error message if something
																				// breaks
				break;
		}
		System.out.println("[NeoPixels] Default: " + commiter);
	}

	@Override
	public void periodic() {
		ticks++;
		if (ticks >= Constants.NeopixelConstants.blinkWhen) {
			ticks = 0;
			if (!blinked && blinking) {
				setDefaultNP(blinkColour);
				blinked = true;
			} else {
				setDefaultNP(default_state);
				blinked = false;
			}
			fillAlliance();
			commit();
		}
	}
}
