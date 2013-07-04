package ai;

import prototype1.ArduinoInstrumentArray;

/**
 * a wrapper around an instrument that also contains its location
 * @author fit-pc
 *
 */
public class RealWorldInstrument extends RealWorldObject {
	
	
	/**
	 * @param instrument
	 */
	public RealWorldInstrument(ArduinoInstrumentArray instrument) {
		this.instrument = instrument;
	}

	private ArduinoInstrumentArray instrument;

	/**
	 * @return the instrument
	 */
	public ArduinoInstrumentArray getInstrument() {
		return instrument;
	}

}
