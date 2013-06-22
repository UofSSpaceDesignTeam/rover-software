package ai;

import prototype1.Instrument;

/**
 * a wrapper around an instrument that also contains its location
 * @author fit-pc
 *
 */
public class RealWorldInstrument extends RealWorldObject {
	
	
	/**
	 * @param instrument
	 */
	public RealWorldInstrument(Instrument instrument) {
		this.instrument = instrument;
	}

	private Instrument instrument;

	/**
	 * @return the instrument
	 */
	public Instrument getInstrument() {
		return instrument;
	}

}
