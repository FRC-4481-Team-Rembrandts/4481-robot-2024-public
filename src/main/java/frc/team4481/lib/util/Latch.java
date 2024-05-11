package frc.team4481.lib.util;

/**
 * Wrapper for easy access to latch functionality
 */
@Deprecated
public class Latch {
	private boolean enabled;

	/**
	 * Creates a new Latch
	 */
	public Latch(){
		enabled = false;
	}

	/**
	 * Creates a new latch with initial state
	 *
	 * @param start initial state of the latch
	 */
	public Latch(boolean start) {
		enabled = start;
	}

	/**
	 * Gets the current state of the latch
	 *
	 * @return current state
	 */
	public boolean get() {
		return enabled;
	}

	/**
	 * Enables the latch
	 */
	public void set() {
		enabled = true;
	}

	/**
	 * Disables the latch
	 */
	public void reset() {
		enabled = false;
	}

	/**
	 * Switches the current state of the latch
	 */
	public void latch() {
		enabled = !enabled;
	}
}