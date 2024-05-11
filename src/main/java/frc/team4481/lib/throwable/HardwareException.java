package frc.team4481.lib.throwable;

public class HardwareException extends Exception {
    /**
     * This exception is thrown when there are known errors in the hardware that needs to be documented in the code
     */
    private String code;

    /**
     *
     * @param code recognizable exception code
     * @param message detailed description
     */
    public HardwareException(String code, String message){
        super(message);
        this.setCode(code);
    }

    /**
     *
     * @param code recognizable exception code
     * @param message detailed description
     * @param cause when the exception is thrown by another exception
     */
    public HardwareException(String code, String message, Throwable cause) {
        super(message, cause);
        this.setCode(code);
    }

    /**
     *
     * @return recognizable exception code
     */
    public String getCode() {
        return code;
    }

    /**
     *
     * @param code recognizable exception code
     */
    public void setCode(String code) {
        this.code = code;
    }
}
