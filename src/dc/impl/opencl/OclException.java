package dc.impl.opencl;

public class OclException extends Exception{
    public OclException() {
    }

    public OclException(String message) {
        super(message);
    }

    public OclException(String message, Throwable cause) {
        super(message, cause);
    }

    public OclException(Throwable cause) {
        super(cause);
    }
}
