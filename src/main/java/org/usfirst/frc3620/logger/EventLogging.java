package org.usfirst.frc3620.logger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.StreamHandler;

import edu.wpi.first.wpilibj.DriverStation;

public class EventLogging {

    // make some levels that correspond to the different SLF4J logging
    // methods. Have the mappings to the underlying j.u.l logging levels.
    public enum Level {
        TRACE(java.util.logging.Level.FINEST), //
        DEBUG(java.util.logging.Level.FINE), //
        INFO(java.util.logging.Level.INFO), //
        WARN(java.util.logging.Level.WARNING), //
        ERROR(java.util.logging.Level.SEVERE);

        java.util.logging.Level julLevel;

        Level(java.util.logging.Level _julLevel) {
            julLevel = _julLevel;
        }
    }

    /**
     * Get an SLF4J logger for a class. Set the underlying j.u.l logger to the
     * desired level.
     * 
     * @param clazz
     *            class for the logger
     * @param l
     *            Level that we want to log at
     * @return
     */
    static public org.slf4j.Logger getLogger(Class<?> clazz, Level l) {
        return getLogger(clazz.getName(), l);
    }

    /**
     * Get an SLF4J logger for a name. Set the underlying j.u.l logger to the
     * desired level.
     * 
     * @param sClazz
     *            name for the logger
     * @param l
     *            Level that we want to log at
     * @return
     */
    static public org.slf4j.Logger getLogger(String sClazz, Level l) {
        setup();
        // set up the underlying logger to log at the level we want
        java.util.logging.Logger julLogger = java.util.logging.Logger
                .getLogger(sClazz);
        julLogger.setLevel(l.julLevel);

        // and return the SLF4J logger.
        org.slf4j.Logger rv = org.slf4j.LoggerFactory.getLogger(sClazz);
        return rv;
    }
    
    /**
     * Log command starts and stops
     * 
     * TODO: update using Thread.currentThread().getStackTrace()
     * 
     * @param logger
     * 			  logger to log to.
     */
    public static void commandMessage (org.slf4j.Logger logger) {
  	  Throwable t = new Throwable();
  	  StackTraceElement[] stackTraceElement = t.getStackTrace();
  	  logger.info("command {}", stackTraceElement[1].getMethodName());
    }

    public static void commandMessage (org.slf4j.Logger logger, Object o) {
  	  Throwable t = new Throwable();
  	  StackTraceElement[] stackTraceElement = t.getStackTrace();
  	  logger.info("command {} with {}", stackTraceElement[1].getMethodName(), o);
    }

    public static String callerAndCallersCallerNames() {
      StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
      StackTraceElement callersCaller = stackTraceElements[3];
      StackTraceElement callersCallerCaller = stackTraceElements[4];
      return callersCaller.getClassName() + "." + callersCaller.getMethodName()
        + " from " + callersCallerCaller.getClassName() + "." + callersCallerCaller.getMethodName();
    }
    
    public static String myAndCallersNames() {
        StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
        StackTraceElement caller = stackTraceElements[2];
        StackTraceElement callersCaller = stackTraceElements[3];
        return caller.getClassName() + "." + caller.getMethodName()
          + " from " + callersCaller.getClassName() + "." + callersCaller.getMethodName();
    }

  /**
   * return a description of the call chain of the caller of this routine. the "level"s in the
   * parameters say what in the caller's chain to include in the description.
   * 0 is the caller of this routine; 1 is the caller's caller, etc.
   *
   * @param closest the first level up the chain to describe.
   * @param furthest the last level up the chain to describe.
   * @return a text description of the call chain
   */
    public static String callChain(int closest, int furthest) {
        StringBuilder sb = new StringBuilder();
        StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
        boolean needFrom = false;
        for (int level = closest + 2; level <= furthest + 2; level++) {
            StackTraceElement ste = stackTraceElements[level];
            if (needFrom) {
                sb.append(" from ");
            }
            sb.append(ste.getClassName() + "." + ste.getMethodName());
            int line = ste.getLineNumber();
            if (line >= 0) {
                sb.append("(line:" + line + ")");
            }
            needFrom = true;
        }
        return sb.toString();
    }
      
      /**
     * Write a warning message to the DriverStation.
     * 
     * @param message
     *            Message to log.
     */
    public static final void writeWarningToDS(String message) {
        if (DriverStation.isDSAttached()) {
        	DriverStation.reportWarning(message, false);
        }
    }

    /**
     * Create a String representation of an Exception.
     * 
     * @param t
     * @return
     */
    public static final String exceptionToString(Throwable t) {
        final StackTraceElement[] stackTrace = t.getStackTrace();
        final StringBuilder message = new StringBuilder(1000);
        final String separator = "===\n";
        final Throwable cause = t.getCause();

        message.append("Exception of type ").append(t.getClass().getName());
        message.append("\nMessage: ").append(t.getMessage()).append('\n');
        message.append(separator);
        message.append("   ").append(stackTrace[0]).append('\n');

        for (int i = 1; i < stackTrace.length; i++) {
            message.append(" \t").append(stackTrace[i]).append('\n');
        }

        if (cause != null) {
            final StackTraceElement[] causeTrace = cause.getStackTrace();
            message.append(" \t\tCaused by ")
                    .append(cause.getClass().getName());
            message.append("\n\t\tBecause: ")
                    .append(cause.getMessage());
            message.append("\n\t\t   ").append(causeTrace[0]);
            message.append("\n\t\t\t").append(causeTrace[2]);
            message.append("\n\t\t\t").append(causeTrace[3]);
        }

        return message.toString();
    }

    private static boolean setupDone = false;

    public static void setup() {
        setup(LoggingMaster.getLoggingDirectory());
    }

	/**
	 * Set up a j.u.l logger that will start logging to a file (with a timestamped
	 * name). The logging to the file will not start until the system time is set.
	 * 
	 * @param logDirectory
	 *            Directory to create the logging file in
	 */
	public static boolean useDriverStation = true;
	public static void setup(File logDirectory) {
		if (!setupDone) // quickly check to see if we are initialized
		{
			synchronized (EventLogging.class) // check slowly and carefully
			{
				if (!setupDone) {
					Logger rootLogger = Logger.getLogger("");
					// get all the existing handlers
					Handler[] handlers = rootLogger.getHandlers();
					// and remove them
					for (Handler handler : handlers) {
						rootLogger.removeHandler(handler);
					}

					// add the handlers we want
					Handler h;
					if (useDriverStation) {
						h = new DriverStationLoggingHandler();
						h.setLevel(Level.DEBUG.julLevel);
						rootLogger.addHandler(h);
					} else {
						h = new ConsoleHandler();
						h.setFormatter(new FormatterForFileHandler());
						h.setLevel(Level.DEBUG.julLevel);
						rootLogger.addHandler(h);
					}

					h = new MyFileHandler(logDirectory);
					h.setFormatter(new FormatterForFileHandler());
					h.setLevel(Level.DEBUG.julLevel);
					rootLogger.addHandler(h);

					setupDone = true;
				}
			}
		}
	}

    static class MyFileHandler extends StreamHandler {
        File logDirectory;
        FileOutputStream fileOutputStream = null;

        public MyFileHandler(File logDirectory) {
            super();
            this.logDirectory = logDirectory;
        }

        @Override
        public void publish(LogRecord arg0) {
            /*
             * if the file is not open, see if we can open it.
             */
            if (fileOutputStream == null) // quick check
            {
                synchronized (this) // take some overhead for the good check
                {
                    if (fileOutputStream == null) // deliberate check
                    {
                        // we'll get a null here if the clock is not yet set
                        Date timestamp = LoggingMaster.getTimestamp();

                        if (timestamp != null) {
                            String timestampString = LoggingMaster.convertTimestampToString(timestamp);
                            // cool, let's make a file to log to!
                            File logFile = new File(logDirectory,
                                    timestampString + ".log");
                            try {
                                fileOutputStream = new FileOutputStream(
                                        logFile);
                                setOutputStream(fileOutputStream);
                            } catch (IOException ex) {
                                ex.printStackTrace(System.err);
                            }

                        }
                    }
                }
            }

            // only log if we have a place to log to
            if (fileOutputStream != null) {
                super.publish(arg0);
                flush();
            }
        }

    }

    static class FormatterForFileHandler extends java.util.logging.Formatter {
        //
        // Create a DateFormat to format the logger timestamp.
        //
        private final DateFormat df = new SimpleDateFormat(
                "yyyy/MM/dd HH:mm:ss.SSS");

        public String format(LogRecord record) {
            StringBuilder builder = new StringBuilder(1000);
            // DateFormat objects are not thread-safe....
            synchronized (df) {
                builder.append(df.format(new Date(record.getMillis())))
                        .append(' ');
            }
            builder.append('[').append(record.getLoggerName()).append("] ");
            builder.append(record.getLevel()).append(" - ");
            builder.append(formatMessage(record));
            builder.append('\n');
            return builder.toString();
        }

        public String getHead(Handler h) {
            return super.getHead(h);
        }

        public String getTail(Handler h) {
            return super.getTail(h);
        }
    }

}
