package com.team1678.lib.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOError;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.sql.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TimeZone;
import java.util.function.Supplier;

import com.team1678.frc2023.Robot;
import com.team1678.frc2023.loops.Loop;

import edu.wpi.first.wpilibj.Timer;

import java.lang.Object;

import static java.util.Map.entry;

public class LoggingSystem {

    // Constants
    private static final int kQueueCapacity = 4000;
    public final List<String> kDriveRoot = Arrays.asList("/media/sdb1/", "/media/sda1/", "/media/sdc1/");
    public final String kLogDirectory = "robotlogs/";

    // Class that holds the filewriter
    public final LogWriter mLogWriter;

    private static ArrayList<ArrayList<Supplier<String>>> mElements = new ArrayList<ArrayList<Supplier<String>>>();
    private static ArrayList<LogStorage> mStorage = new ArrayList<LogStorage>();
    private static ArrayDeque<LogEntry> mQueue = new ArrayDeque<LogEntry>(kQueueCapacity);

    // Directory of logging for this session
    private static File mSessionDirectory = null;

    // Directory of logging sessions
    public String mBaseDirectory;

    // Format for naming logs chronologically
    private static DateFormat dateFormat = new SimpleDateFormat(
            "dd_MMM_yy_hh_mm_ss_aa");

    // Time of enable relative to UTC
    private Date startTime;

    // Time of enable relative to FPGA
    public static double mStartTimestamp;

    // Use different directory if benchmarking in sim
    private final boolean isBenchmark;
    public boolean pathSet = false;

    public static boolean disableLogger = false;

    // Seperate loop for writing to files to not slow down main loop
    private class LoggingLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            mBaseDirectory = null;
            try {
                System.out.println("Starting Logger");

                // Create new directory for this session
                setDirectory();

                // Record timestamp of logging start
                mStartTimestamp = Timer.getFPGATimestamp(); 
            } catch (Exception e) {
                disableLogger = true;
            }

        }

        @Override
        public void onLoop(double timestamp) {
            if (disableLogger) {
                return;
            }
            try {
                if (!pathSet) {
                    setDirectory();
                } else {
                    // Write to files
                    mLogWriter.log();
                }
            } catch (Exception e) {
                disableLogger = true;
            }

        }

        @Override
        public void onStop(double timestamp) {
            if (disableLogger) {
                return;
            }
            System.out.println("Stopping Logger");
            try {
                writeMetaData();
            } catch (Exception e) {
                disableLogger = true;
            }

            if (!pathSet) {
                return;
            }

            // Reset for next logging session
            mQueue.clear();
            mLogWriter.close();
            mSessionDirectory = null;

        }
    }

    private void writeMetaData() {
        LogMetadata metadata = new LogMetadata();
        try {
            FileWriter write = new FileWriter(mSessionDirectory + "/matchinfo.txt");
            write.write(metadata.generateMetadataString());
            write.close();
        } catch (IOException e) {
            e.printStackTrace();
        }


    }

    private LoggingSystem(boolean benchmarking) {
        mLogWriter = new LogWriter(mQueue);
        dateFormat.setTimeZone(TimeZone.getTimeZone("PST"));
        isBenchmark = benchmarking;

    }

    // Overload for registerObject if we don't provide a file name
    public synchronized int registerObject(Class<?> loggedClass, Object loggedObject) {
        String name;
        if (isBenchmark) {
            name = "ANNOTATION_LOGGER";
        } else {
            name = loggedClass.getSimpleName();
        }
        return registerObject(loggedClass, loggedObject, name);
    }

    // Reflect over object and save suppliers to log
    public synchronized int registerObject(Class<?> loggedClass, Object loggedObject, String name) {

        ArrayList<String> headers = new ArrayList<String>();

        int subsystemIndex = mElements.size();
        mElements.add(new ArrayList<Supplier<String>>());

        for (Method method : loggedClass.getDeclaredMethods()) {
            if (!method.isAnnotationPresent(Log.class) || !LoggalbeHandler.containsKey(method.getReturnType())
                    || method.getParameterCount() > 0) {
                continue;
            }
            method.setAccessible(true);

            LoggableProcessor processor = LoggalbeHandler.get(
                    method.getReturnType().isEnum() ? String.class : method.getReturnType());

            processor.getLoggable(() -> {
                try {
                    return method.invoke(loggedObject);
                } catch (IllegalAccessException | InvocationTargetException e) {
                    e.printStackTrace();
                    return null;
                }
            }, subsystemIndex);
            headers.add(method.getName());
        }

        for (Field field : loggedClass.getDeclaredFields()) {
            if (!field.isAnnotationPresent(Log.class) || !LoggalbeHandler.containsKey(field.getType())) {
                continue;
            }
            field.setAccessible(true);

            LoggableProcessor processor = LoggalbeHandler.get(
                    field.getType().isEnum() ? String.class : field.getType());
            
            processor.getLoggable(() -> {
                try {
                    return field.get(loggedObject);
                } catch (IllegalAccessException e) {
                    return null;
                }
            }, subsystemIndex);
            headers.add(field.getName());
        }

        LogStorage store = new LogStorage(name, headers);
        mStorage.add(store);
        return subsystemIndex;
    }

    // Create new logging directory for this session
    public void setDirectory() {
        if (mBaseDirectory == null) {
            // Check if directories are actually real
            if (isBenchmark || Robot.isSimulation()) {
                mBaseDirectory = "./Output Logs";
            } else {
                for (String potentialRoot : kDriveRoot) {

                    // Test the potential directory by writing a 'iotest/' directory then deleting it
                    // Yes this is the only way to do this
                    // Also should catch other IO errors before they result in a runtime crash
                    File testDir = new File(potentialRoot + "iotest/");
                    System.out.println("Checking " + potentialRoot + " for IO errors");
                    try {
                        testDir.mkdir();
                        if (!testDir.exists()) {
                            System.out.println(potentialRoot + " has an IO error, potentially unmounted incorrectly. Skipping and looking for another drive.");
                            continue;
                        }
                        testDir.delete();
                    } catch (IOError e) {
                        System.out.println(potentialRoot + " has an IO error, potentially unmounted incorrectly. Skipping and looking for another drive.");
                        continue;
                    }
                    mBaseDirectory = potentialRoot + kLogDirectory;
                    System.out.println("Chose the location for logs: " + mBaseDirectory);
                    break;
                }
                if (mBaseDirectory == null) {
                    System.out.println("Couldn't find a usable location to store logs. Disabling logger.");
                    disableLogger = true;
                    return;
                }
            }
        }
        String path = mBaseDirectory;

        if (!isBenchmark || Robot.isReal()) {
            // create logs folder if not present
            File rootDirectory = new File(mBaseDirectory);
            if (!rootDirectory.isDirectory()) {
                rootDirectory.mkdir();
            }

            // count up previous logging session and number this session accordingly
            Integer maxNum = 0;
            for (final File entry : rootDirectory.listFiles()) {
                try {
                    if (!entry.isDirectory()) {
                        continue;
                    }
                    String directory_name = entry.getName();
                    int char_index = directory_name.indexOf(")");
                    int num = Integer.parseInt(directory_name.substring(1, char_index));
                    if (num > maxNum) {
                        maxNum = num;
                    }
                } catch (Exception e) {
                    // Files that are not numbers are expected and ignored
                }
            }
            maxNum++;

            // get system time in milliseconds and convert to datetime
            startTime = new Date(System.currentTimeMillis());

            // format time in datetime and add to file name
            path = mBaseDirectory + "/(" + maxNum.toString() + ") " + dateFormat.format(startTime);

        }

        // create new directory
        mSessionDirectory = new File(path);
        mSessionDirectory.mkdir();

        if (mSessionDirectory.isDirectory()) {
            // update filewriters
            for (int i = 0; i < mStorage.size(); i++) {
                mStorage.get(i).setPath(path);
            }

            // Pass new filewriters handler
            mLogWriter.updateStorage(mStorage);
            
            pathSet = true;
        }
    }

    // Pull from suppliers and queue values for logging
    public void queueLogs() {
        for (int i = 0; i < mElements.size(); i++) {
            ArrayList<Supplier<String>> values = mElements.get(i);
            String[] temp = new String[values.size()];
            for (int j = 0; j < values.size(); j++) {
                temp[j] = values.get(j).get();
            }
            mQueue.add(new LogEntry(i, temp));
        }
    }

    public int queueSize() {
        return mQueue.size();
    }

    public boolean isDone() {
        return mQueue.isEmpty();
    }

    private static LoggingSystem mInstance;

    public static LoggingSystem getInstance() {
        if (mInstance == null) {
            mInstance = new LoggingSystem(false);
        }
        return mInstance;
    }

    public LoggingLoop Loop() {
        return new LoggingLoop();
    }

    @FunctionalInterface
    public interface LoggableProcessor {
        public void getLoggable(Supplier<Object> supplier, int subsystem);
    }

    public static Map<Class<?>, LoggableProcessor> LoggalbeHandler = Map.ofEntries(
            entry(int.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return String.valueOf(supplier.get());
                        });
                    }),
            entry(boolean.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return String.valueOf(supplier.get());
                        });
                    }),
            entry(String.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return (String) supplier.get();
                        });
                    }),
            entry(double.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            String ret = String.valueOf(supplier.get());
                            return ret.equals("NaN") ? "" : ret;
                        });
                    }));

}
