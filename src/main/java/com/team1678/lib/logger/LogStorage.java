package com.team1678.lib.logger;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class LogStorage {

    private String filename;
    private String path;
    private FileWriter writer;
    
    private final List<String> headers;
    private final int size; 

    public LogStorage(String filename, List<String> headers) {
        this.filename = filename.toUpperCase();
        this.headers = headers;
        this.size = headers.size();
    }

    public void setPath(String targetPath) {
        this.path = targetPath + "/" + filename + "_LOGS.csv";
        try {
            if (writer != null) {
                writer.close();
            }

            writer = new FileWriter(path);
            // Write headers into logfile
            for (int i = 0; i < headers.size(); i++) {
                writer.write(headers.get(i));
                if (i != size - 1) {
                    writer.write(",");
                }
            }
            writer.write("\n");
            writer.flush();
        } catch (IOException e) {
            System.out.println("Unable to create file \"" + path + "\"");
            e.printStackTrace();
        }
    }

    public void writeData(String[] data) {
        if (writer == null) {
            LoggingSystem.getInstance().pathSet = false;
            return;
        }
        try {
            for (int i = 0; i < size; i++) {
                writer.write(data[i]);
                if (i != size - 1) {
                    writer.write(",");
                }
            }
            writer.write("\n");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void flush() {
        try {
            writer.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void close() {
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
