package com.team1678.lib.logger;

import java.util.ArrayDeque;
import java.util.ArrayList;

public class LogWriter {
    
    private final ArrayDeque<LogEntry> queue;
    private ArrayList<LogStorage> storage;

    public LogWriter(ArrayDeque<LogEntry> queue) {
        this.queue = queue;
    }

    public void updateStorage(ArrayList<LogStorage> storage) {
        this.storage = storage;
    }

    public void log() {
        if (queue.isEmpty()) {
            return;
        }
        
        LogEntry entry = queue.pop();

        LogStorage store = storage.get(entry.getTarget());

        if (entry.getValues() == null) {
            return;
        }

        store.writeData(entry.getValues());
    }


    public void close() {
        while (!queue.isEmpty()) {
            log();
        }
        storage.forEach((s) -> s.close());
    }
}
