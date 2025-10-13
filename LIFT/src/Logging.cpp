#include <Arduino.h>
#include <SdFat.h>
#include "Logging.h"

static SdFat sd;
static SdFile log_file;
static SdFile read_file;
static char log_filename[20];
static uint8_t chip_select_pin = 5;

// Internal helpers
static bool create_new_log_file();
static void write_line(const char *level, const char *message);
static void get_timestamp(char *buffer, size_t size);
static bool find_last_log_file(char *filename, size_t size);

// === Initialization ===
bool sd_init(uint8_t cs_pin) {
    chip_select_pin = cs_pin;
    if (!sd.begin(chip_select_pin, SD_SCK_MHZ(10))) {
        Serial.println("SD init failed");
        return false;
    }

    if (!create_new_log_file()) {
        Serial.println("Failed to create log file");
        return false;
    }

    Serial.print("Logging to: ");
    Serial.println(log_filename);
    return true;
}

// === Writing ===
void sd_log(const char *message) {
    write_line("LOG", message);
}

void sd_log_info(const char *message) {
    write_line("INFO", message);
}

void sd_log_error(const char *message) {
    write_line("ERROR", message);
}

void sd_log_raw(const char *message) {
    write_line("", message);
}

void sd_close(void) {
    if (log_file.isOpen()) {
        log_file.close();
    }
}

// === Reading ===
bool sd_open_for_read(const char *filename) {
    if (read_file.isOpen()) read_file.close();

    if (!sd.exists(filename)) {
        Serial.println("File not found");
        return false;
    }

    if (!read_file.open(filename, O_READ)) {
        Serial.println("Failed to open file for read");
        return false;
    }

    return true;
}

bool sd_read_line(char *buffer, size_t size) {
    if (!read_file.isOpen()) return false;

    int i = 0;
    int c;
    while ((c = read_file.read()) >= 0 && c != '\n' && i < (int)size - 1) {
        buffer[i++] = (char)c;
    }
    buffer[i] = '\0';

    // EOF check
    if (c < 0 && i == 0) {
        return false;
    }

    return true;
}

void sd_close_read(void) {
    if (read_file.isOpen()) read_file.close();
}

// === Utility ===
bool sd_get_last_filename(char *buffer, size_t size) {
    return find_last_log_file(buffer, size);
}

void sd_list_files(void) {
    Serial.println("Files on SD:");
    SdFile dir;
    dir.open("/");
    SdFile entry;

    while (entry.openNext(&dir, O_READ)) {
        if (!entry.isDir()) {
            char name[50];
            entry.getName(name, sizeof(name));
            Serial.println(name);
        }
        entry.close();
    }
    dir.close();
    Serial.println("End of file list.\n");
}

bool sd_read_file(const char *filename) {
    if (!sd.exists(filename)) {
        Serial.print("File not found: ");
        Serial.println(filename);
        return false;
    }

    SdFile f;
    if (!f.open(filename, O_READ)) {
        Serial.println("Failed to open file for reading");
        return false;
    }

    Serial.print("=== Reading file: ");
    Serial.print(filename);
    Serial.println(" ===");

    int c;
    while ((c = f.read()) >= 0) {
        Serial.write((char)c);
    }
    Serial.println("\n=== End of file ===");
    f.close();
    return true;
}

bool sd_delete_file(const char *filename) {
    if (!sd.exists(filename)) {
        Serial.print("File not found: ");
        Serial.println(filename);
        return false;
    }

    if (sd.remove(filename)) {
        Serial.print("Deleted file: ");
        Serial.println(filename);
        return true;
    } else {
        Serial.print("Failed to delete file: ");
        Serial.println(filename);
        return false;
    }
}

void sd_delete_all_files(void) {
    SdFile dir;
    dir.open("/");
    SdFile entry;

    Serial.println("Deleting all files...");

    while (entry.openNext(&dir, O_READ)) {
        if (!entry.isDir()) {
            char name[50];
            entry.getName(name, sizeof(name));
            entry.close();

            if (sd.remove(name)) {
                Serial.print("Deleted: ");
                Serial.println(name);
            } else {
                Serial.print("Failed to delete: ");
                Serial.println(name);
            }
        } else {
            entry.close();
        }
    }

    dir.close();
    Serial.println("All files deleted.\n");
}

// ===== Internal Helpers =====

static bool create_new_log_file() {
    for (int i = 1; i < 1000; i++) {
        snprintf(log_filename, sizeof(log_filename), "log_%03d.txt", i);
        if (!sd.exists(log_filename)) {
            if (log_file.open(log_filename, O_WRITE | O_CREAT)) {
                log_file.println("=== New Log Session ===");
                log_file.close();
                return true;
            }
        }
    }
    return false;
}

static bool find_last_log_file(char *filename, size_t size) {
    for (int i = 999; i >= 1; i--) {
        char test[20];
        snprintf(test, sizeof(test), "log_%03d.txt", i);
        if (sd.exists(test)) {
            strncpy(filename, test, size);
            return true;
        }
    }
    return false;
}

static void get_timestamp(char *buffer, size_t size) {
    unsigned long ms = millis();
    unsigned long s = ms / 1000;
    unsigned long m = s / 60;
    unsigned long h = m / 60;
    snprintf(buffer, size, "%02lu:%02lu:%02lu", h % 24, m % 60, s % 60);
}

static void write_line(const char *level, const char *message) {
    if (!log_file.open(log_filename, O_WRITE | O_APPEND)) {
        Serial.println("Error opening log file");
        return;
    }

    char timestamp[20];
    get_timestamp(timestamp, sizeof(timestamp));

    log_file.print("[");
    log_file.print(timestamp);
    log_file.print("]");
    if (strlen(level) > 0) {
        log_file.print(" [");
        log_file.print(level);
        log_file.print("]");
    }
    log_file.print(" ");
    log_file.println(message);

    log_file.close();
}

