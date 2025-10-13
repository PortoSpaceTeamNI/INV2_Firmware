#ifndef LOGGING_H
#define LOGGING_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool sd_init(uint8_t cs_pin);

// --- Writing ---
void sd_log(const char *message);
void sd_log_info(const char *message);
void sd_log_error(const char *message);
void sd_log_raw(const char *message);
void sd_close(void);

// --- Reading ---
bool sd_open_for_read(const char *filename);
bool sd_read_line(char *buffer, size_t size);
void sd_close_read(void);

// --- Utilities ---
bool sd_get_last_filename(char *buffer, size_t size);
void sd_list_files(void);                         
bool sd_read_file(const char *filename);
bool sd_delete_file(const char *filename);   
void sd_delete_all_files(void);    

#endif // LOGGING_H
