#pragma once

#include <Arduino.h>

/**
 * @brief Function to init the filesystem
 * @param none
 * @return bool result
 */
bool filesystem_init(void);

/**
 * @brief Function to de-init the filesystem
 * @param none
 * @return none
 */
void filesystem_deinit(void);

/**
 * @brief Function to delete old versions of plugin images (by comparing the timestamp)
 *        It also deletes files that are older than 24h to keep SPIFFS from filling up
 * @param name filename
 * @return nothing
 */
void filesystem_purge_old_file(const char *name);

/**
 * @brief Function to read a file into a newly allocated buffer
 * @param name filename
 * @param out_buffer pointer to pointer of the output buffer
 * @return filesize in bytes or 0 if failed
 */
size_t filesystem_read_and_allocate(const char *name, uint8_t **out_buffer);

/**
 * @brief Function to read data from file
 * @param name filename
 * @param out_buffer pointer to output buffer
 * @return result - true if success; false - if failed
 */
bool filesystem_read_from_file(const char *name, uint8_t *out_buffer, size_t size);

/**
 * @brief Function to write data to file
 * @param name filename
 * @param in_buffer pointer to input buffer
 * @param size size of the input buffer
 * @return result - true if success; false - if failed
 */
size_t filesystem_write_to_file(const char *name, uint8_t *in_buffer, size_t size);

/**
 * @brief Function to check if file exists
 * @param name filename
 * @return result - true if exists; false - if not exists
 */
bool filesystem_file_exists(const char *name);

/**
 * @brief Function to delete the file
 * @param name filename
 * @return result - true if success; false - if failed
 */
bool filesystem_file_delete(const char *name);

/**
 * @brief Function to rename the file
 * @param old_name old filename
 * @param new_name new filename
 * @return result - true if success; false - if failed
 */
bool filesystem_file_rename(const char *old_name, const char *new_name);

void list_files();
