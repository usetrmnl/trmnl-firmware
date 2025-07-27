#include <filesystem.h>
#include <Arduino.h>
#include <SPIFFS.h>
#include <trmnl_log.h>

/**
 * @brief Function to init the filesystem
 * @param none
 * @return bool result
 */
bool filesystem_init(void)
{
    if (!SPIFFS.begin(true))
    {
        Log_fatal("Failed to mount SPIFFS");
        ESP.restart();
        return false;
    }
    else
    {
        Log_info("SPIFFS mounted");
        return true;
    }
}

/**
 * @brief Function to de-init the filesystem
 * @param none
 * @return none
 */
void filesystem_deinit(void)
{
    SPIFFS.end();
}

/**
 * @brief Function to read data from file
 * @param name filename
 * @param out_buffer pointer to output buffer
 * @return result - true if success; false - if failed
 */
bool filesystem_read_from_file(const char *name, uint8_t *out_buffer, size_t size)
{
    if (SPIFFS.exists(name))
    {
        Log_info("file %s exists", name);
        File file = SPIFFS.open(name, FILE_READ);
        if (file)
        {
            file.readBytes((char *)out_buffer, size);
            return true;
        }
        else
        {
            Log_error("File %s open error", name);
            return false;
        }
    }
    else
    {
        Log_info("file %s doesn\'t exists", name);
        return false;
    }
}

/**
 * @brief Function to write data to file
 * @param name filename
 * @param in_buffer pointer to input buffer
 * @param size size of the input buffer
 * @return size of written bytes
 */
size_t filesystem_write_to_file(const char *name, uint8_t *in_buffer, size_t size)
{
    uint32_t SPIFFS_freeBytes = (SPIFFS.totalBytes() - SPIFFS.usedBytes());
    Log_info("SPIFFS free space - %d, total -%d", SPIFFS_freeBytes, SPIFFS.totalBytes());
    if (SPIFFS.exists(name))
    {
        Log_info("file %s exists. Deleting...", name);
        if (SPIFFS.remove(name))
            Log_info("file %s deleted", name);
        else
            Log_info("file %s deleting failed", name);
    }
    else
    {
        Log_info("file %s not exists.", name);
    }
    delay(100);
    File file = SPIFFS.open(name, FILE_WRITE);
    if (file)
    {
        // Write the buffer in chunks
        size_t bytesWritten = 0;
        while (bytesWritten < size)
        {

            size_t diff = size - bytesWritten;
            size_t chunkSize = _min(4096, diff);
            uint16_t res = file.write(in_buffer + bytesWritten, chunkSize);
            if (res != chunkSize)
            {
                file.close();

                Log_info("Erasing SPIFFS...");
                if (SPIFFS.format())
                {
                    Log_info("SPIFFS erased successfully.");
                }
                else
                {
                    Log_error("Error erasing SPIFFS.");
                }

                return bytesWritten;
            }
            bytesWritten += chunkSize;
        }
        Log_info("file %s writing success - %d bytes", name, bytesWritten);
        file.close();
        return bytesWritten;
    }
    else
    {
        Log_error("File open ERROR");
        return 0;
    }
}

/**
 * @brief Function to check if file exists
 * @param name filename
 * @return result - true if exists; false - if not exists
 */
bool filesystem_file_exists(const char *name)
{
    if (SPIFFS.exists(name))
    {
        Log_info("file %s exists.", name);
        return true;
    }
    else
    {
        Log_info("file %s not exists.", name);
        return false;
    }
}

/**
 * @brief Function to delete the file
 * @param name filename
 * @return result - true if success; false - if failed
 */
bool filesystem_file_delete(const char *name)
{
    if (SPIFFS.exists(name))
    {
        if (SPIFFS.remove(name))
        {
            Log_info("file %s deleted", name);
            return true;
        }
        else
        {
            Log_error("file %s deleting failed", name);
            return false;
        }
    }
    else
    {
        Log_info("file %s doesn't exist", name);
        return true;
    }
}

/**
 * @brief Function to rename the file
 * @param old_name old filename
 * @param new_name new filename
 * @return result - true if success; false - if failed
 */
bool filesystem_file_rename(const char *old_name, const char *new_name)
{
    if (SPIFFS.exists(old_name))
    {
        Log_info("file %s exists.", old_name);
        bool res = SPIFFS.rename(old_name, new_name);
        if (res)
        {
            Log_info("file %s renamed to %s.", old_name, new_name);
            return true;
        }
        else
            Log_error("file %s wasn't renamed.", old_name);
        return false;
    }
    else
    {
        Log_info("file %s not exists.", old_name);
        return false;
    }
}

void list_files()
{
    Log_info("Filesystem Usage: %d/%d", SPIFFS.usedBytes(), SPIFFS.totalBytes());
    File rootDir = SPIFFS.open("/");

    while (File file = rootDir.openNextFile())
    {
        Log_info("  %d  %s", file.size(), file.name());
    }
    rootDir.close();
}