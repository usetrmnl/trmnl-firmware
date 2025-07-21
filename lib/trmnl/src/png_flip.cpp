#include <stdlib.h>
#include <cstdint>
#include <string.h>

unsigned char reverse_bits(unsigned char b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void flip_image(unsigned char *buffer, int width, int height, bool bFlipH) {
    int row_bytes = width / 8;
    unsigned char *temp_row = (uint8_t *)malloc(row_bytes);

    for (int y = 0; y < height / 2; y++) {
        unsigned char *top_row = buffer + y * row_bytes;
        unsigned char *bottom_row = buffer + (height - y - 1) * row_bytes;

        // Swap top and bottom rows
        memcpy(temp_row, top_row, row_bytes);
        memcpy(top_row, bottom_row, row_bytes);
        memcpy(bottom_row, temp_row, row_bytes);
    }

    free(temp_row);

    if (bFlipH) {
        // Mirror horizontally (bytes and bits)
        for (int y = 0; y < height; y++) {
            unsigned char *row = buffer + y * row_bytes;

            // Reverse bytes within the row
            for (int x = 0; x < row_bytes / 2; x++) {
                unsigned char tmp = row[x];
                row[x] = row[row_bytes - 1 - x];
                row[row_bytes - 1 - x] = tmp;
            }

            // Reverse bits in each byte
            for (int x = 0; x < row_bytes; x++) {
                row[x] = reverse_bits(row[x]);
            }
        }
    }
}

void horizontal_mirror(unsigned char *buffer, int width, int height) {
    int row_bytes = width / 8;

    for (int y = 0; y < height; y++) {
        unsigned char *row = buffer + y * row_bytes;

        // Reverse bytes in row
        for (int x = 0; x < row_bytes / 2; x++) {
            unsigned char tmp = row[x];
            row[x] = row[row_bytes - 1 - x];
            row[row_bytes - 1 - x] = tmp;
        }

        // Reverse bits within each byte
        for (int x = 0; x < row_bytes; x++) {
            row[x] = reverse_bits(row[x]);
        }
    }
}
