//
// TrueType to bb_font converter
// Copyright (c) 2024 BitBank Software, inc.
// Written by Larry Bank
// August 31, 2024
// The bb_font format is a losslessly compressed bitmap font of a single point size
// The data is compressed with a compression scheme based on CCITT T.6
// The font structure includes overall size, per-character glyph info and then the
// compressed image data at the end.
// The font file format is designed to allow both dynamic loading of font data from
// external memory/disk or compiling the data as const into a progarm.
//
// Example usage:
// ./fontconvert <my_font.ttf> <out.bbf> <pt size> <start char> <end char>
// This code requires the freetype library
// found here: www.freetype.org
//
#include <ctype.h>
#include <ft2build.h>
#include <stdint.h>
#include <stdio.h>
#include FT_GLYPH_H
#include FT_MODULE_H
#include FT_TRUETYPE_DRIVER_H
#include "../src/g5enc.inl" // Group5 image compression library
G5ENCIMAGE g5enc; // Group5 encoder state

#define DPI 141 // Approximate resolution of common displays
#define OUTBUF_SIZE 65536
//
// Rotate a character 90/180/270 degrees
//
int RotateBitmap(int iAngle, uint8_t *pSrc, int iWidth, int iHeight, int iSrcPitch, uint8_t *pDst)
{
    int x, y, tx, ty, iDstPitch;
    uint8_t *s, *d, uc, ucMask;
    
    switch (iAngle) {
        case 90:
            iDstPitch = (iHeight+7)/8;
            for (y=0; y<iWidth; y++) {
                ucMask = 0x80 >> (y & 7);
                s = &pSrc[(y>>3) + (iHeight-1) * iSrcPitch]; // work from bottom up
                d = &pDst[y * iDstPitch];
                memset(d, 0, iDstPitch);
                for (x=0; x<iHeight; x++) {
                    if (s[0] & ucMask) {
                        d[x >> 3] |= (0x80 >> (x & 7));
                    }
                    s -= iSrcPitch;
                } // for x
            } // for y
            break;
        case 180:
            iDstPitch = iSrcPitch;
            for (y=0; y<iHeight; y++) {
                s = &pSrc[(iHeight-y-1) * iSrcPitch]; // work from bottom up
                d = &pDst[y * iDstPitch];
                memset(d, 0, iDstPitch);
                for (x=0; x<iWidth; x++) {
                    tx = (iWidth-x-1); // reverse x direction
                    // This code doesn't need to be efficient
                    uc = s[tx>>3] & (0x80 >> (tx & 7)); // source pixel
                    if (uc) {  // set
                        d[x >> 3] |= (0x80 >> (x & 7));
                    }
                } // for x
            } // for y
            break;
        case 270:
            iDstPitch = (iHeight+7)/8;
            for (y=0; y<iWidth; y++) {
                ty = (iWidth-1-y);
                ucMask = 0x80 >> (ty & 7);
                s = &pSrc[ty>>3]; // work from bottom up
                d = &pDst[y * iDstPitch];
                memset(d, 0, iDstPitch);
                for (x=0; x<iHeight; x++) {
                    if (s[0] & ucMask) {
                        d[x >> 3] |= (0x80 >> (x & 7));
                    }
                    s += iSrcPitch;
                } // for x
            } // for y
            break;
        default:
            return -1; // invalid
    }
    return iDstPitch;
} /* RotateBitmap() */
//
// Create the comments and const array boilerplate for the hex data bytes
//
void StartHexFile(FILE *f, int iLen, const char *fname, int size, int first, int last)
{
    int i, j;
    char szTemp[256];
    fprintf(f, "//\n// Created with fontconvert, written by Larry Bank\n");
    fprintf(f, "// Point size: %d, first: %d, last: %d\n", size, first, last);
    fprintf(f, "// compressed font data size = %d bytes\n//\n", iLen);
    fprintf(f, "// for non-Arduino builds...\n#ifndef PROGMEM\n#define PROGMEM\n#endif\n");

    strcpy(szTemp, fname);
    i = strlen(szTemp);
    if (szTemp[i-2] == '.') szTemp[i-2] = 0; // get the leaf name for the data
    j = i;
    // go backwards to get rid trim off just the leaf name
    while (j > 0 && szTemp[j] != '/') {
        j--;
    }
    if (szTemp[j] == '/') j++;
    fprintf(f, "const uint8_t %s[] PROGMEM = {\n", &szTemp[j]);
} /* StartHexFile() */
//
// Add N bytes of hex data to the output
// The data will be arranged in rows of 16 bytes each
//
void AddHexBytes(FILE *f, void *pData, int iLen, int bLast)
{
    static int iCount = 0; // number of bytes processed so far
    int i;
    uint8_t *s = (uint8_t *)pData;
    for (i=0; i<iLen; i++) { // process the given data
        fprintf(f, "0x%02x", *s++);
        iCount++;
        if (i < iLen-1 || !bLast) fprintf(f, ",");
        if ((iCount & 15) == 0) fprintf(f, "\n"); // next row of 16
    }
    if (bLast) {
        fprintf(f, "};\n");
    }
} /* AddHexBytes() */

const uint16_t uc1252Table[256] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 0-15 not used (some can be mapped to printable characters if needed)
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 16-31 not used
    32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,
    48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
    64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,
    80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
    96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,
    112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
    // Now starts the remapped Unicode characters
    0x20ac, 32, 0x201a, 0x192, 0x201e, 0x2026, 0x2020, 0x2021, 0x2c6, 0x2030,0x160,0x2039,0x152,32,0x17d,32, // 0x80-0x8f
    32, 0x2018, 0x2019, 0x201c, 0x201d, 0x2022, 0x2013, 0x2014, 0x2dc, 0x2122, 0x161, 0x2031, 0x153, 32, 0x17e, 0x178, // 0x90-0x9f
    0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,
    0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,
    0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
};

int main(int argc, char *argv[])
{
    int i, err, size, first = ' ', last = '~', y;
    int iLen ,iOffset = 0;
    int iRotation = 0;
    uint8_t *pBitmap, *pTemp;
    FILE *fOut;
    // TrueType library structures
    FT_Library library;
    FT_Face face;
    FT_Glyph glyph;
    FT_Bitmap *bitmap;
    FT_BitmapGlyphRec *g;
    // BitBank Font structures
    int bSmallFont = 0; // indicates if we're creating a normal or small font file
    BB_GLYPH *pGlyphs;
    BB_GLYPH_SMALL *pSmallGlyphs;
    BB_FONT bbff;
    BB_FONT_SMALL bbf2;

    int bHFile; // flag indicating if the output will be a .H file of hex data
    
    if (argc < 4) {
        printf("Usage: %s <in.ttf> <out.bbf or out.h> point_size [first_char] [last_char] [rotation (90/180/270)]\n", argv[0]);
        return 1;
    }
    size = atoi(argv[3]);
    bSmallFont = (size < 60); // Glyph info can fit in signed 8-bit values
    pTemp = (uint8_t *)argv[2] + strlen(argv[2]) - 1;
    bHFile = (pTemp[0] == 'H' || pTemp[0] == 'h'); // output an H file?

    if (argc == 5) {
        last = atoi(argv[4]); // only ending character was provided
    } else if (argc >= 6) {
        first = atoi(argv[4]); // start + end character codes were provided
        last = atoi(argv[5]);
    }
    if (argc == 7) { // rotation angle provided
        iRotation = atoi(argv[6]);
        if (iRotation != 90 && iRotation != 180 && iRotation != 270) {
            printf("Rotation angle can only be 90/180/270\n");
            return 1;
        }
    }
    if (last < first) {
        printf("Something went wrong - the starting character comes after the ending character. Try again...\n");
        return 1;
    }
    if (bSmallFont) {
        pSmallGlyphs = (BB_GLYPH_SMALL *)malloc((last - first + 1) * sizeof(BB_GLYPH_SMALL));
        if (!pSmallGlyphs) {
            printf("Error allocating memory for glyph data\n");
            return 1;
        }
    } else {
        pGlyphs = (BB_GLYPH *)malloc((last - first + 1) * sizeof(BB_GLYPH));
        if (!pGlyphs) {
            printf("Error allocating memory for glyph data\n");
            return 1;
        }
    }
    pBitmap = (uint8_t *)malloc(OUTBUF_SIZE); // Enough to hold the output
    pTemp = (uint8_t *)malloc(OUTBUF_SIZE); // for rotated bitmaps
    if (!pBitmap || ! pTemp) {
        printf("Error allocating memory for bitmap data\n");
        return 1;
    }
    
    // Init FreeType lib, load font
    if ((err = FT_Init_FreeType(&library))) {
        printf("FreeType init error: %d", err);
        return err;
    }
    // Print parameters
    printf("fontconvert %s to %s, size: %dpt, first: %d, last: %d, rotation: %d\n", argv[1], argv[2], size, first, last, iRotation);
    
    // Use TrueType engine version 35, without subpixel rendering.
    // This improves clarity of fonts since this library does not
    // support rendering multiple levels of gray in a glyph.
    // See https://github.com/adafruit/Adafruit-GFX-Library/issues/103
    FT_UInt interpreter_version = TT_INTERPRETER_VERSION_35;
    FT_Property_Set(library, "truetype", "interpreter-version",
                    &interpreter_version);
    
    if ((err = FT_New_Face(library, argv[1], 0, &face))) {
        printf("Font load error: %d\n", err);
        FT_Done_FreeType(library);
        return err;
    }
    
    // Shift the size left by 6 because the library uses '26dot6' fixed-point format
    FT_Set_Char_Size(face, size << 6, 0, DPI, 0);
    
    // Currently all symbols from 'first' to 'last' are processed.
    // Fonts may contain WAY more glyphs than that, but this code
    // will need to handle encoding stuff to deal with extracting
    // the right symbols, and that's not done yet.
    // Process glyphs and output huge bitmap data array
    for (i = first; i <= last; i++) {
        int iChar, index = i - first;
        uint8_t *s;
        int iPitch;
        
        iChar = uc1252Table[i]; // adjust for Codepade 1252 support
        // MONO renderer provides clean image with perfect crop
        // (no wasted pixels) via bitmap struct.
        if ((err = FT_Load_Char(face, iChar, FT_LOAD_TARGET_MONO))) {
            printf("Error %d loading char '%c'\n", err, iChar);
            continue;
        }
        
        if ((err = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_MONO))) {
            printf("Error %d rendering char '%c'\n", err, iChar);
            continue;
        }
        
        if ((err = FT_Get_Glyph(face->glyph, &glyph))) {
            printf("Error %d getting glyph '%c'\n", err, iChar);
            continue;
        }
        
        bitmap = &face->glyph->bitmap;
        g = (FT_BitmapGlyphRec *)glyph;
        
        if (bSmallFont) {
            pSmallGlyphs[index].bitmapOffset = iOffset;
            pSmallGlyphs[index].width = bitmap->width;
            pSmallGlyphs[index].height = bitmap->rows;
            pSmallGlyphs[index].xAdvance = face->glyph->advance.x >> 6;
            pSmallGlyphs[index].xOffset = g->left;
            pSmallGlyphs[index].yOffset = 1 - g->top;
        } else {
            pGlyphs[index].bitmapOffset = iOffset;
            pGlyphs[index].width = bitmap->width;
            pGlyphs[index].height = bitmap->rows;
            pGlyphs[index].xAdvance = face->glyph->advance.x >> 6;
            pGlyphs[index].xOffset = g->left;
            pGlyphs[index].yOffset = 1 - g->top;
        }
        s = bitmap->buffer;
        iPitch = bitmap->pitch;
        if (iRotation != 0) {
            iPitch = RotateBitmap(iRotation, bitmap->buffer, bitmap->width, bitmap->rows, bitmap->pitch, pTemp);
            s = pTemp;
            g5_encode_init(&g5enc, bitmap->rows, bitmap->width, &pBitmap[iOffset], OUTBUF_SIZE-iOffset);
            for (y = 0; y < bitmap->width; y++) {
                g5_encode_encodeLine(&g5enc, &s[y * iPitch]);
            } // for y
        } else { // not rotated
            g5_encode_init(&g5enc, bitmap->width, bitmap->rows, &pBitmap[iOffset], OUTBUF_SIZE-iOffset);
            for (y = 0; y < bitmap->rows; y++) {
                g5_encode_encodeLine(&g5enc, &s[y * iPitch]);
            } // for y
        }
        iLen = g5_encode_getOutSize(&g5enc);
        iOffset += iLen;
        
        FT_Done_Glyph(glyph);
    } // for each glyph
    
    // Try to create the output file
    fOut = fopen(argv[2], "w+b");
    if (!fOut) {
        printf("Error creating output file: %s\n", argv[2]);
        return 1;
    }
    // Write the file header
    if (bSmallFont) {
        bbf2.u16Marker = BB_FONT_MARKER_SMALL;
        bbf2.first = first;
        bbf2.last = last;
        bbf2.rotation = iRotation; // save rotation angle
        if (face->size->metrics.height == 0) {
            // No face height info, assume fixed width and get from a glyph.
            bbf2.height = pGlyphs[0].height;
        } else {
            bbf2.height = (face->size->metrics.height >> 6);
        }
        iLen = sizeof(bbf2) + (last-first+1)*sizeof(BB_GLYPH_SMALL) + iOffset;
        if (bHFile) { // create an H file of hex values
            StartHexFile(fOut, iLen, argv[2], size, first, last);
            AddHexBytes(fOut, &bbf2, sizeof(bbf2), 0);
            AddHexBytes(fOut, pSmallGlyphs, (last-first+1) * sizeof(BB_GLYPH_SMALL), 0);
            AddHexBytes(fOut, pBitmap, iOffset, 1);
        } else {
            fwrite(&bbf2, 1, sizeof(bbf2), fOut);
            // Write the glyph table
            fwrite(pSmallGlyphs, (last-first+1), sizeof(BB_GLYPH_SMALL), fOut);
            // Write the compressed bitmap data
            fwrite(pBitmap, 1, iOffset, fOut);
        }
    } else {
        bbff.u16Marker = BB_FONT_MARKER;
        bbff.first = first;
        bbff.last = last;
        bbff.rotation = iRotation; // save rotation angle
        if (face->size->metrics.height == 0) {
            // No face height info, assume fixed width and get from a glyph.
            bbff.height = pGlyphs[0].height;
        } else {
            bbff.height = (face->size->metrics.height >> 6);
        }
        iLen = sizeof(bbff) + (last-first+1)*sizeof(BB_GLYPH) + iOffset;
        if (bHFile) { // create an H file of hex values
            StartHexFile(fOut, iLen, argv[2], size, first, last);
            AddHexBytes(fOut, &bbff, sizeof(bbff), 0);
            AddHexBytes(fOut, pGlyphs, (last-first+1) * sizeof(BB_GLYPH), 0);
            AddHexBytes(fOut, pBitmap, iOffset, 1);
        } else {
            fwrite(&bbff, 1, sizeof(bbff), fOut);
            // Write the glyph table
            fwrite(pGlyphs, (last-first+1), sizeof(BB_GLYPH), fOut);
            // Write the compressed bitmap data
            fwrite(pBitmap, 1, iOffset, fOut);
        }
    } // large fonts
    fflush(fOut);
    fclose(fOut); // done!
    FT_Done_FreeType(library);
    printf("Success!\nFont file size: %d bytes\n", iLen);
    
    return 0;
} /* main() */
