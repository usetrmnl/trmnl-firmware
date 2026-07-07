#! /usr/bin/env python3

import os
import io
import gzip

allowed_suffix = [".html", ".svg"]
path = os.path.dirname(os.path.realpath(__file__))
output_dir = os.path.join(path, '../src')
files = os.listdir(path)

def gzip_file(file_path):
    compressed_data = io.BytesIO()

    # Open the input file
    with open(file_path, 'rb') as f:
        # Create a GzipFile object with the in-memory buffer
        with gzip.GzipFile(fileobj=compressed_data, mode='wb') as gz:
            # Read the file content and write it to the GzipFile
            gz.write(f.read())

    # Get the compressed data from the buffer
    compressed_data.seek(0)
    return compressed_data.read()

web_files_content = ""

for file in sorted(os.listdir(path)):
    if file.endswith(tuple(allowed_suffix)):
        input_file = os.path.join(path, file)
        content = gzip_file(input_file)
        byte_array = bytearray(content)
        hex_array = [hex(b) for b in byte_array]
        arrayName = os.path.basename(input_file).replace('.','_').upper()
        web_files_content += "const uint8_t "+arrayName+"[] PROGMEM = { " + ', '.join(hex_array) + " };\n"
        web_files_content += f"const int {arrayName}_LEN = sizeof({arrayName});\n\n"

if len(web_files_content) != 0:
    with open(os.path.join(output_dir, 'WifiCaptivePage.h'), 'w') as f_output:
        f_output.write("#ifndef WifiCaptivePage_h\n")
        f_output.write("#define WifiCaptivePage_h\n\n")
        f_output.write("#include <pgmspace.h>\n\n")
        f_output.write(web_files_content)
        f_output.write("#endif")


print('Done.')