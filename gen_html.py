import os

html_path = os.path.join('src', 'index.html')
out_path = os.path.join('src', 'index_html_gen.cpp')

with open(html_path, 'rb') as f:
    data = f.read()

hex_arr = ','.join(f'0x{b:02x}' for b in data)
content = (
    '/* GENERATED dari index.html - jangan diedit */\n'
    '#include <stdint.h>\n'
    'extern "C" {\n'
    'extern const uint8_t index_html_data[];\n'
    'extern const uint32_t index_html_len;\n'
    f'const uint8_t index_html_data[] = {{{hex_arr},0x00}};\n'
    f'const uint32_t index_html_len = {len(data)};\n'
    '}\n'
)

with open(out_path, 'w', encoding='utf-8') as f:
    f.write(content)

print(f"Successfully generated {out_path} ({len(data)} bytes HTML).")
