# ---------------------------------------------------------------------------
# index.html -> src/index_html_gen.cpp (array byte), dibuat saat konfigurasi.
# PlatformIO tidak bisa menangani EMBED_TXTFILES milik IDF (file .S hasil
# generate tidak ditemukan), jadi HTML disisipkan sebagai array C++ biasa.
# Diedit index.html? Konfigurasi ulang otomatis (CMAKE_CONFIGURE_DEPENDS).
# ---------------------------------------------------------------------------
if(NOT CMAKE_SCRIPT_MODE_FILE)
  set(HTML_IN  "${CMAKE_SOURCE_DIR}/src/index.html")
  set(HTML_OUT "${CMAKE_SOURCE_DIR}/src/index_html_gen.cpp")
  set(HTML_TMP "${CMAKE_BINARY_DIR}/index_html_gen.tmp")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HTML_IN}")

  file(READ "${HTML_IN}" HTML_HEX HEX)
  string(LENGTH "${HTML_HEX}" HTML_HEXLEN)
  math(EXPR HTML_LEN "${HTML_HEXLEN} / 2")
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," HTML_ARR "${HTML_HEX}")
  file(WRITE "${HTML_TMP}"
"/* GENERATED dari index.html - jangan diedit */
#include <stdint.h>
extern \"C\" {
extern const uint8_t index_html_data[];
extern const uint32_t index_html_len;
const uint8_t index_html_data[] = {${HTML_ARR}0x00};
const uint32_t index_html_len = ${HTML_LEN};
}
")
  # copy_if_different: file tidak dikompilasi ulang kalau HTML tidak berubah
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
                  "${HTML_TMP}" "${HTML_OUT}")
endif()

# Hanya .c/.cpp. JANGAN tulis "REQUIRES arduino": di PlatformIO hybrid komponen
# Arduino didaftarkan PlatformIO sendiri dan sudah otomatis terlihat.
file(GLOB_RECURSE app_sources
     ${CMAKE_SOURCE_DIR}/src/*.c
     ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS ".")

# IDF menjadikan peringatan format-truncation sebagai ERROR; build Arduino murni
# tidak. Kembalikan jadi peringatan biasa.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=format-truncation
    -Wno-error=format-overflow
    -Wno-error=stringop-truncation
    -Wno-error=unused-variable
    -Wno-error=unused-but-set-variable
    -Wno-error=unused-function)
