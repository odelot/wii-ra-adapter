cmake_minimum_required(VERSION 3.5)

idf_component_register(
                       SRCS "WiFiManager.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES arduino-esp32
)

# WiFiManager 2.0.17 (upstream) uses %d for uint32_t (line 3369). IDF's
# -Werror=all is more specific than -Wno-error (GCC: specific wins regardless of
# order), so -Wno-error doesn't help — silence warnings entirely for this lib.
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
