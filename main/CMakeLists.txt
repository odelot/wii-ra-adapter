# Main component — the sketch (main.cpp) + its .cpp siblings.
# arduino / WiFiManager / rcheevos are components (managed + local).
#
# Console link: EXI (memory-card slot, exi_spi_slave.cpp) by default, or USB
# (ra_usb_link.cpp, same exi_spi_slave.h API) when CONFIG_RA_LINK_USB is set —
# see Kconfig.projbuild and sdkconfig.usb.defaults. CONFIG_ values exist only
# in the real configure pass; SRCS is ignored in the early requirements pass,
# so selecting the source here is safe. esp_tinyusb is a dependency of BOTH
# builds (idf_component.yml): the EXI build compiles it but never starts it.
if(CONFIG_RA_LINK_USB)
    set(ra_link_src "ra_usb_link.cpp")
else()
    set(ra_link_src "exi_spi_slave.cpp")
endif()

idf_component_register(
    SRCS
        "main.cpp"
        "${ra_link_src}"
        "loop_stack.cpp"
        "ra_web.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        arduino-esp32
        WiFiManager
        rcheevos
        esp_timer
        driver
        nvs_flash
        esp_http_server
        mdns
    # LAN dashboard (ra_web): single-file HTML embedded into flash, served at "/".
    EMBED_TXTFILES "web/index.html"
    # Unlock jingle played by the dashboard page, served at "/snd.mp3".
    EMBED_FILES "web/snd.mp3"
)

# main.cpp is the Arduino .ino (lenient Arduino warning level). IDF's -Werror=all
# is more specific than -Wno-error (so -Wno-error doesn't help) — silence warnings
# to get the migration building. Real API ERRORS (arduino-esp32 3.0->3.2 removals)
# still fail and surface; only warnings (format/sign/deprecations) are hushed.
# TODO post-migration: drop -w, clean main.cpp's warnings properly.
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
