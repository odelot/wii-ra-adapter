/**
 * board_config.h
 *
 * Single switch that selects the hardware target. Defines the EXI pin map and
 * the achievement-unlock LED behaviour for each board.
 *
 * To pick a board, set BOARD_VARIANT below (or pass -DBOARD_VARIANT=BOARD_XIAO
 * from the build) to one of:
 *
 *   BOARD_DEV  - ESP32-S3 development module (WROOM-1 N16R8 dev kit). EXI runs on
 *                the native SPI2 IO_MUX pins (GPIO10-14) = the fast path, no GPIO
 *                matrix delay. Onboard addressable WS2812 RGB LED on GPIO48.
 *
 *   BOARD_XIAO - Seeed Studio XIAO ESP32S3. The module only bonds out GPIO1-9 and
 *                GPIO43/44, so GPIO10-14 (the SPI2 IO_MUX pins) are NOT available
 *                -> EXI must route through the GPIO matrix on GPIO1-9 (slight
 *                timing penalty; if the EXI clock is too high lower it on the d2x
 *                side, no exposed pin gives the fast path). No onboard addressable
 *                LED -> the unlock celebration uses the single yellow user LED on
 *                GPIO21 (active-low).
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_DEV    0
#define BOARD_XIAO   1

/* >>> SELECT THE TARGET BOARD HERE <<< */
#ifndef BOARD_VARIANT
#define BOARD_VARIANT  BOARD_DEV
#endif

/* ---------------------------------------------------------------------------
 * BOARD_DEV — ESP32-S3 dev kit
 * ------------------------------------------------------------------------- */
#if BOARD_VARIANT == BOARD_DEV

  /* EXI on the SPI2 native IO_MUX pins (fast path, no GPIO matrix). */
  #define EXI_PIN_MOSI   11   // GameCube DI  -> ESP32 (GC writes to us)
  #define EXI_PIN_MISO   13   // ESP32 -> GameCube DO  (we write to GC)
  #define EXI_PIN_CLK    12   // GameCube CLK
  #define EXI_PIN_CS     10   // GameCube CS (active low)
  #define EXI_PIN_INT    14   // INT ESP32 -> GameCube  (must stay < 32: w1tc clear)

  /* Unlock LED: onboard WS2812 RGB on GPIO48. */
  #define LED_USE_RGB    1
  #define PIN_RGB        48

/* ---------------------------------------------------------------------------
 * BOARD_XIAO — Seeed Studio XIAO ESP32S3
 * ------------------------------------------------------------------------- */
#elif BOARD_VARIANT == BOARD_XIAO

  /* EXI via the GPIO matrix on the exposed pads. GPIO10-14 are not bonded out
   * on this module, so there is no IO_MUX fast path. Constraints honoured:
   *   - avoid GPIO3   (strapping pin: JTAG select; the Wii drives CS/CLK during
   *                    our boot and could disturb the boot mode)
   *   - avoid GPIO43/44 (UART0 = serial console / log output)
   *   - all pins < 32 (the INT line uses the GPIO_OUT_W1TC_REG fast-clear, which
   *                    only covers GPIO 0-31). */
  #define EXI_PIN_CS      7   // XIAO pad D8  — GameCube CS (active low)
  #define EXI_PIN_CLK     8   // XIAO pad D9  — GameCube CLK
  #define EXI_PIN_MOSI    9   // XIAO pad D10 — GameCube DI  -> ESP32
  #define EXI_PIN_MISO    5   // XIAO pad D4  — ESP32 -> GameCube DO
  #define EXI_PIN_INT     6   // XIAO pad D5  — INT ESP32 -> GameCube

  /* Unlock LED: single yellow user LED on GPIO21, ACTIVE-LOW (drive low = on). */
  #define LED_USE_GPIO        1
  #define PIN_LED_GPIO        21
  #define PIN_LED_ACTIVE_LOW  1

#else
  #error "board_config.h: BOARD_VARIANT must be BOARD_DEV or BOARD_XIAO"
#endif

#endif /* BOARD_CONFIG_H */
