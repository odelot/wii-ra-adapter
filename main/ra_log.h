#pragma once
/* Shared logging facade so translation units other than main.cpp (e.g.
 * exi_spi_slave.cpp) can route through the async SRAM log ring instead of
 * calling Serial.* directly (which blocks the calling task when the UART TX
 * buffer fills). ralog_printf is DEFINED in main.cpp; declared here.
 *
 *   LOG_ERR  — always printed (errors / ultra-necessary, independent of level)
 *   LOG_INFO — printed when RA_LOG_LEVEL >= 1 (lifecycle milestones; also the
 *              level at which FRAME/CATCHUP telemetry is emitted — see main.cpp)
 *   LOG_DBG  — printed when RA_LOG_LEVEL >= 2 (diagnostics / dumps)
 *
 * For always-on, non-error lines (boot banner, ACHIEVEMENT) call ralog_printf
 * directly. Keep this default in sync with main.cpp's RA_LOG_LEVEL. */

void ralog_printf(const char *fmt, ...);

#ifndef RA_LOG_LEVEL
#define RA_LOG_LEVEL 1
#endif

#define LOG_ERR(...)  ralog_printf(__VA_ARGS__)
#define LOG_INFO(...) do { if (RA_LOG_LEVEL >= 1) ralog_printf(__VA_ARGS__); } while (0)
#define LOG_DBG(...)  do { if (RA_LOG_LEVEL >= 2) ralog_printf(__VA_ARGS__); } while (0)
