/* libretro.h stub for Arduino IDE compilation of rcheevos.
 * Contains only the minimal definitions required by rc_libretro.h / rc_libretro.c.
 * The gc-ra-adapter does not use libretro frontend functionality; this stub
 * exists solely to satisfy the #include so the rest of rcheevos can compile.
 */

#ifndef LIBRETRO_H
#define LIBRETRO_H

#include <stdint.h>
#include <stddef.h>

/* Memory type constants used by rc_libretro.c */
#define RETRO_MEMORY_MASK        0xff
#define RETRO_MEMORY_SAVE_RAM    0
#define RETRO_MEMORY_RTC         1
#define RETRO_MEMORY_SYSTEM_RAM  2
#define RETRO_MEMORY_VIDEO_RAM   3

/* Memory descriptor flags */
#define RETRO_MEMDESC_CONST      (1 << 0)
#define RETRO_MEMDESC_BIGENDIAN  (1 << 1)
#define RETRO_MEMDESC_SYSTEM_RAM (1 << 2)
#define RETRO_MEMDESC_SAVE_RAM   (1 << 3)
#define RETRO_MEMDESC_VIDEO_RAM  (1 << 4)
#define RETRO_MEMDESC_ALIGN_2    (1 << 16)
#define RETRO_MEMDESC_MINSIZE_2  (1 << 24)

/* Memory descriptor — mirrors the real libretro struct layout */
struct retro_memory_descriptor
{
   uint64_t    flags;
   void       *ptr;
   size_t      offset;
   size_t      start;
   size_t      select;
   size_t      disconnect;
   size_t      len;
   const char *addrspace;
};

/* Memory map passed to the frontend */
struct retro_memory_map
{
   const struct retro_memory_descriptor *descriptors;
   unsigned                              num_descriptors;
};

#endif /* LIBRETRO_H */
