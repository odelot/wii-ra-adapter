#include "rc_internal.h"

#include <stdlib.h> /* malloc/realloc */
#include <string.h> /* memcpy */
#include <math.h>   /* INFINITY/NAN */

#define MEMREF_PLACEHOLDER_ADDRESS 0xFFFFFFFF

#ifdef RC_SHADOW_VALUES
/* ===========================================================================
 * Compact shadow value cache (RC_SHADOW_VALUES) — see project_shadow_array.
 * Eval reads memref values out of 56-byte structs (strided -> busts cache, 13.7x
 * slower per MEMBENCH). Keep compact arrays of {value,prior,changed} indexed by a
 * slot carried in each rc_operand_t (cache-hot). UPD (single-threaded) mirrors
 * struct->shadow; eval reads shadow (read-only -> parallel-safe). Shadow==struct by
 * construction -> bit-identical (host-validated 2313/2314). Device: arena is PSRAM,
 * set once at boot via rc_shadow_set_arena; slots built at game load.
 * =========================================================================== */
volatile int g_rc_shadow_enabled = 1;   /* runtime A/B */
uint32_t* g_rc_sv_value = NULL;
uint32_t* g_rc_sv_prior = NULL;
uint8_t*  g_rc_sv_changed = NULL;
uint32_t  g_rc_shadow_count = 0;
uint32_t  g_rc_shadow_cap = 0;

void rc_shadow_set_arena(uint32_t* value, uint32_t* prior, uint8_t* changed, uint32_t cap) {
  g_rc_sv_value = value;
  g_rc_sv_prior = prior;
  g_rc_sv_changed = changed;
  g_rc_shadow_cap = cap;
  g_rc_shadow_count = 0;   /* reset on each game load (re-call with same arena) */
}

static uint16_t rc_shadow_assign(rc_memref_t* memref) {
  if (memref->shadow_slot != 0xFFFF)
    return memref->shadow_slot;
  if (!g_rc_sv_value || g_rc_shadow_count >= g_rc_shadow_cap || g_rc_shadow_count >= 0xFFFF)
    return 0xFFFF;
  memref->shadow_slot = (uint16_t)g_rc_shadow_count++;
  return memref->shadow_slot;
}

static void rc_shadow_build_operand(rc_operand_t* op) {
  op->shadow_slot = rc_operand_is_memref(op) ? rc_shadow_assign(op->value.memref) : 0xFFFF;
}

static void rc_shadow_build_condset(rc_condset_t* cs) {
  rc_condition_t* c;
  if (!cs)
    return;
  for (c = cs->conditions; c != NULL; c = c->next) {
    rc_shadow_build_operand(&c->operand1);
    rc_shadow_build_operand(&c->operand2);
  }
}

void rc_shadow_build_trigger(rc_trigger_t* trigger) {
  rc_condset_t* cs;
  if (!trigger)
    return;
  rc_shadow_build_condset(trigger->requirement);
  for (cs = trigger->alternative; cs != NULL; cs = cs->next)
    rc_shadow_build_condset(cs);
}

/* UPD side: mirror one memref's value into the compact shadow. Lazy-assigns the slot
 * (works whether UPD runs before or after the build). Skipped (incr-upd) memrefs keep
 * their last shadow = still correct (skip implies the value is unchanged). */
void rc_shadow_mirror(rc_memref_t* m) {
  uint16_t s = m->shadow_slot;
  if (s == 0xFFFF) {
    s = rc_shadow_assign(m);
    if (s == 0xFFFF)
      return;
  }
  g_rc_sv_value[s]   = m->value.value;
  g_rc_sv_prior[s]   = m->value.prior;
  g_rc_sv_changed[s] = m->value.changed;
}
#endif /* RC_SHADOW_VALUES */

rc_memref_t* rc_alloc_memref(rc_parse_state_t* parse, uint32_t address, uint8_t size) {
  rc_memref_list_t* memref_list = NULL;
  rc_memref_t* memref = NULL;
  int i;

  for (i = 0; i < 2; i++) {
    if (i == 0) {
      if (!parse->existing_memrefs)
        continue;

      memref_list = &parse->existing_memrefs->memrefs;
    }
    else {
      memref_list = &parse->memrefs->memrefs;
    }

    do
    {
      const rc_memref_t* memref_stop;

      memref = memref_list->items;
      memref_stop = memref + memref_list->count;

      for (; memref < memref_stop; ++memref) {
        if (memref->address == address && memref->value.size == size)
          return memref;
      }

      if (!memref_list->next)
        break;

      memref_list = memref_list->next;
    } while (1);
  }

  /* no match found, find a place to put the new entry */
  memref_list = &parse->memrefs->memrefs;
  while (memref_list->count == memref_list->capacity && memref_list->next)
    memref_list = memref_list->next;

  /* create a new entry */
  if (memref_list->count < memref_list->capacity) {
    memref = &memref_list->items[memref_list->count++];
  } else {
    const int32_t old_offset = parse->offset;

    if (memref_list->capacity != 0) {
      memref_list = memref_list->next = RC_ALLOC_SCRATCH(rc_memref_list_t, parse);
      memref_list->next = NULL;
    }

    memref_list->items = RC_ALLOC_ARRAY_SCRATCH(rc_memref_t, 8, parse);
    memref_list->count = 1;
    memref_list->capacity = 8;
    memref_list->allocated = 0;

    memref = memref_list->items;

    /* in preparse mode, don't count this memory, we'll do a single allocation once we have
     * the final total */
    if (!parse->buffer)
      parse->offset = old_offset;
  }

  memset(memref, 0, sizeof(*memref));
  memref->value.memref_type = RC_MEMREF_TYPE_MEMREF;
  memref->value.type = RC_VALUE_TYPE_UNSIGNED;
  memref->value.size = size;
  memref->address = address;
#ifdef RC_SHADOW_VALUES
  memref->shadow_slot = 0xFFFF;
#endif

  return memref;
}

rc_modified_memref_t* rc_alloc_modified_memref(rc_parse_state_t* parse, uint8_t size, const rc_operand_t* parent,
                                               uint8_t modifier_type, const rc_operand_t* modifier) {
  rc_modified_memref_list_t* modified_memref_list = NULL;
  rc_modified_memref_t* modified_memref = NULL;
  int i = 0;

  for (i = 0; i < 2; i++) {
    if (i == 0) {
      if (!parse->existing_memrefs)
        continue;

      modified_memref_list = &parse->existing_memrefs->modified_memrefs;
    }
    else {
      modified_memref_list = &parse->memrefs->modified_memrefs;
    }

    do {
      const rc_modified_memref_t* memref_stop;

      modified_memref = modified_memref_list->items;
      memref_stop = modified_memref + modified_memref_list->count;

      for (; modified_memref < memref_stop; ++modified_memref) {
        if (modified_memref->memref.value.size == size &&
            modified_memref->modifier_type == modifier_type &&
            rc_operands_are_equal(&modified_memref->parent, parent) &&
            rc_operands_are_equal(&modified_memref->modifier, modifier)) {
          return modified_memref;
        }
      }

      if (!modified_memref_list->next)
        break;

      modified_memref_list = modified_memref_list->next;
    } while (1);
  }

  /* no match found, find a place to put the new entry */
  modified_memref_list = &parse->memrefs->modified_memrefs;
  while (modified_memref_list->count == modified_memref_list->capacity && modified_memref_list->next)
    modified_memref_list = modified_memref_list->next;

  /* create a new entry */
  if (modified_memref_list->count < modified_memref_list->capacity) {
    modified_memref = &modified_memref_list->items[modified_memref_list->count++];
  } else {
    const int32_t old_offset = parse->offset;

    if (modified_memref_list->capacity != 0) {
      modified_memref_list = modified_memref_list->next = RC_ALLOC_SCRATCH(rc_modified_memref_list_t, parse);
      modified_memref_list->next = NULL;
    }

    modified_memref_list->items = RC_ALLOC_ARRAY_SCRATCH(rc_modified_memref_t, 8, parse);
    modified_memref_list->count = 1;
    modified_memref_list->capacity = 8;
    modified_memref_list->allocated = 0;

    modified_memref = modified_memref_list->items;

    /* in preparse mode, don't count this memory, we'll do a single allocation once we have
     * the final total */
    if (!parse->buffer)
      parse->offset = old_offset;
  }

  memset(modified_memref, 0, sizeof(*modified_memref));
  modified_memref->memref.value.memref_type = RC_MEMREF_TYPE_MODIFIED_MEMREF;
  modified_memref->memref.value.size = size;
  modified_memref->memref.value.type = rc_memsize_is_float(size) ? RC_VALUE_TYPE_FLOAT : RC_VALUE_TYPE_UNSIGNED;
  memcpy(&modified_memref->parent, parent, sizeof(modified_memref->parent));
  memcpy(&modified_memref->modifier, modifier, sizeof(modified_memref->modifier));
  modified_memref->modifier_type = modifier_type;
  modified_memref->depth = 0;
  modified_memref->memref.address = rc_operand_is_memref(modifier) ? modifier->value.memref->address : modifier->value.num;
#ifdef RC_SHADOW_VALUES
  modified_memref->memref.shadow_slot = 0xFFFF;
#endif

  if (rc_operand_is_memref(parent) && parent->value.memref->value.memref_type == RC_MEMREF_TYPE_MODIFIED_MEMREF) {
    const rc_modified_memref_t* parent_modified_memref = (rc_modified_memref_t*)parent->value.memref;
    modified_memref->depth = parent_modified_memref->depth + 1;
  }

  return modified_memref;
}

void rc_memrefs_init(rc_memrefs_t* memrefs)
{
  memset(memrefs, 0, sizeof(*memrefs));

  memrefs->memrefs.capacity = 32;
  memrefs->memrefs.items =
    (rc_memref_t*)malloc(memrefs->memrefs.capacity * sizeof(rc_memref_t));
  memrefs->memrefs.allocated = 1;

  memrefs->modified_memrefs.capacity = 16;
  memrefs->modified_memrefs.items =
    (rc_modified_memref_t*)malloc(memrefs->modified_memrefs.capacity * sizeof(rc_modified_memref_t));
  memrefs->modified_memrefs.allocated = 1;
}

void rc_memrefs_destroy(rc_memrefs_t* memrefs)
{
  rc_memref_list_t* memref_list = &memrefs->memrefs;
  rc_modified_memref_list_t* modified_memref_list = &memrefs->modified_memrefs;

  do {
    rc_memref_list_t* current_memref_list = memref_list;
    memref_list = memref_list->next;

    if (current_memref_list->allocated) {
      if (current_memref_list->items)
        free(current_memref_list->items);

      if (current_memref_list != &memrefs->memrefs)
        free(current_memref_list);
    }
  } while (memref_list);

  do {
    rc_modified_memref_list_t* current_modified_memref_list = modified_memref_list;
    modified_memref_list = modified_memref_list->next;

    if (current_modified_memref_list->allocated) {
      if (current_modified_memref_list->items)
        free(current_modified_memref_list->items);

      if (current_modified_memref_list != &memrefs->modified_memrefs)
        free(current_modified_memref_list);
    }
  } while (modified_memref_list);

  free(memrefs);
}

uint32_t rc_memrefs_count_memrefs(const rc_memrefs_t* memrefs)
{
  uint32_t count = 0;
  const rc_memref_list_t* memref_list = &memrefs->memrefs;
  while (memref_list) {
    count += memref_list->count;
    memref_list = memref_list->next;
  }

  return count;
}

uint32_t rc_memrefs_count_modified_memrefs(const rc_memrefs_t* memrefs)
{
  uint32_t count = 0;
  const rc_modified_memref_list_t* modified_memref_list = &memrefs->modified_memrefs;
  while (modified_memref_list) {
    count += modified_memref_list->count;
    modified_memref_list = modified_memref_list->next;
  }

  return count;
}

int rc_parse_memref(const char** memaddr, uint8_t* size, uint32_t* address) {
  const char* aux = *memaddr;
  char* end;
  unsigned long value;

  if (aux[0] == '0') {
    if (aux[1] != 'x' && aux[1] != 'X')
      return RC_INVALID_MEMORY_OPERAND;

    aux += 2;
    switch (*aux++) {
      /* ordered by estimated frequency in case compiler doesn't build a jump table */
      case 'h': case 'H': *size = RC_MEMSIZE_8_BITS; break;
      case ' ':           *size = RC_MEMSIZE_16_BITS; break;
      case 'x': case 'X': *size = RC_MEMSIZE_32_BITS; break;

      case 'm': case 'M': *size = RC_MEMSIZE_BIT_0; break;
      case 'n': case 'N': *size = RC_MEMSIZE_BIT_1; break;
      case 'o': case 'O': *size = RC_MEMSIZE_BIT_2; break;
      case 'p': case 'P': *size = RC_MEMSIZE_BIT_3; break;
      case 'q': case 'Q': *size = RC_MEMSIZE_BIT_4; break;
      case 'r': case 'R': *size = RC_MEMSIZE_BIT_5; break;
      case 's': case 'S': *size = RC_MEMSIZE_BIT_6; break;
      case 't': case 'T': *size = RC_MEMSIZE_BIT_7; break;
      case 'l': case 'L': *size = RC_MEMSIZE_LOW; break;
      case 'u': case 'U': *size = RC_MEMSIZE_HIGH; break;
      case 'k': case 'K': *size = RC_MEMSIZE_BITCOUNT; break;
      case 'w': case 'W': *size = RC_MEMSIZE_24_BITS; break;
      case 'g': case 'G': *size = RC_MEMSIZE_32_BITS_BE; break;
      case 'i': case 'I': *size = RC_MEMSIZE_16_BITS_BE; break;
      case 'j': case 'J': *size = RC_MEMSIZE_24_BITS_BE; break;

      /* case 'v': case 'V': */
      /* case 'y': case 'Y': 64 bit? */
      /* case 'z': case 'Z': 128 bit? */

      case '0':
        if (*aux == 'x') /* user mistyped an extra 0x: 0x0xabcd */
          return RC_INVALID_MEMORY_OPERAND;
        /* fallthrough */

      case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
      case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
      case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
        /* legacy support - addresses without a size prefix are assumed to be 16-bit */
        aux--;
        *size = RC_MEMSIZE_16_BITS;
        break;

      default:
        return RC_INVALID_MEMORY_OPERAND;
    }
  }
  else if (aux[0] == 'f' || aux[0] == 'F') {
    ++aux;
    switch (*aux++) {
      case 'f': case 'F': *size = RC_MEMSIZE_FLOAT; break;
      case 'b': case 'B': *size = RC_MEMSIZE_FLOAT_BE; break;
      case 'h': case 'H': *size = RC_MEMSIZE_DOUBLE32; break;
      case 'i': case 'I': *size = RC_MEMSIZE_DOUBLE32_BE; break;
      case 'm': case 'M': *size = RC_MEMSIZE_MBF32; break;
      case 'l': case 'L': *size = RC_MEMSIZE_MBF32_LE; break;

      default:
        return RC_INVALID_FP_OPERAND;
    }
  }
  else {
    return RC_INVALID_MEMORY_OPERAND;
  }

  value = strtoul(aux, &end, 16);

  if (end == aux)
    return RC_INVALID_MEMORY_OPERAND;

  if (value > 0xffffffffU)
    value = 0xffffffffU;

  *address = (uint32_t)value;
  *memaddr = end;
  return RC_OK;
}

static float rc_build_float(uint32_t mantissa_bits, int32_t exponent, int sign) {
  /* 32-bit float has a 23-bit mantissa and 8-bit exponent */
  const uint32_t implied_bit = 1 << 23;
  const uint32_t mantissa = mantissa_bits | implied_bit;
  double dbl = ((double)mantissa) / ((double)implied_bit);

  if (exponent > 127) {
    /* exponent above 127 is a special number */
    if (mantissa_bits == 0) {
      /* infinity */
#ifdef INFINITY /* INFINITY and NAN #defines require C99 */
      dbl = (double)INFINITY;
#else
      dbl = -log(0.0);
#endif
    }
    else {
      /* NaN */
#ifdef NAN
      dbl = NAN;
#else
      dbl = -sqrt(-1);
#endif
    }
  }
  else if (exponent > 0) {
    /* exponent from 1 to 127 is a number greater than 1 */
    while (exponent > 30) {
      dbl *= (double)(1 << 30);
      exponent -= 30;
    }
    dbl *= (double)((long long)1 << exponent);
  }
  else if (exponent < 0) {
    /* exponent from -1 to -127 is a number less than 1 */

    if (exponent == -127) {
      /* exponent -127 (all exponent bits were zero) is a denormalized value
       * (no implied leading bit) with exponent -126 */
      dbl = ((double)mantissa_bits) / ((double)implied_bit);
      exponent = 126;
    } else {
      exponent = -exponent;
    }

    while (exponent > 30) {
      dbl /= (double)(1 << 30);
      exponent -= 30;
    }
    dbl /= (double)((long long)1 << exponent);
  }
  else {
    /* exponent of 0 requires no adjustment */
  }

  return (sign) ? (float)-dbl : (float)dbl;
}

static void rc_transform_memref_float(rc_typed_value_t* value) {
  /* decodes an IEEE 754 float */
  const uint32_t mantissa = (value->value.u32 & 0x7FFFFF);
  const int32_t exponent = (int32_t)((value->value.u32 >> 23) & 0xFF) - 127;
  const int sign = (value->value.u32 & 0x80000000);
  value->value.f32 = rc_build_float(mantissa, exponent, sign);
  value->type = RC_VALUE_TYPE_FLOAT;
}

static void rc_transform_memref_float_be(rc_typed_value_t* value) {
  /* decodes an IEEE 754 float in big endian format */
  const uint32_t mantissa = ((value->value.u32 & 0xFF000000) >> 24) |
                            ((value->value.u32 & 0x00FF0000) >> 8) |
                            ((value->value.u32 & 0x00007F00) << 8);
  const int32_t exponent = (int32_t)(((value->value.u32 & 0x0000007F) << 1) |
                                     ((value->value.u32 & 0x00008000) >> 15)) - 127;
  const int sign = (value->value.u32 & 0x00000080);
  value->value.f32 = rc_build_float(mantissa, exponent, sign);
  value->type = RC_VALUE_TYPE_FLOAT;
}

static void rc_transform_memref_double32(rc_typed_value_t* value)
{
  /* decodes the four most significant bytes of an IEEE 754 double into a float */
  const uint32_t mantissa = (value->value.u32 & 0x000FFFFF) << 3;
  const int32_t exponent = (int32_t)((value->value.u32 >> 20) & 0x7FF) - 1023;
  const int sign = (value->value.u32 & 0x80000000);
  value->value.f32 = rc_build_float(mantissa, exponent, sign);
  value->type = RC_VALUE_TYPE_FLOAT;
}

static void rc_transform_memref_double32_be(rc_typed_value_t* value)
{
  /* decodes the four most significant bytes of an IEEE 754 double in big endian format into a float */
  const uint32_t mantissa = (((value->value.u32 & 0xFF000000) >> 24) |
    ((value->value.u32 & 0x00FF0000) >> 8) |
    ((value->value.u32 & 0x00000F00) << 8)) << 3;
  const int32_t exponent = (int32_t)(((value->value.u32 & 0x0000007F) << 4) |
    ((value->value.u32 & 0x0000F000) >> 12)) - 1023;
  const int sign = (value->value.u32 & 0x00000080);
  value->value.f32 = rc_build_float(mantissa, exponent, sign);
  value->type = RC_VALUE_TYPE_FLOAT;
}

static void rc_transform_memref_mbf32(rc_typed_value_t* value) {
  /* decodes a Microsoft Binary Format float */
  /* NOTE: 32-bit MBF is stored in memory as big endian (at least for Apple II) */
  const uint32_t mantissa = ((value->value.u32 & 0xFF000000) >> 24) |
                            ((value->value.u32 & 0x00FF0000) >> 8) |
                            ((value->value.u32 & 0x00007F00) << 8);
  const int32_t exponent = (int32_t)(value->value.u32 & 0xFF) - 129;
  const int sign = (value->value.u32 & 0x00008000);

  if (mantissa == 0 && exponent == -129)
    value->value.f32 = (sign) ? -0.0f : 0.0f;
  else
    value->value.f32 = rc_build_float(mantissa, exponent, sign);

  value->type = RC_VALUE_TYPE_FLOAT;
}

static void rc_transform_memref_mbf32_le(rc_typed_value_t* value) {
  /* decodes a Microsoft Binary Format float */
  /* Locomotive BASIC (CPC) uses MBF40, but in little endian format */
  const uint32_t mantissa = value->value.u32 & 0x007FFFFF;
  const int32_t exponent = (int32_t)(value->value.u32 >> 24) - 129;
  const int sign = (value->value.u32 & 0x00800000);

  if (mantissa == 0 && exponent == -129)
    value->value.f32 = (sign) ? -0.0f : 0.0f;
  else
    value->value.f32 = rc_build_float(mantissa, exponent, sign);

  value->type = RC_VALUE_TYPE_FLOAT;
}

static const uint8_t rc_bits_set[16] = { 0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4 };

void rc_transform_memref_value(rc_typed_value_t* value, uint8_t size) {
  /* ASSERT: value->type == RC_VALUE_TYPE_UNSIGNED */
  switch (size)
  {
    case RC_MEMSIZE_8_BITS:
      value->value.u32 = (value->value.u32 & 0x000000ff);
      break;

    case RC_MEMSIZE_16_BITS:
      value->value.u32 = (value->value.u32 & 0x0000ffff);
      break;

    case RC_MEMSIZE_24_BITS:
      value->value.u32 = (value->value.u32 & 0x00ffffff);
      break;

    case RC_MEMSIZE_32_BITS:
      break;

    case RC_MEMSIZE_BIT_0:
      value->value.u32 = (value->value.u32 >> 0) & 1;
      break;

    case RC_MEMSIZE_BIT_1:
      value->value.u32 = (value->value.u32 >> 1) & 1;
      break;

    case RC_MEMSIZE_BIT_2:
      value->value.u32 = (value->value.u32 >> 2) & 1;
      break;

    case RC_MEMSIZE_BIT_3:
      value->value.u32 = (value->value.u32 >> 3) & 1;
      break;

    case RC_MEMSIZE_BIT_4:
      value->value.u32 = (value->value.u32 >> 4) & 1;
      break;

    case RC_MEMSIZE_BIT_5:
      value->value.u32 = (value->value.u32 >> 5) & 1;
      break;

    case RC_MEMSIZE_BIT_6:
      value->value.u32 = (value->value.u32 >> 6) & 1;
      break;

    case RC_MEMSIZE_BIT_7:
      value->value.u32 = (value->value.u32 >> 7) & 1;
      break;

    case RC_MEMSIZE_LOW:
      value->value.u32 = value->value.u32 & 0x0f;
      break;

    case RC_MEMSIZE_HIGH:
      value->value.u32 = (value->value.u32 >> 4) & 0x0f;
      break;

    case RC_MEMSIZE_BITCOUNT:
      value->value.u32 = rc_bits_set[(value->value.u32 & 0x0F)]
                       + rc_bits_set[((value->value.u32 >> 4) & 0x0F)];
      break;

    case RC_MEMSIZE_16_BITS_BE:
      value->value.u32 = ((value->value.u32 & 0xFF00) >> 8) |
                         ((value->value.u32 & 0x00FF) << 8);
      break;

    case RC_MEMSIZE_24_BITS_BE:
      value->value.u32 = ((value->value.u32 & 0xFF0000) >> 16) |
                          (value->value.u32 & 0x00FF00) |
                         ((value->value.u32 & 0x0000FF) << 16);
      break;

    case RC_MEMSIZE_32_BITS_BE:
      value->value.u32 = ((value->value.u32 & 0xFF000000) >> 24) |
                         ((value->value.u32 & 0x00FF0000) >> 8) |
                         ((value->value.u32 & 0x0000FF00) << 8) |
                         ((value->value.u32 & 0x000000FF) << 24);
      break;

    case RC_MEMSIZE_FLOAT:
      rc_transform_memref_float(value);
      break;

    case RC_MEMSIZE_FLOAT_BE:
      rc_transform_memref_float_be(value);
      break;

    case RC_MEMSIZE_DOUBLE32:
      rc_transform_memref_double32(value);
      break;

    case RC_MEMSIZE_DOUBLE32_BE:
      rc_transform_memref_double32_be(value);
      break;

    case RC_MEMSIZE_MBF32:
      rc_transform_memref_mbf32(value);
      break;

    case RC_MEMSIZE_MBF32_LE:
      rc_transform_memref_mbf32_le(value);
      break;

    default:
      break;
  }
}

static const uint32_t rc_memref_masks[] = {
  0x000000ff, /* RC_MEMSIZE_8_BITS     */
  0x0000ffff, /* RC_MEMSIZE_16_BITS    */
  0x00ffffff, /* RC_MEMSIZE_24_BITS    */
  0xffffffff, /* RC_MEMSIZE_32_BITS    */
  0x0000000f, /* RC_MEMSIZE_LOW        */
  0x000000f0, /* RC_MEMSIZE_HIGH       */
  0x00000001, /* RC_MEMSIZE_BIT_0      */
  0x00000002, /* RC_MEMSIZE_BIT_1      */
  0x00000004, /* RC_MEMSIZE_BIT_2      */
  0x00000008, /* RC_MEMSIZE_BIT_3      */
  0x00000010, /* RC_MEMSIZE_BIT_4      */
  0x00000020, /* RC_MEMSIZE_BIT_5      */
  0x00000040, /* RC_MEMSIZE_BIT_6      */
  0x00000080, /* RC_MEMSIZE_BIT_7      */
  0x000000ff, /* RC_MEMSIZE_BITCOUNT   */
  0x0000ffff, /* RC_MEMSIZE_16_BITS_BE */
  0x00ffffff, /* RC_MEMSIZE_24_BITS_BE */
  0xffffffff, /* RC_MEMSIZE_32_BITS_BE */
  0xffffffff, /* RC_MEMSIZE_FLOAT      */
  0xffffffff, /* RC_MEMSIZE_MBF32      */
  0xffffffff, /* RC_MEMSIZE_MBF32_LE   */
  0xffffffff, /* RC_MEMSIZE_FLOAT_BE   */
  0xffffffff, /* RC_MEMSIZE_DOUBLE32   */
  0xffffffff, /* RC_MEMSIZE_DOUBLE32_BE*/
  0xffffffff  /* RC_MEMSIZE_VARIABLE   */
};

uint32_t rc_memref_mask(uint8_t size) {
  const size_t index = (size_t)size;
  if (index >= sizeof(rc_memref_masks) / sizeof(rc_memref_masks[0]))
    return 0xffffffff;

  return rc_memref_masks[index];
}

/* all sizes less than 8-bits (1 byte) are mapped to 8-bits. 24-bit is mapped to 32-bit
 * as we don't expect the client to understand a request for 3 bytes. all other reads are
 * mapped to the little-endian read of the same size. */
static const uint8_t rc_memref_shared_sizes[] = {
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_8_BITS     */
  RC_MEMSIZE_16_BITS, /* RC_MEMSIZE_16_BITS    */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_24_BITS    */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_32_BITS    */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_LOW        */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_HIGH       */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_0      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_1      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_2      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_3      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_4      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_5      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_6      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BIT_7      */
  RC_MEMSIZE_8_BITS,  /* RC_MEMSIZE_BITCOUNT   */
  RC_MEMSIZE_16_BITS, /* RC_MEMSIZE_16_BITS_BE */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_24_BITS_BE */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_32_BITS_BE */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_FLOAT      */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_MBF32      */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_MBF32_LE   */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_FLOAT_BE   */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_DOUBLE32   */
  RC_MEMSIZE_32_BITS, /* RC_MEMSIZE_DOUBLE32_BE*/
  RC_MEMSIZE_32_BITS  /* RC_MEMSIZE_VARIABLE   */
};

uint8_t rc_memref_shared_size(uint8_t size) {
  const size_t index = (size_t)size;
  if (index >= sizeof(rc_memref_shared_sizes) / sizeof(rc_memref_shared_sizes[0]))
    return size;

  return rc_memref_shared_sizes[index];
}

uint32_t rc_peek_value(uint32_t address, uint8_t size, rc_peek_t peek, void* ud) {
  if (!peek)
    return 0;

  switch (size)
  {
    case RC_MEMSIZE_8_BITS:
      return peek(address, 1, ud);

    case RC_MEMSIZE_16_BITS:
      return peek(address, 2, ud);

    case RC_MEMSIZE_32_BITS:
      return peek(address, 4, ud);

    default:
    {
      uint32_t value;
      const size_t index = (size_t)size;
      if (index >= sizeof(rc_memref_shared_sizes) / sizeof(rc_memref_shared_sizes[0]))
        return 0;

      /* fetch the larger value and mask off the bits associated to the specified size
       * for correct deduction of prior value. non-prior memrefs should already be using
       * shared size memrefs to minimize the total number of memory reads required. */
      value = rc_peek_value(address, rc_memref_shared_sizes[index], peek, ud);
      return value & rc_memref_masks[index];
    }
  }
}

void rc_update_memref_value(rc_memref_value_t* memref, uint32_t new_value) {
  if (memref->value == new_value) {
    memref->changed = 0;
  }
  else {
    memref->prior = memref->value;
    memref->value = new_value;
    memref->changed = 1;
  }
}

void rc_init_parse_state_memrefs(rc_parse_state_t* parse, rc_memrefs_t* memrefs)
{
  if (memrefs)
    memset(memrefs, 0, sizeof(*memrefs));

  parse->memrefs = memrefs;
}

static uint32_t rc_get_memref_value_value(const rc_memref_value_t* memref, int operand_type) {
  switch (operand_type)
  {
    /* most common case explicitly first, even though it could be handled by default case.
     * this helps the compiler to optimize if it turns the switch into a series of if/elses */
    case RC_OPERAND_ADDRESS:
      return memref->value;

    case RC_OPERAND_DELTA:
      if (!memref->changed) {
        /* fallthrough */
    default:
        return memref->value;
      }
      /* fallthrough */
    case RC_OPERAND_PRIOR:
      return memref->prior;
  }
}

void rc_get_memref_value(rc_typed_value_t* value, rc_memref_t* memref, int operand_type) {
  value->type = memref->value.type;
  value->value.u32 = rc_get_memref_value_value(&memref->value, operand_type);
}

uint32_t rc_get_modified_memref_value(const rc_modified_memref_t* memref, rc_peek_t peek, void* ud) {
  rc_typed_value_t value, modifier;

  rc_evaluate_operand(&value, &memref->parent, NULL);
  rc_evaluate_operand(&modifier, &memref->modifier, NULL);

  switch (memref->modifier_type) {
    case RC_OPERATOR_INDIRECT_READ:
      rc_typed_value_add(&value, &modifier);
      rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
      value.value.u32 = rc_peek_value(value.value.u32, memref->memref.value.size, peek, ud);
      value.type = memref->memref.value.type;
      break;

    case RC_OPERATOR_SUB_PARENT:
      /* sub parent is "-parent + modifier" */
      rc_typed_value_negate(&value);
      rc_typed_value_add(&value, &modifier);
      rc_typed_value_convert(&value, memref->memref.value.type);
      break;

    case RC_OPERATOR_SUB_ACCUMULATOR:
      rc_typed_value_negate(&modifier);
      /* fallthrough */ /* to case RC_OPERATOR_SUB_ACCUMULATOR */

    case RC_OPERATOR_ADD_ACCUMULATOR:
      /* when modifying the accumulator, force the modifier to match the accumulator
       * type instead of promoting them both to the less restrictive type.
       *
       *   18 - 17.5  will result in an integer. should it be 0 or 1?
       *
       * default: float is less restrictive, convert both to float for combine,
       *          then convert to the memref type.
       *   (int)((float)18 - 17.5) -> (int)(0.5) -> 0
       *
       * accumulator is integer: force modifier to be integer before combining
       *   (int)(18 - (int)17.5) -> (int)(18 - 17) -> 1
       */
      rc_typed_value_convert(&modifier, value.type);
      rc_typed_value_add(&value, &modifier);
      rc_typed_value_convert(&value, memref->memref.value.type);
      break;

    default:
      rc_typed_value_combine(&value, &modifier, memref->modifier_type);
      rc_typed_value_convert(&value, memref->memref.value.type);
      break;
  }

  return value.value.u32;
}

#ifdef RC_INCREMENTAL_UPD
/* opt B (deterministic, host-tested slice): true if this operand's VALUE cannot
 * have changed this frame, so a value-transform modified_memref built only from
 * stable operands keeps its value (and we can skip re-evaluating it). CONST/FP
 * are fixed; ADDRESS/BCD/INVERTED read a memref's CURRENT value -> stable iff it
 * didn't change. DELTA/PRIOR shift the frame AFTER a change (the "settle") and
 * RECALL/FUNC depend on external accumulator state -> treated as NOT stable
 * (always re-resolve) to stay correct without extra per-memref bookkeeping. */
static int rc_operand_value_stable(const rc_operand_t* op) {
  switch (op->type) {
    case RC_OPERAND_CONST:
    case RC_OPERAND_FP:
      return 1;
    case RC_OPERAND_ADDRESS:
    case RC_OPERAND_BCD:
    case RC_OPERAND_INVERTED:
      return rc_operand_is_memref(op) && !op->value.memref->value.changed;
    default: /* DELTA, PRIOR, RECALL, FUNC */
      return 0;
  }
}

/* opt B2 (device-only): adapter hook. Returns nonzero iff every one of the
 * num_bytes at `address` is present in the snapshot cache AND none changed in
 * the latest snapshot. For an INDIRECT_READ whose pointer (parent+modifier) is
 * stable, the leaf address equals last frame's, so an unchanged leaf means the
 * resolved value is unchanged -> skip. NULL (default) => never skip INDIRECT.
 * Set by the adapter at init; gated by g_rc_indirect_skip_enabled for A/B. */
int (*g_rc_leaf_unchanged)(uint32_t address, uint32_t num_bytes) = 0;
volatile int g_rc_indirect_skip_enabled = 1;

/* instrumentation (read+reset by the adapter's FRAME log): per-do_frame counts
 * of B1 value-transform skips, B2 INDIRECT skips, and modified_memrefs that
 * actually re-resolved. */
unsigned long g_rc_b1_skips = 0;
unsigned long g_rc_b2_skips = 0;
unsigned long g_rc_upd_resolves = 0;

/* size enum -> number of memory bytes the leaf read actually touches (the
 * SHARED size; sub-byte sizes share an 8-bit read). Conservative: a sub-byte
 * field whose containing byte changed re-resolves even if its nibble didn't. */
static uint32_t rc_indirect_num_bytes(uint8_t size) {
  switch (rc_memref_shared_size(size)) {
    case RC_MEMSIZE_16_BITS: return 2;
    case RC_MEMSIZE_24_BITS: return 3;
    case RC_MEMSIZE_32_BITS: return 4;
    default:                 return 1;
  }
}

/* opt B (shared by rc_update_memref_values AND rc_client_update_memref_values —
 * the device's do_frame uses the latter, so this helper must be reachable from
 * both). Returns 1 if `mm` can keep its value this frame (caller skips the
 * re-resolve): incr (warmed) AND parent+modifier stable AND either a
 * non-INDIRECT value transform (B1) or an INDIRECT_READ whose leaf bytes didn't
 * move per the adapter hook (B2). Clears .changed + bumps a counter on a skip.
 * Skip-with-changed=0 == resolving an unchanged value (prior/delta stay right). */
int rc_modified_memref_can_skip(rc_modified_memref_t* mm, int incr) {
  if (!incr
      || !rc_operand_value_stable(&mm->parent)
      || !rc_operand_value_stable(&mm->modifier))
    return 0;

  if (mm->modifier_type != RC_OPERATOR_INDIRECT_READ) {
    mm->memref.value.changed = 0;     /* B1 */
    ++g_rc_b1_skips;
    return 1;
  }

  if (g_rc_indirect_skip_enabled && g_rc_leaf_unchanged) {  /* B2 */
    rc_typed_value_t a, m;
    rc_evaluate_operand(&a, &mm->parent, NULL);
    rc_evaluate_operand(&m, &mm->modifier, NULL);
    rc_typed_value_add(&a, &m);
    rc_typed_value_convert(&a, RC_VALUE_TYPE_UNSIGNED);
    if (g_rc_leaf_unchanged(a.value.u32, rc_indirect_num_bytes(mm->memref.value.size))) {
      mm->memref.value.changed = 0;
      ++g_rc_b2_skips;
      return 1;
    }
  }
  return 0;
}
#endif

void rc_update_memref_values(rc_memrefs_t* memrefs, rc_peek_t peek, void* ud) {
  rc_memref_list_t* memref_list;
  rc_modified_memref_list_t* modified_memref_list;

  memref_list = &memrefs->memrefs;
  do
  {
    rc_memref_t* memref = memref_list->items;
    const rc_memref_t* memref_stop = memref + memref_list->count;

    for (; memref < memref_stop; ++memref) {
      if (memref->value.type != RC_VALUE_TYPE_NONE)
        rc_update_memref_value(&memref->value, rc_peek_value(memref->address, memref->value.size, peek, ud));
#ifdef RC_SHADOW_VALUES
      rc_shadow_mirror(memref);
#endif
    }

    memref_list = memref_list->next;
  } while (memref_list);

  modified_memref_list = &memrefs->modified_memrefs;
  if (modified_memref_list->count) {
#ifdef RC_INCREMENTAL_UPD
    /* opt B: skipping is allowed only AFTER the first full pass has populated
     * every chain's value (the 1st pass must resolve all). Flag lives in the
     * head memref list's padding (see rc_memref_list_t). */
    const int incr = memrefs->memrefs.incr_warmed;
#endif
    do {
      rc_modified_memref_t* modified_memref = modified_memref_list->items;
      const rc_modified_memref_t* modified_memref_stop = modified_memref + modified_memref_list->count;

      for (; modified_memref < modified_memref_stop; ++modified_memref) {
#ifdef RC_INCREMENTAL_UPD
        /* Depth order guarantees a parent modified_memref is resolved before
         * this one, so its .changed is fresh when the helper inspects it. */
        if (rc_modified_memref_can_skip(modified_memref, incr))
          continue;
        ++g_rc_upd_resolves;
#endif
        rc_update_memref_value(&modified_memref->memref.value, rc_get_modified_memref_value(modified_memref, peek, ud));
#ifdef RC_SHADOW_VALUES
        rc_shadow_mirror(&modified_memref->memref);
#endif
      }

      modified_memref_list = modified_memref_list->next;
    } while (modified_memref_list);
  }

#ifdef RC_INCREMENTAL_UPD
  memrefs->memrefs.incr_warmed = 1;
#endif
}

/* === Prefetch API for external memory adapters (e.g. GameCube RA Adapter) ===
 *
 * Collects all memory addresses that will be read on the next call to
 * rc_update_memref_values(). For static memrefs, this is the address field.
 * For modified memrefs with RC_OPERATOR_INDIRECT_READ, the final computed
 * address is resolved using either:
 *   - A caller-provided peek callback (if non-NULL), which reads FRESH memory
 *     values from the current snapshot, OR
 *   - The stored memref values from the last do_frame (if peek is NULL).
 *
 * This function is READ-ONLY: it does NOT modify any state (no hit counts,
 * no delta/prior updates, no value changes).
 *
 * peek/peek_ud:   Optional callback to read memory for resolving indirect
 *                 addresses. When provided, the base pointer value is read
 *                 from the current snapshot rather than stale memref state.
 * out_addresses:  caller-provided array to receive (address, num_bytes) pairs.
 * out_capacity:   maximum number of entries in out_addresses.
 * Returns:        number of entries written. If return value == out_capacity,
 *                 the array may have been truncated.
 */
uint32_t rc_memrefs_get_addresses(const rc_memrefs_t* memrefs,
                                  uint32_t* out_addresses,
                                  uint8_t* out_sizes,
                                  uint32_t out_capacity,
                                  rc_peek_t peek,
                                  void* peek_ud) {
  const rc_memref_list_t* memref_list;
  const rc_modified_memref_list_t* modified_list;
  uint32_t count = 0;
  uint32_t i;

  if (!memrefs || !out_addresses || !out_sizes || out_capacity == 0)
    return 0;

  /* 1. Collect static memref addresses */
  memref_list = &memrefs->memrefs;
  do {
    const rc_memref_t* m = memref_list->items;
    const rc_memref_t* m_stop = m + memref_list->count;

    for (; m < m_stop; ++m) {
      uint8_t shared_size;
      uint8_t num_bytes;
      int already_present;

      if (m->value.type == RC_VALUE_TYPE_NONE)
        continue;

      shared_size = rc_memref_shared_size(m->value.size);
      switch (shared_size) {
        case RC_MEMSIZE_8_BITS:  num_bytes = 1; break;
        case RC_MEMSIZE_16_BITS: num_bytes = 2; break;
        case RC_MEMSIZE_32_BITS: num_bytes = 4; break;
        default:                 num_bytes = 1; break;
      }

      /* deduplicate */
      already_present = 0;
      for (i = 0; i < count; ++i) {
        if (out_addresses[i] == m->address && out_sizes[i] == num_bytes) {
          already_present = 1;
          break;
        }
      }

      if (!already_present && count < out_capacity) {
        out_addresses[count] = m->address;
        out_sizes[count] = num_bytes;
        ++count;
      }
    }

    memref_list = memref_list->next;
  } while (memref_list);

  /* 2. Collect modified memref addresses (resolved with current values) */
  modified_list = &memrefs->modified_memrefs;
  if (modified_list->count) {
    do {
      const rc_modified_memref_t* mm = modified_list->items;
      const rc_modified_memref_t* mm_stop = mm + modified_list->count;

      for (; mm < mm_stop; ++mm) {
        if (mm->modifier_type == RC_OPERATOR_INDIRECT_READ) {
          rc_typed_value_t parent_val, modifier_val;
          uint32_t resolved_addr;
          uint8_t num_bytes;
          int already_present;

          if (peek) {
            /* Use fresh snapshot values via caller-provided peek callback.
             * This resolves the base pointer using the CURRENT frame's data
             * rather than stale values from the last do_frame. */
            parent_val.type = RC_VALUE_TYPE_UNSIGNED;
            parent_val.value.u32 = rc_peek_value(
                mm->parent.value.memref->address,
                mm->parent.value.memref->value.size,
                peek, peek_ud);
          } else {
            /* Fall back to stored memref values (from last do_frame) */
            rc_evaluate_operand(&parent_val, &mm->parent, NULL);
          }

          rc_evaluate_operand(&modifier_val, &mm->modifier, NULL);
          rc_typed_value_add(&parent_val, &modifier_val);
          rc_typed_value_convert(&parent_val, RC_VALUE_TYPE_UNSIGNED);
          resolved_addr = parent_val.value.u32;

          {
            uint8_t shared_size = rc_memref_shared_size(mm->memref.value.size);
            switch (shared_size) {
              case RC_MEMSIZE_8_BITS:  num_bytes = 1; break;
              case RC_MEMSIZE_16_BITS: num_bytes = 2; break;
              case RC_MEMSIZE_32_BITS: num_bytes = 4; break;
              default:                 num_bytes = 1; break;
            }
          }

          /* deduplicate */
          already_present = 0;
          for (i = 0; i < count; ++i) {
            if (out_addresses[i] == resolved_addr && out_sizes[i] == num_bytes) {
              already_present = 1;
              break;
            }
          }

          if (!already_present && count < out_capacity) {
            out_addresses[count] = resolved_addr;
            out_sizes[count] = num_bytes;
            ++count;
          }
        }
      }

      modified_list = modified_list->next;
    } while (modified_list);
  }

  return count;
}

/* Address byte-width for a memref size (shared-size collapsed). */
static uint8_t rc_memref_addr_bytes(uint8_t size) {
  switch (rc_memref_shared_size(size)) {
    case RC_MEMSIZE_8_BITS:  return 1;
    case RC_MEMSIZE_16_BITS: return 2;
    case RC_MEMSIZE_32_BITS: return 4;
    default:                 return 1;
  }
}

/* Resolve the RAW (untransformed) value a memref yields — the equivalent of
 * its stored memref->value — reading every level through the caller's cache
 * via peek and recursing through modifier chains so pointer paths resolve
 * within a single frame. If any address along the path is not yet cached, the
 * FIRST such address is reported via *pending_* and *found_miss is set; the
 * walk unwinds immediately (no cascade into garbage built on an unresolved
 * pointer).
 *
 * CRITICAL: the size transform (rc_transform_memref_value) is applied to a
 * parent value using the PARENT OPERAND's size (mm->parent.size), exactly as
 * rc_evaluate_operand does — NOT the parent memref's own size. The two can
 * differ, and the trigger evaluator uses the operand size; using the wrong
 * one makes the resolved child address differ from what do_frame computes, so
 * the resolver silently misses what the evaluator then reads (cm=0, ms>0).
 * READ-ONLY: never writes any memref value/prior/address. */
#ifdef RC_INCREMENTAL_COLLECT
/* Reverse-hash incremental collect state. g_rc_incr_chain is the chain currently
 * being walked (so rc_resolve_cached_raw's reads can be attributed to it via the
 * adapter's reverse-index hook). */
void (*g_rc_chain_read_cb)(uint32_t address, uint8_t num_bytes, void* chain) = 0;
volatile int g_rc_incr_collect_enabled = 0;
volatile unsigned long g_rc_collect_skips = 0, g_rc_collect_walks = 0;  /* per-collect diag */
static void* g_rc_incr_chain = 0;

void rc_modified_memref_mark_dirty(void* chain) {
  if (chain) ((rc_modified_memref_t*)chain)->incr_clean = 0;
}

void rc_modified_memrefs_mark_all_dirty(const rc_memrefs_t* memrefs) {
  const rc_modified_memref_list_t* list;
  if (!memrefs) return;
  list = &memrefs->modified_memrefs;
  do {
    rc_modified_memref_t* mm = list->items;
    const rc_modified_memref_t* stop = mm + list->count;
    for (; mm < stop; ++mm) mm->incr_clean = 0;
    list = list->next;
  } while (list);
}
#endif

/* Resolver/evaluator divergence-fix instrumentation (2026-06-20). Counts how often
 * each audited parent-resolution path is taken so we can confirm on-device that the
 * fix is exercised and the peek_miss/df=0 rate drops. d1 = static DELTA/PRIOR parent
 * read from stored frame-history (matches the evaluator); d2 = BCD/INVERT transform
 * that actually changed the parent value; chain_dp = DELTA/PRIOR-of-a-chain (no stored
 * prior to reconstruct — falls through to current value, a rare residual the peek_miss
 * net backstops). The collect walk is single-threaded, so plain volatile, no atomics. */
volatile int g_rc_resfix_enabled = 1;   /* A/B + safety toggle: 0 = old diverging behavior */
volatile unsigned long g_rc_resfix_d1 = 0, g_rc_resfix_d2 = 0, g_rc_resfix_chain_dp = 0;

/* Pointer-aware dirtying (RC_POINTER_AWARE_DIRTY): only register POINTER reads (the
 * intermediate chain levels, reached via recursion at depth>0) in the reverse index,
 * NOT the top-level LEAF read (depth 0). A chain's resolved leaf ADDRESS only changes
 * when a pointer above it MOVES; the leaf VALUE changing (most snap_changed churn) does
 * NOT need a re-walk. Registering the leaf made it falsely-dirty every time its value
 * changed -> ~9 wasted walks per real miss (measured). Skipping it cuts that. SAFE: a
 * missed dirty (wrongly-skipped pointer) is backstopped by the do_frame gate + peek_miss
 * net (deferred frame, never a false unlock). Device-only (no upstream/host coverage). */
#ifdef RC_POINTER_AWARE_DIRTY
#define RC_REGISTER_READ(depth) ((depth) > 0)
#else
#define RC_REGISTER_READ(depth) 1
#endif

static uint32_t rc_resolve_cached_raw(const rc_memref_t* m,
                                      rc_peek_t peek, rc_is_cached_t is_cached, void* ud,
                                      uint32_t* pending_addr, uint8_t* pending_size,
                                      int* found_miss, int depth) {
  rc_typed_value_t value, modifier;
  const rc_modified_memref_t* mm;
  uint8_t num_bytes;

  if (*found_miss || !m)
    return 0;

  /* Static (root) memref — raw stored value == rc_peek_value (no transform;
   * the caller applies it with the operand size, like rc_evaluate_operand). */
  if (m->value.memref_type != RC_MEMREF_TYPE_MODIFIED_MEMREF) {
    num_bytes = rc_memref_addr_bytes(m->value.size);
#ifdef RC_INCREMENTAL_COLLECT
    /* Record this read against the chain being walked (reverse index) — pointers only. */
    if (g_rc_chain_read_cb && g_rc_incr_chain && RC_REGISTER_READ(depth))
      g_rc_chain_read_cb(m->address, num_bytes, g_rc_incr_chain);
#endif
    if (!is_cached(m->address, num_bytes, ud)) {
      *pending_addr = m->address;
      *pending_size = num_bytes;
      *found_miss = 1;
      return 0;
    }
    return rc_peek_value(m->address, m->value.size, peek, ud);
  }

  /* Modified memref — the rc_memref_t is the first field, so cast back. */
  mm = (const rc_modified_memref_t*)m;

  /* Resolve the parent to EXACTLY mirror rc_evaluate_operand (operand.c:559) — all
   * three steps: (1) value by operand TYPE (current/delta/prior), (2) size mask via
   * rc_transform_memref_value, (3) BCD/INVERT via rc_transform_operand_value. The
   * resolver historically did only step 2; D1/D2 below complete it (see the audit).
   * The parent is USUALLY a memref, but can be a constant/float — only recurse when
   * it really points at a memref, else evaluate it directly. (Recursing on a
   * non-memref operand reads value.memref = garbage/NULL and faults: LoadProhibited
   * at offset 0xb, the next level's memref_type.) */
  value.type = RC_VALUE_TYPE_UNSIGNED;
  if (rc_operand_type_is_memref(mm->parent.type)) {
    const rc_operand_t* p = &mm->parent;
    /* D1 (2026-06-20): a DELTA/PRIOR pointer reads stored frame-history
     * (memref->prior), which a fresh snapshot peek CANNOT reproduce. For a STATIC
     * parent the stored typed value IS maintained (rc_update_memref_values runs
     * before collect), so read it exactly like rc_evaluate_operand -> the predicted
     * leaf matches the evaluator. DELTA/PRIOR-of-a-chain has no stored prior; it
     * falls through to the recursion (current value) — a rare residual the do_frame
     * gate's peek_miss net still backstops (no false unlock, just a deferred frame). */
    if (g_rc_resfix_enabled &&
        (p->type == RC_OPERAND_DELTA || p->type == RC_OPERAND_PRIOR) &&
        p->value.memref &&
        p->value.memref->value.memref_type != RC_MEMREF_TYPE_MODIFIED_MEMREF) {
      rc_get_memref_value(&value, p->value.memref, p->type);
      ++g_rc_resfix_d1;
    } else {
      if (g_rc_resfix_enabled && (p->type == RC_OPERAND_DELTA || p->type == RC_OPERAND_PRIOR))
        ++g_rc_resfix_chain_dp;   /* delta/prior of a chain: unreconstructable residual */
      value.value.u32 = rc_resolve_cached_raw(p->value.memref, peek, is_cached, ud,
                                              pending_addr, pending_size, found_miss, depth + 1);
      if (*found_miss)
        return 0;
    }
    rc_transform_memref_value(&value, p->size);
    /* D2 (2026-06-20): mirror rc_evaluate_operand step 3 — BCD-decode / bitwise-INVERT
     * the parent pointer. The resolver used to skip this, so a BCD/INVERTED pointer
     * predicted a different leaf than the evaluator read -> peek_miss -> df=0. */
    if (g_rc_resfix_enabled && value.type == RC_VALUE_TYPE_UNSIGNED) {
      uint32_t pre = value.value.u32;
      value.value.u32 = rc_transform_operand_value(value.value.u32, p);
      if (value.value.u32 != pre) ++g_rc_resfix_d2;
    }
    rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
  } else {
    rc_evaluate_operand(&value, &mm->parent, NULL);
    rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
  }

  rc_evaluate_operand(&modifier, &mm->modifier, NULL);

  switch (mm->modifier_type) {
    case RC_OPERATOR_INDIRECT_READ: {
      uint32_t addr;
      rc_typed_value_add(&value, &modifier);
      rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
      addr = value.value.u32;
      num_bytes = rc_memref_addr_bytes(m->value.size);
#ifdef RC_INCREMENTAL_COLLECT
      if (g_rc_chain_read_cb && g_rc_incr_chain && RC_REGISTER_READ(depth))
        g_rc_chain_read_cb(addr, num_bytes, g_rc_incr_chain);
#endif
      if (!is_cached(addr, num_bytes, ud)) {
        *pending_addr = addr;
        *pending_size = num_bytes;
        *found_miss = 1;
        return 0;
      }
      return rc_peek_value(addr, m->value.size, peek, ud);
    }

    case RC_OPERATOR_SUB_PARENT:
      rc_typed_value_negate(&value);
      rc_typed_value_add(&value, &modifier);
      rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
      return value.value.u32;

    case RC_OPERATOR_SUB_ACCUMULATOR:
      rc_typed_value_negate(&modifier);
      /* fallthrough */
    case RC_OPERATOR_ADD_ACCUMULATOR:
      rc_typed_value_convert(&modifier, value.type);
      rc_typed_value_add(&value, &modifier);
      rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
      return value.value.u32;

    default:
      rc_typed_value_combine(&value, &modifier, mm->modifier_type);
      rc_typed_value_convert(&value, RC_VALUE_TYPE_UNSIGNED);
      return value.value.u32;
  }
}

uint32_t rc_memrefs_get_pending_addresses(const rc_memrefs_t* memrefs,
                                          uint32_t* out_addresses,
                                          uint8_t* out_sizes,
                                          uint32_t out_capacity,
                                          rc_peek_t peek,
                                          rc_is_cached_t is_cached,
                                          void* ud) {
  const rc_modified_memref_list_t* modified_list;
  uint32_t count = 0;
  uint32_t i;

  if (!memrefs || !out_addresses || !out_sizes || out_capacity == 0 || !is_cached)
    return 0;

  modified_list = &memrefs->modified_memrefs;
  if (!modified_list->count)
    return 0;

  do {
    const rc_modified_memref_t* mm = modified_list->items;
    const rc_modified_memref_t* mm_stop = mm + modified_list->count;

    for (; mm < mm_stop; ++mm) {
      uint32_t pending_addr = 0;
      uint8_t pending_size = 1;
      int found_miss = 0;
      int already_present;

      /* Only INDIRECT chains introduce fetchable RAM addresses; other
       * modifiers (mask/accumulate) are value transforms with no own
       * address. (Their parents are still walked when reached via an
       * INDIRECT leaf's recursion.) */
      if (mm->modifier_type != RC_OPERATOR_INDIRECT_READ)
        continue;

#ifdef RC_INCREMENTAL_COLLECT
      /* Reverse-hash incremental: a CLEAN chain's inputs are unchanged since its
       * last fully-resolved walk -> its leaf is still cached -> nothing to fetch.
       * Skip the walk. (Dirtied by the adapter when a read address changes or the
       * watchlist mutates; a full rebuild periodically re-walks everything.) */
      if (g_rc_incr_collect_enabled && mm->incr_clean) {
        ++g_rc_collect_skips;
        continue;
      }
      ++g_rc_collect_walks;
      g_rc_incr_chain = (void*)mm;   /* attribute this walk's reads to this chain */
#endif

      /* Walk this chain's leaf through the cache; the first uncached
       * address on the path is reported. */
      rc_resolve_cached_raw(&mm->memref, peek, is_cached, ud,
                            &pending_addr, &pending_size, &found_miss, 0);

#ifdef RC_INCREMENTAL_COLLECT
      /* Clean iff fully resolved (no miss). A miss keeps it dirty so the next
       * collect re-walks until the leaf is cached. */
      if (g_rc_incr_collect_enabled)
        ((rc_modified_memref_t*)mm)->incr_clean = found_miss ? 0 : 1;
#endif
      if (!found_miss)
        continue;

      /* deduplicate */
      already_present = 0;
      for (i = 0; i < count; ++i) {
        if (out_addresses[i] == pending_addr && out_sizes[i] == pending_size) {
          already_present = 1;
          break;
        }
      }

      if (!already_present && count < out_capacity) {
        out_addresses[count] = pending_addr;
        out_sizes[count] = pending_size;
        ++count;
      }
    }

    modified_list = modified_list->next;
  } while (modified_list);

#ifdef RC_INCREMENTAL_COLLECT
  g_rc_incr_chain = 0;   /* don't attribute reads outside the collect walk */
#endif
  return count;
}
