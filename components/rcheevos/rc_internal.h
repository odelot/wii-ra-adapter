#ifndef RC_INTERNAL_H
#define RC_INTERNAL_H

#include "rc_runtime_types.h"
#include "rc_util.h"

RC_BEGIN_C_DECLS

typedef struct rc_scratch_string {
  char* value;
  struct rc_scratch_string* left;
  struct rc_scratch_string* right;
}
rc_scratch_string_t;

typedef struct rc_modified_memref_t {
  rc_memref_t memref;              /* For compatibility with rc_operand_t.value.memref */
  rc_operand_t parent;             /* The parent memref this memref is derived from (type will always be a memref type) */
  rc_operand_t modifier;           /* The modifier to apply to the parent. */
  uint8_t modifier_type;           /* How to apply the modifier to the parent. (RC_OPERATOR_*) */
  uint16_t depth;                  /* The number of parents this memref has. */
}
rc_modified_memref_t;

typedef struct rc_memref_list_t {
  rc_memref_t* items;
  struct rc_memref_list_t* next;
  uint16_t count;
  uint16_t capacity;
  uint8_t allocated;
} rc_memref_list_t;

typedef struct rc_modified_memref_list_t {
  rc_modified_memref_t* items;
  struct rc_modified_memref_list_t* next;
  uint16_t count;
  uint16_t capacity;
  uint8_t allocated;
} rc_modified_memref_list_t;

typedef struct rc_memrefs_t {
  rc_memref_list_t memrefs;
  rc_modified_memref_list_t modified_memrefs;
} rc_memrefs_t;

typedef struct rc_trigger_with_memrefs_t {
  rc_trigger_t trigger;
  rc_memrefs_t memrefs;
} rc_trigger_with_memrefs_t;

typedef struct rc_lboard_with_memrefs_t {
  rc_lboard_t lboard;
  rc_memrefs_t memrefs;
} rc_lboard_with_memrefs_t;

typedef struct rc_richpresence_with_memrefs_t {
  rc_richpresence_t richpresence;
  rc_memrefs_t memrefs;
} rc_richpresence_with_memrefs_t;

typedef struct rc_value_with_memrefs_t {
  rc_value_t value;
  rc_memrefs_t memrefs;
} rc_value_with_memrefs_t;

/* enum helpers for natvis expansion. Have to use a struct to define the mapping,
 * and a single field to allow the conditional logic to switch on the value */
typedef struct __rc_bool_enum_t { uint8_t value; } __rc_bool_enum_t;
typedef struct __rc_memsize_enum_t { uint8_t value; } __rc_memsize_enum_t;
typedef struct __rc_memsize_enum_func_t { uint8_t value; } __rc_memsize_enum_func_t;
typedef struct __rc_operand_enum_t { uint8_t value; } __rc_operand_enum_t;
typedef struct __rc_value_type_enum_t { uint8_t value; } __rc_value_type_enum_t;
typedef struct __rc_memref_type_enum_t { uint8_t value; } __rc_memref_type_enum_t;
typedef struct __rc_condition_enum_t { uint8_t value; } __rc_condition_enum_t;
typedef struct __rc_condition_enum_str_t { uint8_t value; } __rc_condition_enum_str_t;
typedef struct __rc_condset_list_t { rc_condset_t* first_condset; } __rc_condset_list_t;
typedef struct __rc_operator_enum_t { uint8_t value; } __rc_operator_enum_t;
typedef struct __rc_operator_enum_str_t { uint8_t value; } __rc_operator_enum_str_t;
typedef struct __rc_operand_memref_t { rc_operand_t operand; } __rc_operand_memref_t; /* requires &rc_operand_t to be the same as &rc_operand_t.value.memref */
typedef struct __rc_value_list_t { rc_value_t* first_value; } __rc_value_list_t;
typedef struct __rc_trigger_state_enum_t { uint8_t value; } __rc_trigger_state_enum_t;
typedef struct __rc_lboard_state_enum_t { uint8_t value; } __rc_lboard_state_enum_t;
typedef struct __rc_richpresence_display_list_t { rc_richpresence_display_t* first_display; } __rc_richpresence_display_list_t;
typedef struct __rc_richpresence_display_part_list_t { rc_richpresence_display_part_t* display; } __rc_richpresence_display_part_list_t;
typedef struct __rc_richpresence_lookup_list_t { rc_richpresence_lookup_t* first_lookup; } __rc_richpresence_lookup_list_t;
typedef struct __rc_format_enum_t { uint8_t value; } __rc_format_enum_t;

#define RC_ALLOW_ALIGN(T) struct __align_ ## T { uint8_t ch; T t; };

RC_ALLOW_ALIGN(rc_condition_t)
RC_ALLOW_ALIGN(rc_condset_t)
RC_ALLOW_ALIGN(rc_modified_memref_t)
RC_ALLOW_ALIGN(rc_lboard_t)
RC_ALLOW_ALIGN(rc_lboard_with_memrefs_t)
RC_ALLOW_ALIGN(rc_memref_t)
RC_ALLOW_ALIGN(rc_memref_list_t)
RC_ALLOW_ALIGN(rc_memrefs_t)
RC_ALLOW_ALIGN(rc_modified_memref_list_t)
RC_ALLOW_ALIGN(rc_operand_t)
RC_ALLOW_ALIGN(rc_richpresence_t)
RC_ALLOW_ALIGN(rc_richpresence_display_t)
RC_ALLOW_ALIGN(rc_richpresence_display_part_t)
RC_ALLOW_ALIGN(rc_richpresence_lookup_t)
RC_ALLOW_ALIGN(rc_richpresence_lookup_item_t)
RC_ALLOW_ALIGN(rc_richpresence_with_memrefs_t)
RC_ALLOW_ALIGN(rc_scratch_string_t)
RC_ALLOW_ALIGN(rc_trigger_t)
RC_ALLOW_ALIGN(rc_trigger_with_memrefs_t)
RC_ALLOW_ALIGN(rc_value_t)
RC_ALLOW_ALIGN(rc_value_with_memrefs_t)
RC_ALLOW_ALIGN(char)

#define RC_ALIGNOF(T) (sizeof(struct __align_ ## T) - sizeof(T))
#define RC_OFFSETOF(o, t) (int)((uint8_t*)&(o.t) - (uint8_t*)&(o))

#define RC_ALLOC(t, p) ((t*)rc_alloc((p)->buffer, &(p)->offset, sizeof(t), RC_ALIGNOF(t), &(p)->scratch, RC_OFFSETOF((p)->scratch.objs, __ ## t)))
#define RC_ALLOC_SCRATCH(t, p) ((t*)rc_alloc_scratch((p)->buffer, &(p)->offset, sizeof(t), RC_ALIGNOF(t), &(p)->scratch, RC_OFFSETOF((p)->scratch.objs, __ ## t)))
#define RC_ALLOC_ARRAY(t, n, p) ((t*)rc_alloc((p)->buffer, &(p)->offset, (n) * sizeof(t), RC_ALIGNOF(t), &(p)->scratch, RC_OFFSETOF((p)->scratch.objs, __ ## t)))
#define RC_ALLOC_ARRAY_SCRATCH(t, n, p) ((t*)rc_alloc_scratch((p)->buffer, &(p)->offset, (n) * sizeof(t), RC_ALIGNOF(t), &(p)->scratch, RC_OFFSETOF((p)->scratch.objs, __ ## t)))

#define RC_ALLOC_WITH_TRAILING(container_type, trailing_type, trailing_field, trailing_count, parse) ((container_type*)rc_alloc(\
          (parse)->buffer, &(parse)->offset, \
          RC_OFFSETOF((*(container_type*)NULL),trailing_field) + trailing_count * sizeof(trailing_type), \
          RC_ALIGNOF(container_type), &(parse)->scratch, 0))
#define RC_GET_TRAILING(container_pointer, container_type, trailing_type, trailing_field) (trailing_type*)(&((container_type*)(container_pointer))->trailing_field)

/* force alignment to 4 bytes on 32-bit systems, or 8 bytes on 64-bit systems */
#define RC_ALIGN(n) (((n) + (sizeof(void*)-1)) & ~(sizeof(void*)-1))

typedef struct {
  rc_buffer_t buffer;
  rc_scratch_string_t* strings;

  struct objs {
    rc_condition_t* __rc_condition_t;
    rc_condset_t* __rc_condset_t;
    rc_modified_memref_t* __rc_modified_memref_t;
    rc_lboard_t* __rc_lboard_t;
    rc_lboard_with_memrefs_t* __rc_lboard_with_memrefs_t;
    rc_memref_t* __rc_memref_t;
    rc_memref_list_t* __rc_memref_list_t;
    rc_memrefs_t* __rc_memrefs_t;
    rc_modified_memref_list_t* __rc_modified_memref_list_t;
    rc_operand_t* __rc_operand_t;
    rc_richpresence_t* __rc_richpresence_t;
    rc_richpresence_display_t* __rc_richpresence_display_t;
    rc_richpresence_display_part_t* __rc_richpresence_display_part_t;
    rc_richpresence_lookup_t* __rc_richpresence_lookup_t;
    rc_richpresence_lookup_item_t* __rc_richpresence_lookup_item_t;
    rc_richpresence_with_memrefs_t* __rc_richpresence_with_memrefs_t;
    rc_scratch_string_t __rc_scratch_string_t;
    rc_trigger_t* __rc_trigger_t;
    rc_trigger_with_memrefs_t* __rc_trigger_with_memrefs_t;
    rc_value_t* __rc_value_t;
    rc_value_with_memrefs_t* __rc_value_with_memrefs_t;

    /* these fields aren't actually used by the code, but they force the
     * virtual enum wrapper types to exist so natvis can use them */
    union {
      __rc_bool_enum_t boolean;
      __rc_memsize_enum_t memsize;
      __rc_memsize_enum_func_t memsize_func;
      __rc_operand_enum_t operand;
      __rc_value_type_enum_t value_type;
      __rc_memref_type_enum_t memref_type;
      __rc_condition_enum_t condition;
      __rc_condition_enum_str_t condition_str;
      __rc_condset_list_t condset_list;
      __rc_operator_enum_t oper;
      __rc_operator_enum_str_t oper_str;
      __rc_operand_memref_t operand_memref;
      __rc_value_list_t value_list;
      __rc_trigger_state_enum_t trigger_state;
      __rc_lboard_state_enum_t lboard_state;
      __rc_richpresence_display_list_t richpresence_display_list;
      __rc_richpresence_display_part_list_t richpresence_display_part_list;
      __rc_richpresence_lookup_list_t richpresence_lookup_list;
      __rc_format_enum_t format;
    } natvis_extension;
  } objs;
}
rc_scratch_t;

enum {
  RC_VALUE_TYPE_NONE,
  RC_VALUE_TYPE_UNSIGNED,
  RC_VALUE_TYPE_SIGNED,
  RC_VALUE_TYPE_FLOAT
};

typedef struct {
  union {
    uint32_t u32;
    int32_t i32;
    float f32;
  } value;

  char type;
}
rc_typed_value_t;

enum {
  RC_MEMREF_TYPE_MEMREF,                 /* rc_memref_t */
  RC_MEMREF_TYPE_MODIFIED_MEMREF,        /* rc_modified_memref_t */
  RC_MEMREF_TYPE_VALUE                   /* rc_value_t */
};

#define RC_MEASURED_UNKNOWN 0xFFFFFFFF
#define RC_OPERAND_NONE 0xFF

typedef struct {
  /* memory accessors */
  rc_peek_t peek;
  void* peek_userdata;

  /* processing state */
  rc_typed_value_t measured_value;     /* captured Measured value */
  int32_t add_hits;                    /* AddHits/SubHits accumulator */
  uint8_t is_true;                     /* true if all conditions are true */
  uint8_t is_primed;                   /* true if all non-Trigger conditions are true */
  uint8_t is_paused;                   /* true if one or more PauseIf conditions is true */
  uint8_t can_measure;                 /* false if the measured value should be ignored */
  uint8_t measured_from_hits;          /* true if the measured_value came from a condition's hit count */
  uint8_t and_next;                    /* true if the previous condition was AndNext true */
  uint8_t or_next;                     /* true if the previous condition was OrNext true */
  uint8_t reset_next;                  /* true if the previous condition was ResetNextIf true */
  uint8_t stop_processing;             /* true to abort the processing loop */

  /* result state */
  uint8_t has_hits;                    /* true if one of more hit counts is non-zero */
  uint8_t was_reset;                   /* true if one or more ResetIf conditions is true */
  uint8_t was_cond_reset;              /* true if one or more ResetNextIf conditions is true */

  /* control settings */
  uint8_t can_short_curcuit;           /* allows logic processing to stop as soon as a false condition is encountered */
}
rc_eval_state_t;

typedef struct {
  int32_t offset;

  void* buffer;
  rc_scratch_t scratch;

  rc_memrefs_t* memrefs;
  rc_memrefs_t* existing_memrefs;
  rc_value_t** variables;

  uint32_t measured_target;
  int lines_read;

  rc_operand_t addsource_parent;
  rc_operand_t indirect_parent;
  rc_operand_t remember;
  uint8_t addsource_oper;

  uint8_t is_value;
  uint8_t has_required_hits;
  uint8_t measured_as_percent;
  uint8_t ignore_non_parse_errors;
}
rc_parse_state_t;

typedef struct rc_preparse_state_t {
  rc_parse_state_t parse;
  rc_memrefs_t memrefs;
} rc_preparse_state_t;

void rc_init_parse_state(rc_parse_state_t* parse, void* buffer);
void rc_init_parse_state_memrefs(rc_parse_state_t* parse, rc_memrefs_t* memrefs);
void rc_reset_parse_state(rc_parse_state_t* parse, void* buffer);
void rc_destroy_parse_state(rc_parse_state_t* parse);
void rc_init_preparse_state(rc_preparse_state_t* preparse);
void rc_preparse_alloc_memrefs(rc_memrefs_t* memrefs, rc_preparse_state_t* preparse);
void rc_preparse_reserve_memrefs(rc_preparse_state_t* preparse, rc_memrefs_t* memrefs);
void rc_preparse_copy_memrefs(rc_parse_state_t* parse, rc_memrefs_t* memrefs);
void rc_destroy_preparse_state(rc_preparse_state_t *preparse);

void* rc_alloc(void* pointer, int32_t* offset, uint32_t size, uint32_t alignment, rc_scratch_t* scratch, uint32_t scratch_object_pointer_offset);
void* rc_alloc_scratch(void* pointer, int32_t* offset, uint32_t size, uint32_t alignment, rc_scratch_t* scratch, uint32_t scratch_object_pointer_offset);
char* rc_alloc_str(rc_parse_state_t* parse, const char* text, size_t length);

rc_memref_t* rc_alloc_memref(rc_parse_state_t* parse, uint32_t address, uint8_t size);
rc_modified_memref_t* rc_alloc_modified_memref(rc_parse_state_t* parse, uint8_t size, const rc_operand_t* parent,
                                               uint8_t modifier_type, const rc_operand_t* modifier);
int rc_parse_memref(const char** memaddr, uint8_t* size, uint32_t* address);
void rc_update_memref_values(rc_memrefs_t* memrefs, rc_peek_t peek, void* ud);
void rc_update_memref_value(rc_memref_value_t* memref, uint32_t value);
void rc_get_memref_value(rc_typed_value_t* value, rc_memref_t* memref, int operand_type);
uint32_t rc_get_modified_memref_value(const rc_modified_memref_t* memref, rc_peek_t peek, void* ud);
uint8_t rc_memref_shared_size(uint8_t size);
uint32_t rc_memref_mask(uint8_t size);
void rc_transform_memref_value(rc_typed_value_t* value, uint8_t size);
uint32_t rc_transform_operand_value(uint32_t value, const rc_operand_t* self);
uint32_t rc_peek_value(uint32_t address, uint8_t size, rc_peek_t peek, void* ud);

void rc_memrefs_init(rc_memrefs_t* memrefs);
void rc_memrefs_destroy(rc_memrefs_t* memrefs);
uint32_t rc_memrefs_count_memrefs(const rc_memrefs_t* memrefs);
uint32_t rc_memrefs_count_modified_memrefs(const rc_memrefs_t* memrefs);
uint32_t rc_memrefs_get_addresses(const rc_memrefs_t* memrefs,
                                  uint32_t* out_addresses,
                                  uint8_t* out_sizes,
                                  uint32_t out_capacity,
                                  rc_peek_t peek,
                                  void* peek_ud);

/* Callback: returns nonzero if all num_bytes bytes starting at address are
 * already present in the caller's cache. See rc_memrefs_get_pending_addresses. */
typedef int (RC_CCONV* rc_is_cached_t)(uint32_t address, uint32_t num_bytes, void* ud);

/* Like rc_memrefs_get_addresses, but resolves AddAddress (indirect) pointer
 * chains LEVEL BY LEVEL using the caller's cache: each chain is walked from its
 * static root through peek, and the FIRST address along the path that is not yet
 * cached (is_cached==0) is reported. Walking stops there (no cascade into garbage
 * past an unresolved pointer). Call in a loop, committing reported addresses to
 * the cache each round, until it returns 0 — resolves a whole multi-level chain
 * within one frame. READ-ONLY: never writes any memref value/prior/address. */
uint32_t rc_memrefs_get_pending_addresses(const rc_memrefs_t* memrefs,
                                          uint32_t* out_addresses,
                                          uint8_t* out_sizes,
                                          uint32_t out_capacity,
                                          rc_peek_t peek,
                                          rc_is_cached_t is_cached,
                                          void* ud);

/* Phase C step-0 census v2 (memref.c): classify every INDIRECT_READ chain by
 * whether the d2x-side chain walker can resolve it. Walker node shape:
 * INDIRECT = read(mask(parent)+modifier); combine = ALU_op(mask(parent),
 * modifier) for MULT..SUB_ACCUMULATOR. Parents = current-value memrefs
 * (transitive); modifier = int const (strict) or current-value memref
 * (grade 2, eligible_mref_mod). Read-only, one-shot at game load. Ineligible
 * chains stay on legacy ADDR_QUERY. NOTE: declared in BOTH rc_internal.h. */
typedef struct rc_phasec_census_t {
  uint32_t mm_total;             /* all modified memrefs */
  uint32_t mm_indirect;          /* INDIRECT_READ chains (leaf reads) */
  uint32_t eligible;             /* strict: const-only walk */
  uint32_t eligible_mref_mod;    /* needs memref-modifier slots in the descriptor */
  uint32_t inel_parent_dp;       /* parent operand is DELTA/PRIOR */
  uint32_t inel_parent_xform;    /* parent operand is BCD/INVERTED */
  uint32_t inel_parent_nonmem;   /* parent operand not a memref (const/func/recall) */
  uint32_t inel_parent_chain;    /* null memref / depth guard tripped */
  uint32_t inel_modifier;        /* modifier not const nor current-value memref */
  uint32_t inel_op;              /* combine operator outside MULT..SUB_ACCUMULATOR */
  uint32_t op_hist[18];          /* combine RC_OPERATOR_* walked in eligible chains */
  uint32_t nonmem_hist[10];      /* RC_OPERAND_* of the non-memref parents (idx 9 = other) */
  uint32_t depth_hist[8];        /* eligible chains by mm->depth (clamped; levels = depth+1) */
  uint32_t leaf_bytes_hist[5];   /* eligible chains by leaf byte width (1/2/3/4+) */
  uint32_t parent_size_hist[32]; /* parent operand RC_MEMSIZE_* per level walked (eligible) */
  uint32_t eligible_value_bytes; /* wire cost: sum of leaf bytes across eligible chains */
  uint32_t recall_hops;          /* v3: RECALL edges followed in eligible chains (static
                                  * refs to the remembered expression's memref) */
  uint32_t const_base;           /* v3: const parent operands (fixed base) in eligible chains */
} rc_phasec_census_t;
void rc_memrefs_phasec_census(const rc_memrefs_t* memrefs, rc_phasec_census_t* out);

/* Phase C node table (emitter in memref.c). One 8-byte node per distinct
 * walker step, dependency-ordered (parents before children), DAG-dedup'd by
 * memref pointer. Wire format for the d2x CHAIN_TABLE.
 *   op byte: bits 0-4 = RC_OPERATOR_* (INDIRECT_READ = deref [read
 *            mask(parent)+operand]; MULT..SUB_ACCUMULATOR = integer ALU
 *            combine [SUB_PARENT = operand-parent, ACCs = add/sub, mirroring
 *            rc_resolve_cached_raw]; NONE = integer immediate, operand =
 *            value, no parent), bits 5-6 = deref read width - 1 (raw bytes,
 *            LE-packed exactly like the adapter peek / memory_data), bit 7 =
 *            shipped (leaf bytes present in the per-frame value blob, slot
 *            order; blob offsets derivable by scanning shipped widths).
 *   psize:   RC_MEMSIZE_* transform applied to the PARENT's value before
 *            op/operand (only full-byte sizes 8/16/24/32 + 16/24/32 BE are
 *            emitted — sub-byte/float edges are refused to keep the ARM
 *            walker's transform surface exact vs rc_peek_value masking).
 *   root deref: parent == RC_PHASEC_PARENT_NONE, operand = absolute address
 *            (not shipped — roots stay in the flat watchlist). */
#define RC_PHASEC_PARENT_NONE 0xFFFF
/* Phase D: dp-kind in the psize HIGH bits (must match gc_ra_protocol.h
 * RA_CN_PSZ_DP_SHIFT / RA_CN_DP_*): the edge reads the parent's CURRENT (0),
 * PREVIOUS-frame (1 = rcheevos DELTA) or last-DIFFERENT (2 = PRIOR) value.
 * RC_PHASEC_MAX_DP must match RA_MAX_DP_PARENTS. */
#define RC_PHASEC_PSZ_DP_SHIFT 6
#define RC_PHASEC_DP_DELTA     1
#define RC_PHASEC_DP_PRIOR     2
#define RC_PHASEC_MAX_DP       128
#define RC_PHASEC_OP(n)       ((uint8_t)((n)->op & 0x1F))
#define RC_PHASEC_WIDTH(n)    ((uint8_t)((((n)->op >> 5) & 3) + 1))
#define RC_PHASEC_SHIPPED(n)  ((uint8_t)((n)->op >> 7))
typedef struct rc_phasec_node_t {
  uint32_t operand;   /* const offset/mask/addend/immediate, or root address */
  uint16_t parent;    /* parent slot, or RC_PHASEC_PARENT_NONE */
  uint8_t  op;        /* see above */
  uint8_t  psize;     /* RC_MEMSIZE_* edge transform on the parent value */
} rc_phasec_node_t;
/* Emit the table for every eligible INDIRECT chain. keys = caller scratch
 * (cap entries) for the dedup map, only needed during the call. Returns node
 * count, or 0xFFFFFFFF on cap overflow. out_blob_bytes = per-frame value blob
 * size; out_shipped = shipped deref count; out_refused = census-eligible
 * chains the emitter still refused (stricter edge-size/immediate rules). */
uint32_t rc_memrefs_phasec_emit(const rc_memrefs_t* memrefs,
                                rc_phasec_node_t* nodes, const void** keys,
                                uint32_t cap, uint32_t* out_blob_bytes,
                                uint32_t* out_shipped, uint32_t* out_refused);

/* Phase C authoritative hooks (memref.c; adapter-registered, NULL = legacy).
 * mm_index = dense modified_memref position in list order (== the U1 cursor
 * ordering == the counter in rc_memrefs_get_pending_addresses).
 *   g_rc_phasec_read: 0 = not covered, 1 = *raw_out is the LE-packed leaf
 *     bytes (apply rc_memref_mask), 2 = slot not ready (gate bumped; keep
 *     stale value, frame defers). Used by rc_upd_resolve_one (rc_client.c).
 *   g_rc_phasec_covered: collect walk skips covered chains entirely.
 * NOTE: declared in BOTH rc_internal.h copies. */
extern int (*g_rc_phasec_read)(uint32_t mm_index, uint32_t* raw_out);
extern int (*g_rc_phasec_covered)(uint32_t mm_index);

void rc_parse_trigger_internal(rc_trigger_t* self, const char** memaddr, rc_parse_state_t* parse);
int rc_trigger_state_active(int state);
rc_memrefs_t* rc_trigger_get_memrefs(rc_trigger_t* self);

typedef struct rc_condset_with_trailing_conditions_t {
  rc_condset_t condset;
  rc_condition_t conditions[2];
} rc_condset_with_trailing_conditions_t;
RC_ALLOW_ALIGN(rc_condset_with_trailing_conditions_t)

rc_condset_t* rc_parse_condset(const char** memaddr, rc_parse_state_t* parse);
int rc_test_condset(rc_condset_t* self, rc_eval_state_t* eval_state);
void rc_reset_condset(rc_condset_t* self);
rc_condition_t* rc_condset_get_conditions(rc_condset_t* self);
#ifdef RC_EVAL_PLAN
/* Compact HOT per-condition eval record (see project_compiled_eval). 16B, 4 per cache
 * line, parallel to the condset's trailing conditions[]. The eval streams this in the
 * common (optimized_comparator) path instead of the 32B rc_condition_t; the COLD fields
 * (operand metadata, required_hits, next) stay in rc_condition_t, read only for the rare
 * paths (hit-target / non-optimized operand / reset / serialization). op1/op2 hold the
 * operand union as u32 (the memref POINTER — stable, built once — or const num / f32
 * bits; the comparator says which). current_hits is the only rw field. */
typedef struct rc_eval_hot_t {
  uint32_t op1;            /* operand1 union as u32 (memref ptr or const) */
  uint32_t op2;            /* operand2 union */
  uint32_t current_hits;   /* rw accrual (canonical for eval; synced to rc_condition_t for cold readers) */
  uint8_t  type;           /* RC_CONDITION_* (evaluator dispatch) */
  uint8_t  oper;           /* RC_OPERATOR_* */
  uint8_t  cmp;            /* optimized_comparator (RC_PROCESSING_COMPARE_*) */
  uint8_t  flags;          /* bit0=is_true(truth) bit1=is_true(reset-responsible) bit2=has_target */
} rc_eval_hot_t;
/* build the HOT array for a condset from its conditions[] (idempotent; sets self->hot). */
void rc_build_eval_hot(rc_condset_t* self);
/* optional device override for the hot-array allocation (NULL => libc malloc). Set it to
 * a PSRAM allocator on platforms where the hot arrays must avoid internal RAM. */
extern void* (*g_rc_eval_hot_alloc)(size_t);
/* Plumb the parallel hot record through the eval call-chain. Under RC_EVAL_PLAN the
 * functions carry an extra trailing `rc_eval_hot_t* hot` arg; without it they expand to
 * nothing so the stock signatures (and the off-build) are unchanged. RC_HOTP = the param
 * in a declaration/definition; RC_HOTA(x) = the matching trailing arg at a call site. */
#define RC_HOTP , rc_eval_hot_t* hot
#define RC_HOTA(x) , (x)
/* Canonical current_hits lvalue: the eval keeps current_hits in the hot record when it
 * exists (built once, the only rw field), else in the cold rc_condition_t. All eval-path
 * accesses go through this so the read and write side never diverge. */
#define RC_CUR_HITS(cond, hot) (*((hot) ? &(hot)->current_hits : &(cond)->current_hits))
/* "does this condition have a hit target?" from the hot flag (bit2), avoiding the cold
 * rc_condition_t read on the common no-target tally path. The required_hits VALUE itself
 * (only needed when this is true) is still read from the cold struct. */
#define RC_HAS_TARGET(cond, hot) ((hot) ? ((hot)->flags & 0x04) : ((cond)->required_hits != 0))
#else
#define RC_HOTP
#define RC_HOTA(x)
#define RC_CUR_HITS(cond, hot) ((cond)->current_hits)
#define RC_HAS_TARGET(cond, hot) ((cond)->required_hits != 0)
#endif
void rc_test_condset_internal(rc_condition_t* condition, uint32_t num_conditions, rc_eval_state_t* eval_state, int can_short_circuit RC_HOTP);

enum {
  RC_PROCESSING_COMPARE_DEFAULT = 0,
  RC_PROCESSING_COMPARE_MEMREF_TO_CONST,
  RC_PROCESSING_COMPARE_MEMREF_TO_DELTA,
  RC_PROCESSING_COMPARE_MEMREF_TO_MEMREF,
  RC_PROCESSING_COMPARE_DELTA_TO_MEMREF,
  RC_PROCESSING_COMPARE_DELTA_TO_CONST,
  RC_PROCESSING_COMPARE_MEMREF_TO_CONST_TRANSFORMED,
  RC_PROCESSING_COMPARE_MEMREF_TO_DELTA_TRANSFORMED,
  RC_PROCESSING_COMPARE_MEMREF_TO_MEMREF_TRANSFORMED,
  RC_PROCESSING_COMPARE_DELTA_TO_MEMREF_TRANSFORMED,
  RC_PROCESSING_COMPARE_DELTA_TO_CONST_TRANSFORMED,
  RC_PROCESSING_COMPARE_ALWAYS_TRUE,
  RC_PROCESSING_COMPARE_ALWAYS_FALSE
};

rc_condition_t* rc_parse_condition(const char** memaddr, rc_parse_state_t* parse);
void rc_parse_condition_internal(rc_condition_t* self, const char** memaddr, rc_parse_state_t* parse);
void rc_condition_update_parse_state(rc_condition_t* condition, rc_parse_state_t* parse);
int rc_test_condition(rc_condition_t* self, rc_eval_state_t* eval_state RC_HOTP);
void rc_evaluate_condition_value(rc_typed_value_t* value, rc_condition_t* self, rc_eval_state_t* eval_state);
int rc_condition_is_combining(const rc_condition_t* self);
void rc_condition_convert_to_operand(const rc_condition_t* condition, rc_operand_t* operand, rc_parse_state_t* parse);
const rc_operand_t* rc_condition_get_real_operand1(const rc_condition_t* self);

int rc_parse_operand(rc_operand_t* self, const char** memaddr, rc_parse_state_t* parse);
void rc_evaluate_operand(rc_typed_value_t* value, const rc_operand_t* self, rc_eval_state_t* eval_state);
int rc_operator_is_modifying(int oper);
int rc_memsize_is_float(uint8_t size);
int rc_operand_is_float_memref(const rc_operand_t* self);
int rc_operand_is_float(const rc_operand_t* self);
int rc_operand_is_recall(const rc_operand_t* self);
int rc_operand_type_is_memref(uint8_t type);
int rc_operand_type_is_transform(uint8_t type);
int rc_operands_are_equal(const rc_operand_t* left, const rc_operand_t* right);
void rc_operand_addsource(rc_operand_t* self, rc_parse_state_t* parse, uint8_t new_size);
void rc_operand_set_const(rc_operand_t* self, uint32_t value);
void rc_operand_set_float_const(rc_operand_t* self, double value);

int rc_is_valid_variable_character(char ch, int is_first);
void rc_parse_value_internal(rc_value_t* self, const char** memaddr, rc_parse_state_t* parse);
int rc_evaluate_value_typed(rc_value_t* self, rc_typed_value_t* value, rc_peek_t peek, void* ud);
void rc_reset_value(rc_value_t* self);
int rc_value_from_hits(rc_value_t* self);
rc_value_t* rc_alloc_variable(const char* memaddr, size_t memaddr_len, rc_parse_state_t* parse);
uint32_t rc_count_values(const rc_value_t* values);
void rc_update_values(rc_value_t* values, rc_peek_t peek, void* ud);
void rc_reset_values(rc_value_t* values);

void rc_typed_value_convert(rc_typed_value_t* value, char new_type);
void rc_typed_value_add(rc_typed_value_t* value, const rc_typed_value_t* amount);
void rc_typed_value_multiply(rc_typed_value_t* value, const rc_typed_value_t* amount);
void rc_typed_value_divide(rc_typed_value_t* value, const rc_typed_value_t* amount);
void rc_typed_value_modulus(rc_typed_value_t* value, const rc_typed_value_t* amount);
void rc_typed_value_negate(rc_typed_value_t* value);
int rc_typed_value_compare(const rc_typed_value_t* value1, const rc_typed_value_t* value2, char oper);
void rc_typed_value_combine(rc_typed_value_t* value, rc_typed_value_t* amount, uint8_t oper);
void rc_typed_value_from_memref_value(rc_typed_value_t* value, const rc_memref_value_t* memref);

int rc_format_typed_value(char* buffer, size_t size, const rc_typed_value_t* value, int format);

void rc_parse_lboard_internal(rc_lboard_t* self, const char* memaddr, rc_parse_state_t* parse);
int rc_lboard_state_active(int state);

void rc_parse_richpresence_internal(rc_richpresence_t* self, const char* script, rc_parse_state_t* parse);
rc_memrefs_t* rc_richpresence_get_memrefs(rc_richpresence_t* self);
void rc_reset_richpresence_triggers(rc_richpresence_t* self);
void rc_update_richpresence_internal(rc_richpresence_t* richpresence, rc_peek_t peek, void* peek_ud);

int rc_validate_memrefs_for_console(const rc_memrefs_t* memrefs, char result[], const size_t result_size, uint32_t console_id);

RC_END_C_DECLS

#endif /* RC_INTERNAL_H */
