#include "rc_internal.h"

#include <string.h> /* memcpy */
#ifdef RC_EVAL_PLAN
#include <stdlib.h> /* malloc (host foundation; device port uses ps_malloc/buffer) */
#endif

enum {
  RC_CONDITION_CLASSIFICATION_COMBINING,
  RC_CONDITION_CLASSIFICATION_PAUSE,
  RC_CONDITION_CLASSIFICATION_RESET,
  RC_CONDITION_CLASSIFICATION_HITTARGET,
  RC_CONDITION_CLASSIFICATION_MEASURED,
  RC_CONDITION_CLASSIFICATION_OTHER,
  RC_CONDITION_CLASSIFICATION_INDIRECT
};

static int rc_classify_condition(const rc_condition_t* cond) {
  switch (cond->type) {
    case RC_CONDITION_PAUSE_IF:
      return RC_CONDITION_CLASSIFICATION_PAUSE;

    case RC_CONDITION_RESET_IF:
      return RC_CONDITION_CLASSIFICATION_RESET;

    case RC_CONDITION_ADD_ADDRESS:
    case RC_CONDITION_ADD_SOURCE:
    case RC_CONDITION_SUB_SOURCE:
      /* these are handled by rc_modified_memref_t */
      return RC_CONDITION_CLASSIFICATION_INDIRECT;

    case RC_CONDITION_ADD_HITS:
    case RC_CONDITION_AND_NEXT:
    case RC_CONDITION_OR_NEXT:
    case RC_CONDITION_REMEMBER:
    case RC_CONDITION_RESET_NEXT_IF:
    case RC_CONDITION_SUB_HITS:
      return RC_CONDITION_CLASSIFICATION_COMBINING;

    case RC_CONDITION_MEASURED:
    case RC_CONDITION_MEASURED_IF:
      /* even if not measuring a hit target, we still want to evaluate it every frame */
      return RC_CONDITION_CLASSIFICATION_MEASURED;

    default:
      if (cond->required_hits != 0)
        return RC_CONDITION_CLASSIFICATION_HITTARGET;

      return RC_CONDITION_CLASSIFICATION_OTHER;
  }
}

static int32_t rc_classify_conditions(rc_condset_t* self, const char* memaddr, const rc_parse_state_t* parent_parse) {
  rc_parse_state_t parse;
  rc_memrefs_t memrefs;
  rc_condition_t condition;
  int classification;
  uint32_t index = 0;
  uint32_t chain_length = 1;

  rc_init_parse_state(&parse, NULL);
  rc_init_parse_state_memrefs(&parse, &memrefs);
  parse.ignore_non_parse_errors = parent_parse->ignore_non_parse_errors;

  do {
    rc_parse_condition_internal(&condition, &memaddr, &parse);

    if (parse.offset < 0) {
      rc_destroy_parse_state(&parse);
      return parse.offset;
    }

    ++index;

    classification = rc_classify_condition(&condition);
    switch (classification) {
      case RC_CONDITION_CLASSIFICATION_COMBINING:
        ++chain_length;
        continue;

      case RC_CONDITION_CLASSIFICATION_INDIRECT:
        ++self->num_indirect_conditions;
        continue;

      case RC_CONDITION_CLASSIFICATION_PAUSE:
        self->num_pause_conditions += chain_length;
        break;

      case RC_CONDITION_CLASSIFICATION_RESET:
        self->num_reset_conditions += chain_length;
        break;

      case RC_CONDITION_CLASSIFICATION_HITTARGET:
        self->num_hittarget_conditions += chain_length;
        break;

      case RC_CONDITION_CLASSIFICATION_MEASURED:
        self->num_measured_conditions += chain_length;
        break;

      default:
        self->num_other_conditions += chain_length;
        break;
    }

    chain_length = 1;
  } while (*memaddr++ == '_');

  /* any combining conditions that don't actually feed into a non-combining condition
   * need to still have space allocated for them. put them in "other" to match the
   * logic in rc_find_next_classification */
  self->num_other_conditions += chain_length - 1;

  rc_destroy_parse_state(&parse);

  return (int32_t)index;
}

static int rc_find_next_classification(const char* memaddr) {
  rc_parse_state_t parse;
  rc_memrefs_t memrefs;
  rc_condition_t condition;
  int classification;

  rc_init_parse_state(&parse, NULL);
  rc_init_parse_state_memrefs(&parse, &memrefs);

  do {
    rc_parse_condition_internal(&condition, &memaddr, &parse);
    if (parse.offset < 0)
      break;

    classification = rc_classify_condition(&condition);
    switch (classification) {
      case RC_CONDITION_CLASSIFICATION_COMBINING:
      case RC_CONDITION_CLASSIFICATION_INDIRECT:
        break;

      default:
        rc_destroy_parse_state(&parse);
        return classification;
    }
  } while (*memaddr++ == '_');

  rc_destroy_parse_state(&parse);
  return RC_CONDITION_CLASSIFICATION_OTHER;
}

static void rc_condition_update_recall_operand(rc_operand_t* operand, const rc_operand_t* remember)
{
  if (operand->type == RC_OPERAND_RECALL) {
    if (rc_operand_type_is_memref(operand->memref_access_type) && operand->value.memref == NULL) {
      memcpy(operand, remember, sizeof(*remember));
      operand->memref_access_type = operand->type;
      operand->type = RC_OPERAND_RECALL;
    }
  }
  else if (rc_operand_is_memref(operand) && operand->value.memref->value.memref_type == RC_MEMREF_TYPE_MODIFIED_MEMREF) {
    rc_modified_memref_t* modified_memref = (rc_modified_memref_t*)operand->value.memref;
    rc_condition_update_recall_operand(&modified_memref->parent, remember);
    rc_condition_update_recall_operand(&modified_memref->modifier, remember);
  }
}

static void rc_update_condition_pause_remember(rc_condset_t* self) {
  rc_operand_t* pause_remember = NULL;
  rc_condition_t* condition;
  rc_condition_t* pause_conditions;
  const rc_condition_t* end_pause_condition;

  /* ASSERT: pause conditions are first conditions */
  pause_conditions = rc_condset_get_conditions(self);
  end_pause_condition = pause_conditions + self->num_pause_conditions;

  for (condition = pause_conditions; condition < end_pause_condition; ++condition) {
    if (condition->type == RC_CONDITION_REMEMBER) {
      pause_remember = &condition->operand1;
    }
    else if (pause_remember == NULL) {
      /* if we picked up a non-pause remember, discard it */
      if (condition->operand1.type == RC_OPERAND_RECALL &&
          rc_operand_type_is_memref(condition->operand1.memref_access_type)) {
        condition->operand1.value.memref = NULL;
      }

      if (condition->operand2.type == RC_OPERAND_RECALL &&
          rc_operand_type_is_memref(condition->operand2.memref_access_type)) {
        condition->operand2.value.memref = NULL;
      }
    }
  }

  if (pause_remember) {
    for (condition = self->conditions; condition; condition = condition->next) {
      if (condition >= end_pause_condition) {
        /* if we didn't find a remember for a non-pause condition, use the last pause remember */
        rc_condition_update_recall_operand(&condition->operand1, pause_remember);
        rc_condition_update_recall_operand(&condition->operand2, pause_remember);
      }

      /* Anything after this point will have already been handled */
      if (condition->type == RC_CONDITION_REMEMBER)
        break;
    }
  }
}

rc_condset_t* rc_parse_condset(const char** memaddr, rc_parse_state_t* parse) {
  rc_condset_with_trailing_conditions_t* condset_with_conditions;
  rc_condset_t local_condset;
  rc_condset_t* self;
  rc_condition_t condition;
  rc_condition_t* conditions;
  rc_condition_t** next;
  rc_condition_t* pause_conditions = NULL;
  rc_condition_t* reset_conditions = NULL;
  rc_condition_t* hittarget_conditions = NULL;
  rc_condition_t* measured_conditions = NULL;
  rc_condition_t* other_conditions = NULL;
  rc_condition_t* indirect_conditions = NULL;
  int classification, combining_classification = RC_CONDITION_CLASSIFICATION_COMBINING;
  uint32_t measured_target = 0;
  int32_t result;

  if (**memaddr == 'S' || **memaddr == 's' || !**memaddr) {
    /* empty group - editor allows it, so we have to support it */
    self = RC_ALLOC(rc_condset_t, parse);
    memset(self, 0, sizeof(*self));
    return self;
  }

  memset(&local_condset, 0, sizeof(local_condset));
  result = rc_classify_conditions(&local_condset, *memaddr, parse);
  if (result < 0) {
    parse->offset = result;
    return NULL;
  }

  condset_with_conditions = RC_ALLOC_WITH_TRAILING(rc_condset_with_trailing_conditions_t,
                                                   rc_condition_t, conditions, result, parse);
  if (parse->offset < 0)
    return NULL;

  self = (rc_condset_t*)condset_with_conditions;
  memcpy(self, &local_condset, sizeof(local_condset));
  conditions = &condset_with_conditions->conditions[0];

  if (parse->buffer) {
    pause_conditions = conditions;
    conditions += self->num_pause_conditions;

    reset_conditions = conditions;
    conditions += self->num_reset_conditions;

    hittarget_conditions = conditions;
    conditions += self->num_hittarget_conditions;

    measured_conditions = conditions;
    conditions += self->num_measured_conditions;

    other_conditions = conditions;
    conditions += self->num_other_conditions;

    indirect_conditions = conditions;
  }

  next = &self->conditions;

  /* prevent bleedthrough of incomplete conditions from other groups */
  parse->addsource_oper = RC_OPERATOR_NONE;
  parse->addsource_parent.type = RC_OPERAND_NONE;
  parse->indirect_parent.type = RC_OPERAND_NONE;

  /* each condition set has a functionally new recall accumulator */
  parse->remember.type = RC_OPERAND_NONE;

  for (;;) {
    rc_parse_condition_internal(&condition, memaddr, parse);

    if (parse->offset < 0)
      return NULL;

    if (condition.oper == RC_OPERATOR_NONE && !parse->ignore_non_parse_errors) {
      switch (condition.type) {
        case RC_CONDITION_ADD_ADDRESS:
        case RC_CONDITION_ADD_SOURCE:
        case RC_CONDITION_SUB_SOURCE:
        case RC_CONDITION_REMEMBER:
          /* these conditions don't require a right hand side (implied *1) */
          break;

        case RC_CONDITION_MEASURED:
          /* right hand side is not required when Measured is used in a value */
          if (parse->is_value)
            break;
          /* fallthrough */ /* to default */

        default:
          parse->offset = RC_INVALID_OPERATOR;
          return NULL;
      }
    }

    switch (condition.type) {
      case RC_CONDITION_MEASURED:
        if (measured_target != 0) {
          /* multiple Measured flags cannot exist in the same group */
          if (!parse->ignore_non_parse_errors) {
            parse->offset = RC_MULTIPLE_MEASURED;
            return NULL;
          }
        }
        else if (parse->is_value) {
          measured_target = (uint32_t)-1;
          if (!rc_operator_is_modifying(condition.oper)) {
            /* measuring comparison in a value results in a tally (hit count). set target to MAX_INT */
            condition.required_hits = measured_target;
          }
        }
        else if (condition.required_hits != 0) {
          measured_target = condition.required_hits;
        }
        else if (condition.operand2.type == RC_OPERAND_CONST) {
          measured_target = condition.operand2.value.num;
        }
        else if (condition.operand2.type == RC_OPERAND_FP) {
          measured_target = (unsigned)condition.operand2.value.dbl;
        }
        else if (!parse->ignore_non_parse_errors) {
          parse->offset = RC_INVALID_MEASURED_TARGET;
          return NULL;
        }

        if (parse->measured_target && measured_target != parse->measured_target) {
          /* multiple Measured flags in separate groups must have the same target */
          if (!parse->ignore_non_parse_errors) {
            parse->offset = RC_MULTIPLE_MEASURED;
            return NULL;
          }
        }

        parse->measured_target = measured_target;
        break;

      case RC_CONDITION_STANDARD:
      case RC_CONDITION_TRIGGER:
        /* these flags are not allowed in value expressions */
        if (parse->is_value && !parse->ignore_non_parse_errors) {
          parse->offset = RC_INVALID_VALUE_FLAG;
          return NULL;
        }
        break;
    }

    rc_condition_update_parse_state(&condition, parse);

    if (parse->buffer) {
      classification = rc_classify_condition(&condition);
      if (classification == RC_CONDITION_CLASSIFICATION_COMBINING) {
        if (combining_classification == RC_CONDITION_CLASSIFICATION_COMBINING) {
          if (**memaddr == '_')
            combining_classification = rc_find_next_classification(&(*memaddr)[1]); /* skip over '_' */
          else
            combining_classification = RC_CONDITION_CLASSIFICATION_OTHER;
        }

        classification = combining_classification;
      }
      else {
        combining_classification = RC_CONDITION_CLASSIFICATION_COMBINING;
      }

      switch (classification) {
        case RC_CONDITION_CLASSIFICATION_PAUSE:
          memcpy(pause_conditions, &condition, sizeof(condition));
          *next = pause_conditions++;
          break;

        case RC_CONDITION_CLASSIFICATION_RESET:
          memcpy(reset_conditions, &condition, sizeof(condition));
          *next = reset_conditions++;
          break;

        case RC_CONDITION_CLASSIFICATION_HITTARGET:
          memcpy(hittarget_conditions, &condition, sizeof(condition));
          *next = hittarget_conditions++;
          break;

        case RC_CONDITION_CLASSIFICATION_MEASURED:
          memcpy(measured_conditions, &condition, sizeof(condition));
          *next = measured_conditions++;
          break;

        case RC_CONDITION_CLASSIFICATION_INDIRECT:
          memcpy(indirect_conditions, &condition, sizeof(condition));
          *next = indirect_conditions++;
          break;

        default:
          memcpy(other_conditions, &condition, sizeof(condition));
          *next = other_conditions++;
          break;
      }

      next = &(*next)->next;
    }

    if (**memaddr != '_')
      break;

    (*memaddr)++;
  }

  *next = NULL;

  self->has_pause = self->num_pause_conditions > 0;
  if (self->has_pause && parse->buffer && parse->remember.type != RC_OPERAND_NONE)
    rc_update_condition_pause_remember(self);

  return self;
}

static uint8_t rc_condset_evaluate_condition_no_add_hits(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  /* evaluate the current condition */
  uint8_t cond_valid = (uint8_t)rc_test_condition(condition, eval_state RC_HOTA(hot));
#ifdef RC_EVAL_PLAN
  /* store the raw truth in hot->flags bit0; clearing bit1 each frame mirrors the cold
   * `is_true = cond_valid` (which drops the ResetIf-responsible flag), bit2 (has_target)
   * is preserved. A later ResetIf may OR bit1 back in. */
  if (hot)
    hot->flags = (uint8_t)((hot->flags & ~0x03) | (cond_valid & 0x01));
  else
#endif
    condition->is_true = cond_valid;

  if (eval_state->reset_next) {
    /* previous ResetNextIf resets the hit count on this condition and prevents it from being true */
    eval_state->was_cond_reset |= (RC_CUR_HITS(condition, hot) != 0);

    RC_CUR_HITS(condition, hot) = 0;
    cond_valid = 0;
  }
  else {
    /* apply chained logic flags */
    cond_valid &= eval_state->and_next;
    cond_valid |= eval_state->or_next;

    if (cond_valid) {
      /* true conditions should update their hit count */
      eval_state->has_hits = 1;

      if (!RC_HAS_TARGET(condition, hot)) {
        /* no target hit count, just keep tallying */
        ++RC_CUR_HITS(condition, hot);
      }
      else if (RC_CUR_HITS(condition, hot) < condition->required_hits) {
        /* target hit count hasn't been met, tally and revalidate - only true if hit count becomes met */
        ++RC_CUR_HITS(condition, hot);
        cond_valid = (RC_CUR_HITS(condition, hot) == condition->required_hits);
      }
      else {
        /* target hit count has been met, do nothing */
      }
    }
    else if (RC_CUR_HITS(condition, hot) > 0) {
      /* target has been true in the past, if the hit target is met, consider it true now */
      eval_state->has_hits = 1;
      cond_valid = (RC_CUR_HITS(condition, hot) == condition->required_hits);
    }
  }

  /* reset chained logic flags for the next condition */
  eval_state->and_next = 1;
  eval_state->or_next = 0;

  return cond_valid;
}

static uint32_t rc_condset_evaluate_total_hits(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  uint32_t total_hits = RC_CUR_HITS(condition, hot);

  if (RC_HAS_TARGET(condition, hot)) {
    /* if the condition has a target hit count, we have to recalculate cond_valid including the AddHits counter */
    const int32_t signed_hits = (int32_t)RC_CUR_HITS(condition, hot) + eval_state->add_hits;
    total_hits = (signed_hits >= 0) ? (uint32_t)signed_hits : 0;
  }
  else {
    /* no target hit count. we can't tell if the add_hits value is from this frame or not, so ignore it.
       complex condition will only be true if the current condition is true */
  }

  eval_state->add_hits = 0;

  return total_hits;
}

static uint8_t rc_condset_evaluate_condition(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  uint8_t cond_valid = rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));

  if (eval_state->add_hits != 0 && RC_HAS_TARGET(condition, hot)) {
    uint32_t total_hits = rc_condset_evaluate_total_hits(condition, eval_state RC_HOTA(hot));
    cond_valid = (total_hits >= condition->required_hits);
  }

  /* reset logic flags for the next condition */
  eval_state->reset_next = 0;

  return cond_valid;
}

static void rc_condset_evaluate_standard(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  const uint8_t cond_valid = rc_condset_evaluate_condition(condition, eval_state RC_HOTA(hot));

  eval_state->is_true &= cond_valid;
  eval_state->is_primed &= cond_valid;

  if (!cond_valid && eval_state->can_short_curcuit)
    eval_state->stop_processing = 1;
}

static void rc_condset_evaluate_pause_if(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  const uint8_t cond_valid = rc_condset_evaluate_condition(condition, eval_state RC_HOTA(hot));

  if (cond_valid) {
    eval_state->is_paused = 1;

    /* set cannot be valid if it's paused */
    eval_state->is_true = eval_state->is_primed = 0;

    /* as soon as we find a PauseIf that evaluates to true, stop processing the rest of the group */
    eval_state->stop_processing = 1;
  }
  else if (!RC_HAS_TARGET(condition, hot)) {
    /* PauseIf didn't evaluate true, and doesn't have a HitCount, reset the HitCount to indicate the condition didn't match */
    RC_CUR_HITS(condition, hot) = 0;
  }
  else {
    /* PauseIf has a HitCount that hasn't been met, ignore it for now. */
  }
}

static void rc_condset_evaluate_reset_if(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  const uint8_t cond_valid = rc_condset_evaluate_condition(condition, eval_state RC_HOTA(hot));

  if (cond_valid) {
    /* flag the condition as being responsible for the reset */
    /* make sure not to modify bit0, as we use bitwise-and operators to combine truthiness */
#ifdef RC_EVAL_PLAN
    if (hot)
      hot->flags |= 0x02;
    else
#endif
      condition->is_true |= 0x02;

    /* set cannot be valid if we've hit a reset condition */
    eval_state->is_true = eval_state->is_primed = 0;

    /* let caller know to reset all hit counts */
    eval_state->was_reset = 1;

    /* can stop processing once an active ResetIf is encountered */
    eval_state->stop_processing = 1;
  }
}

static void rc_condset_evaluate_trigger(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  const uint8_t cond_valid = rc_condset_evaluate_condition(condition, eval_state RC_HOTA(hot));

  eval_state->is_true &= cond_valid;
}

static void rc_condset_evaluate_measured(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  if (!RC_HAS_TARGET(condition, hot)) {
    rc_condset_evaluate_standard(condition, eval_state RC_HOTA(hot));

    /* Measured condition without a hit target measures the value of the left operand */
    rc_evaluate_operand(&eval_state->measured_value, &condition->operand1, eval_state);
    eval_state->measured_from_hits = 0;
  }
  else {
    /* this largely mimicks rc_condset_evaluate_condition, but captures the total_hits */
    uint8_t cond_valid = rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));
    const uint32_t total_hits = rc_condset_evaluate_total_hits(condition, eval_state RC_HOTA(hot));

    cond_valid = (total_hits >= condition->required_hits);
    eval_state->is_true &= cond_valid;
    eval_state->is_primed &= cond_valid;

    /* if there is a hit target, capture the current hits */
    eval_state->measured_value.value.u32 = total_hits;
    eval_state->measured_value.type = RC_VALUE_TYPE_UNSIGNED;
    eval_state->measured_from_hits = 1;

    /* reset logic flags for the next condition */
    eval_state->reset_next = 0;
  }
}

static void rc_condset_evaluate_measured_if(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  const uint8_t cond_valid = rc_condset_evaluate_condition(condition, eval_state RC_HOTA(hot));

  eval_state->is_true &= cond_valid;
  eval_state->is_primed &= cond_valid;
  eval_state->can_measure &= cond_valid;
}

static void rc_condset_evaluate_add_hits(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));

  eval_state->add_hits += (int32_t)RC_CUR_HITS(condition, hot);

  /* ResetNextIf was applied to this AddHits condition; don't apply it to future conditions */
  eval_state->reset_next = 0;
}

static void rc_condset_evaluate_sub_hits(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));

  eval_state->add_hits -= (int32_t)RC_CUR_HITS(condition, hot);

  /* ResetNextIf was applied to this AddHits condition; don't apply it to future conditions */
  eval_state->reset_next = 0;
}

static void rc_condset_evaluate_reset_next_if(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  eval_state->reset_next = rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));
}

static void rc_condset_evaluate_and_next(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  eval_state->and_next = rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));
}

static void rc_condset_evaluate_or_next(rc_condition_t* condition, rc_eval_state_t* eval_state RC_HOTP) {
  eval_state->or_next = rc_condset_evaluate_condition_no_add_hits(condition, eval_state RC_HOTA(hot));
}

void rc_test_condset_internal(rc_condition_t* condition, uint32_t num_conditions,
                              rc_eval_state_t* eval_state, int can_short_circuit RC_HOTP) {
  const rc_condition_t* condition_end = condition + num_conditions;
  /* hot[] parallels the conditions[] slice this call was handed (advanced by category
   * in rc_test_condset); the parameter `hot` walks it in lockstep with condition. NULL
   * (alloc failed) stays NULL so the evaluators fall back to the cold rc_condition_t. */
  for (; condition < condition_end; ++condition
#ifdef RC_EVAL_PLAN
       , hot = (hot ? hot + 1 : hot)
#endif
       ) {
#ifdef RC_EVAL_PLAN
    switch (hot ? hot->type : condition->type) {
#else
    switch (condition->type) {
#endif
      case RC_CONDITION_STANDARD:
        rc_condset_evaluate_standard(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_PAUSE_IF:
        rc_condset_evaluate_pause_if(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_RESET_IF:
        rc_condset_evaluate_reset_if(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_TRIGGER:
        rc_condset_evaluate_trigger(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_MEASURED:
        rc_condset_evaluate_measured(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_MEASURED_IF:
        rc_condset_evaluate_measured_if(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_ADD_SOURCE:
      case RC_CONDITION_SUB_SOURCE:
      case RC_CONDITION_ADD_ADDRESS:
      case RC_CONDITION_REMEMBER:
        /* these are handled by rc_modified_memref_t */
        break;
      case RC_CONDITION_ADD_HITS:
        rc_condset_evaluate_add_hits(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_SUB_HITS:
        rc_condset_evaluate_sub_hits(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_RESET_NEXT_IF:
        rc_condset_evaluate_reset_next_if(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_AND_NEXT:
        rc_condset_evaluate_and_next(condition, eval_state RC_HOTA(hot));
        break;
      case RC_CONDITION_OR_NEXT:
        rc_condset_evaluate_or_next(condition, eval_state RC_HOTA(hot));
        break;
      default:
        eval_state->stop_processing = 1;
        eval_state->is_true = eval_state->is_primed = 0;
        break;
    }

    if (eval_state->stop_processing && can_short_circuit)
      break;
  }
}

rc_condition_t* rc_condset_get_conditions(rc_condset_t* self) {
  if (self->conditions)
    return RC_GET_TRAILING(self, rc_condset_with_trailing_conditions_t, rc_condition_t, conditions);

  return NULL;
}

#ifdef RC_EVAL_PLAN
/* Optional device override: point this at a PSRAM allocator (e.g. ps_malloc) so the hot
 * arrays don't consume scarce internal RAM. NULL => libc malloc (host / default). A NULL
 * return from either path is handled gracefully — the eval falls back to the cold struct. */
void* (*g_rc_eval_hot_alloc)(size_t) = 0;

/* Build the compact HOT array parallel to the EVAL-iterated conditions (pause/reset/
 * hittarget/measured/other — NOT indirect, which the eval skips). Idempotent; lazy.
 * op1/op2 capture the operand union's raw 4 bytes (memref ptr / const / f32 bits) —
 * the memref pointer is stable so this is built ONCE; current_hits is the rw state. */
void rc_build_eval_hot(rc_condset_t* self) {
  rc_condition_t* conds;
  rc_eval_hot_t* hot;
  uint32_t n, i;

  if (!self || self->hot)
    return;

  n = (uint32_t)self->num_pause_conditions + self->num_reset_conditions +
      self->num_hittarget_conditions + self->num_measured_conditions + self->num_other_conditions;
  if (n == 0)
    return;

  hot = (rc_eval_hot_t*)(g_rc_eval_hot_alloc ? g_rc_eval_hot_alloc((size_t)n * sizeof(rc_eval_hot_t))
                                             : malloc((size_t)n * sizeof(rc_eval_hot_t)));
  if (!hot)
    return;   /* fallback: eval reads the cold rc_condition_t */

  conds = rc_condset_get_conditions(self);
  for (i = 0; i < n; ++i) {
    hot[i].op1 = conds[i].operand1.value.num;
    hot[i].op2 = conds[i].operand2.value.num;
    hot[i].current_hits = conds[i].current_hits;
    hot[i].type = conds[i].type;
    hot[i].oper = conds[i].oper;
    hot[i].cmp = conds[i].optimized_comparator;
    hot[i].flags = (uint8_t)((conds[i].is_true & 0x03) | (conds[i].required_hits ? 0x04 : 0x00));
  }

  self->hot = hot;
}
#endif

#ifdef RC_EVAL_PLAN_HOST_SYNC
/* HOST-ONLY: mirror the eval-canonical hot fields back into the cold rc_condition_t so
 * the host tests / serialization / reset (which read the cold struct) stay correct. This
 * re-touches the full 32B/cond, NEGATING the streaming win on purpose — the host measures
 * correctness, not perf. The DEVICE build does NOT define RC_EVAL_PLAN_HOST_SYNC. */
static void rc_sync_hot_to_cold(rc_condset_t* self) {
  rc_eval_hot_t* hot = self->hot;
  rc_condition_t* conds;
  uint32_t n, i;

  if (!hot)
    return;

  conds = rc_condset_get_conditions(self);
  n = (uint32_t)self->num_pause_conditions + self->num_reset_conditions +
      self->num_hittarget_conditions + self->num_measured_conditions + self->num_other_conditions;
  for (i = 0; i < n; ++i) {
    conds[i].current_hits = hot[i].current_hits;
    conds[i].is_true = (uint8_t)(hot[i].flags & 0x03);
  }
}
#endif

int rc_test_condset(rc_condset_t* self, rc_eval_state_t* eval_state) {
  rc_condition_t* conditions;
#ifdef RC_EVAL_PLAN
  rc_eval_hot_t* hot;
#endif

#ifdef RC_EVAL_PLAN
  /* lazily build the compact HOT array on first eval. */
  if (!self->hot)
    rc_build_eval_hot(self);
#endif

  /* reset the processing state before processing each condset. do not reset the result state. */
  eval_state->measured_value.type = RC_VALUE_TYPE_NONE;
  eval_state->add_hits = 0;
  eval_state->is_true = 1;
  eval_state->is_primed = 1;
  eval_state->is_paused = 0;
  eval_state->can_measure = 1;
  eval_state->measured_from_hits = 0;
  eval_state->and_next = 1;
  eval_state->or_next = 0;
  eval_state->reset_next = 0;
  eval_state->stop_processing = 0;

  /* the conditions array is allocated immediately after the rc_condset_t, without a separate pointer */
  conditions = rc_condset_get_conditions(self);
#ifdef RC_EVAL_PLAN
  /* hot[] parallels conditions[] over the 5 eval categories (pause/reset/hittarget/
   * measured/other, in that order); advance it in lockstep with conditions. NULL if
   * the alloc failed -> the eval falls back to reading the cold rc_condition_t. */
  hot = self->hot;
#endif

  if (self->num_pause_conditions) {
    /* one or more Pause conditions exist. if any of them are true (eval_state->is_paused),
     * stop processing this group */
    rc_test_condset_internal(conditions, self->num_pause_conditions, eval_state, 1 RC_HOTA(hot));

    self->is_paused = eval_state->is_paused;
    if (self->is_paused) {
      /* condset is paused. stop processing immediately. */
#ifdef RC_EVAL_PLAN_HOST_SYNC
      rc_sync_hot_to_cold(self);
#endif
      return 0;
    }

    conditions += self->num_pause_conditions;
#ifdef RC_EVAL_PLAN
    if (hot) hot += self->num_pause_conditions;
#endif
  }

  if (self->num_reset_conditions) {
    /* one or more Reset conditions exists. if any of them are true (eval_state->was_reset),
     * we'll skip some of the later steps */
    rc_test_condset_internal(conditions, self->num_reset_conditions, eval_state, eval_state->can_short_curcuit RC_HOTA(hot));
    conditions += self->num_reset_conditions;
#ifdef RC_EVAL_PLAN
    if (hot) hot += self->num_reset_conditions;
#endif
  }

  if (self->num_hittarget_conditions) {
    /* one or more hit target conditions exists. these must be processed every frame,
     * unless their hit count is going to be reset */
    if (!eval_state->was_reset)
      rc_test_condset_internal(conditions, self->num_hittarget_conditions, eval_state, 0 RC_HOTA(hot));

    conditions += self->num_hittarget_conditions;
#ifdef RC_EVAL_PLAN
    if (hot) hot += self->num_hittarget_conditions;
#endif
  }

  if (self->num_measured_conditions) {
    /* IMPORTANT: reset hit counts on these conditions before processing them so
     *            the MeasuredIf logic and Measured value are correct.
     *      NOTE: a ResetIf in a later alt group may not have been processed yet.
     *            Accept that as a weird edge case, and just recommend the user
     *            move the ResetIf if it becomes a problem. */
    if (eval_state->was_reset) {
      int i;
      for (i = 0; i < self->num_measured_conditions; ++i) {
        conditions[i].current_hits = 0;
#ifdef RC_EVAL_PLAN
        if (hot) hot[i].current_hits = 0;
#endif
      }
    }

    /* the measured value must be calculated every frame, even if hit counts will be reset */
    rc_test_condset_internal(conditions, self->num_measured_conditions, eval_state, 0 RC_HOTA(hot));
    conditions += self->num_measured_conditions;
#ifdef RC_EVAL_PLAN
    if (hot) hot += self->num_measured_conditions;
#endif

    if (eval_state->measured_value.type != RC_VALUE_TYPE_NONE) {
      /* if a MeasuredIf was false (!eval_state->can_measure), or the measured
       * value is a hitcount and a ResetIf is true, zero out the measured value */
      if (!eval_state->can_measure ||
          (eval_state->measured_from_hits && eval_state->was_reset)) {
        eval_state->measured_value.type = RC_VALUE_TYPE_UNSIGNED;
        eval_state->measured_value.value.u32 = 0;
      }
    }
  }

  if (self->num_other_conditions) {
    /* the remaining conditions only need to be evaluated if the rest of the condset is true */
    if (eval_state->is_true)
      rc_test_condset_internal(conditions, self->num_other_conditions, eval_state, eval_state->can_short_curcuit RC_HOTA(hot));
    /* something else is false. if we can't short circuit, and there wasn't a reset, we still need to evaluate these */
    else if (!eval_state->can_short_curcuit && !eval_state->was_reset)
      rc_test_condset_internal(conditions, self->num_other_conditions, eval_state, eval_state->can_short_curcuit RC_HOTA(hot));
  }

#ifdef RC_EVAL_PLAN_HOST_SYNC
  rc_sync_hot_to_cold(self);
#endif
  return eval_state->is_true;
}

void rc_reset_condset(rc_condset_t* self) {
  rc_condition_t* condition;

  for (condition = self->conditions; condition != 0; condition = condition->next) {
    condition->current_hits = 0;
  }

#ifdef RC_EVAL_PLAN
  /* the eval keeps current_hits canonical in hot[]; reset it too so the next eval
   * doesn't read stale accruals (the cold loop above doesn't reach the hot array). */
  if (self->hot) {
    uint32_t n = (uint32_t)self->num_pause_conditions + self->num_reset_conditions +
                 self->num_hittarget_conditions + self->num_measured_conditions +
                 self->num_other_conditions;
    uint32_t i;
    for (i = 0; i < n; ++i)
      self->hot[i].current_hits = 0;
  }
#endif
}
