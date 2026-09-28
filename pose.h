/*==============================================================================
  Header for poses and pose sequences

    Description
    -----------
    A pose is a pulse width for every joint. Poses are kept in EEPROM slots so
    they survive power cycles and re-flashing. A sequence plays a list of pose
    slots in order, moving smoothly between them (MOTION_MoveTo) and pausing
    for the dwell time at each one.

 =============================================================================*/
#ifndef POSE_H
#define POSE_H

#include "global.h"
#include "motion.h"

#define POSE_SLOTS 16
#define SEQ_MAX_STEPS 16
#define SEQ_DWELL_DEFAULT_MS 500

/* events returned by SEQ_Poll */
#define SEQ_EVT_NONE     0
#define SEQ_EVT_FINISHED 1
#define SEQ_EVT_ERROR    2

// Store / fetch / erase a pose; return -1 for a bad slot or an empty slot
extern int POSE_Save(uint8_t slot, const uint16_t us[N_JOINTS]);
extern int POSE_Load(uint8_t slot, uint16_t us[N_JOINTS]);
extern int POSE_Clear(uint8_t slot);

// Play pose slots in order, repeating if loop is set.
// Returns -1 (and plays nothing) if any slot is bad or empty.
extern int SEQ_Start(const uint8_t *slots, uint8_t n_slots, uint8_t loop);
extern void SEQ_Stop(void);
extern uint8_t SEQ_Active(void);

// Current step (1-based) and number of steps
extern uint8_t SEQ_Step(void);
extern uint8_t SEQ_Length(void);
extern uint8_t SEQ_Looping(void);

// Pause at each pose, in ms
extern void SEQ_SetDwell(uint16_t ms);
extern uint16_t SEQ_GetDwell(void);

// Advance the sequence; call often from the main loop. Returns a SEQ_EVT_*.
extern uint8_t SEQ_Poll(void);

#endif
