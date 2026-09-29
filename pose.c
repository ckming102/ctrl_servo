/*==============================================================================
  Source for poses and pose sequences; see pose.h
 =============================================================================*/
#include <avr/eeprom.h>
#include <util/crc16.h>
#include "global.h"
#include "motion.h"
#include "pose.h"

/* ------------------ */
/*  Static variables  */
/* ------------------ */

#define POSE_MAGIC 0xA5

/* one EEPROM slot; erased EEPROM reads 0xFF so an unused slot has no magic */
typedef struct POSE_RECORD
{
    uint8_t magic;
    uint16_t us[N_JOINTS];
    uint8_t crc;
} POSE_RECORD;

static POSE_RECORD EEMEM pose_eeprom[POSE_SLOTS];

/* sequence state */
enum seq_states
{
    seq_idle, seq_next, seq_moving, seq_dwell
};

static uint8_t seq_state = seq_idle;
static uint8_t seq_slots[SEQ_MAX_STEPS];
static uint8_t seq_len;
static uint8_t seq_idx;
static uint8_t seq_loop;
static uint16_t seq_dwell_ms = SEQ_DWELL_DEFAULT_MS;
static uint16_t seq_dwell_start;

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

static uint8_t _Crc(const POSE_RECORD *rec)
{
    const uint8_t *byte = (const uint8_t *)rec;
    uint8_t crc = 0;
    uint8_t i;

    /* every byte except the crc itself */
    for(i = 0; i < sizeof(POSE_RECORD) - 1; i++)
        crc = _crc8_ccitt_update(crc, byte[i]);
    return crc;
}

int POSE_Save(uint8_t slot, const uint16_t us[N_JOINTS])
{
    POSE_RECORD rec;
    uint8_t j;

    if(slot >= POSE_SLOTS)
        return -1;

    rec.magic = POSE_MAGIC;
    for(j = 0; j < N_JOINTS; j++) rec.us[j] = us[j];
    rec.crc = _Crc(&rec);

    eeprom_update_block(&rec, &pose_eeprom[slot], sizeof(rec));
    return 0;
}

int POSE_Load(uint8_t slot, uint16_t us[N_JOINTS])
{
    POSE_RECORD rec;
    uint8_t j;

    if(slot >= POSE_SLOTS)
        return -1;

    eeprom_read_block(&rec, &pose_eeprom[slot], sizeof(rec));
    if(rec.magic != POSE_MAGIC || rec.crc != _Crc(&rec))
        return -1;

    for(j = 0; j < N_JOINTS; j++) us[j] = rec.us[j];
    return 0;
}

int POSE_Clear(uint8_t slot)
{
    if(slot >= POSE_SLOTS)
        return -1;

    eeprom_update_byte(&pose_eeprom[slot].magic, 0xFF);
    return 0;
}

/* # Start playing a list of pose slots */
int SEQ_Start(const uint8_t *slots, uint8_t n_slots, uint8_t loop)
{
    uint16_t us[N_JOINTS];
    uint8_t i;

    if(n_slots == 0 || n_slots > SEQ_MAX_STEPS)
        return -1;

    /* check every pose up front so a bad slot doesn't stop the arm half way */
    for(i = 0; i < n_slots; i++)
        if(POSE_Load(slots[i], us) != 0)
            return -1;

    for(i = 0; i < n_slots; i++) seq_slots[i] = slots[i];
    seq_len = n_slots;
    seq_idx = 0;
    seq_loop = loop;
    seq_state = seq_next;
    return 0;
}

void SEQ_Stop(void)
{
    seq_state = seq_idle;
}

uint8_t SEQ_Active(void)
{
    return seq_state != seq_idle;
}

uint8_t SEQ_Step(void)
{
    return seq_idx + 1;
}

uint8_t SEQ_Length(void)
{
    return seq_len;
}

uint8_t SEQ_Looping(void)
{
    return seq_loop;
}

void SEQ_SetDwell(uint16_t ms)
{
    seq_dwell_ms = ms;
}

uint16_t SEQ_GetDwell(void)
{
    return seq_dwell_ms;
}

/* # Advance the sequence */
uint8_t SEQ_Poll(void)
{
    uint16_t us[N_JOINTS];
    uint16_t dwell_ticks;

    switch(seq_state)
    {
        case seq_next:
            if(POSE_Load(seq_slots[seq_idx], us) != 0)
            {
                /* slot erased or overwritten since SEQ_Start */
                seq_state = seq_idle;
                return SEQ_EVT_ERROR;
            }
            MOTION_MoveTo(us);
            seq_state = seq_moving;
        break;

        case seq_moving:
            if(!MOTION_Busy())
            {
                seq_dwell_start = MOTION_Ticks();
                seq_state = seq_dwell;
            }
        break;

        case seq_dwell:
            dwell_ticks = (uint16_t)(((uint32_t)seq_dwell_ms * MOTION_TickHz()) / 1000UL);
            if((uint16_t)(MOTION_Ticks() - seq_dwell_start) < dwell_ticks)
                break;

            seq_idx++;
            if(seq_idx >= seq_len)
            {
                if(!seq_loop)
                {
                    seq_state = seq_idle;
                    return SEQ_EVT_FINISHED;
                }
                seq_idx = 0;
            }
            seq_state = seq_next;
        break;

        default:
        break;
    }
    return SEQ_EVT_NONE;
}
