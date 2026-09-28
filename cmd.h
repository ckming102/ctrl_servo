/*==============================================================================
  Header for the Command Table and terminal input
 =============================================================================*/
#ifndef CMD_H
#define CMD_H

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <avr/io.h>
#include "global.h"
#include "uart.h"

/* ---------------------------------- */
/*  Registered commands and callbacks */
/* ---------------------------------- */

typedef struct CMD
{
    /* what the user types */
    const char * name;
    /* arguments and description, shown by help */
    const char * usage;
    /* returns 0, or an error number that is reported to the user */
    int (*callback)(uint8_t argc, char ** argv);
} CMD;

/* --------------------------------- */
/*  Command arguments from terminal  */
/* --------------------------------- */

#define CLI_LINE_SIZE 80
#define ARGV_SIZE 24

/* ------------------------------ */
/*  Keys returned by DecodeKey()  */
/* ------------------------------ */

/* plain characters are returned as 0 - 255 */
#define KEY_NONE   -1
#define KEY_UP     0x101
#define KEY_DOWN   0x102
#define KEY_RIGHT  0x103
#define KEY_LEFT   0x104
#define KEY_ENTER  '\r'
#define KEY_CTRL_C 0x03

/* ----------------------------------- */
/*  Helper functions for parsing etc.  */
/* ----------------------------------- */

/* tokenize str_buffer for argument passing, str_buffer is modified.
   Returns the number of tokens, or max_tokens + 1 if there are too many. */
extern uint8_t tokenize(
    char **tokens, uint8_t max_tokens, char *str_buffer, const char* delim
);

/* register the command table */
extern void cli_Init(const CMD *cmd_table, uint8_t n_cmds);

/* turn a received byte into a key; arrow key escape sequences become
   KEY_UP etc. Returns KEY_NONE while in the middle of a sequence. */
extern int16_t cli_DecodeKey(uint8_t data);

/* line editing; returns TRUE when a command line was run (enter pressed) */
extern uint8_t cli_Keypress(int16_t key);

/* print the prompt and any partly typed line */
extern void cli_Prompt(void);

/* forget any partly typed line */
extern void cli_Reset(void);

/* list the registered commands */
extern void cli_PrintHelp(void);

#endif
