/*==============================================================================
  Header for the Command Table
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
    extern char * cmd_name[];
    extern int (*cmd_list[])(uint8_t argc, char ** argv);

/* --------------------------------- */
/*  Command arguments from terminal  */
/* --------------------------------- */
    # define ARGV_SIZE 24
    extern char *argv[ARGV_SIZE];
    extern uint8_t argc;

/* ----------------------------------- */
/*  Helper functions for parsing etc.  */
/* ----------------------------------- */

    /* tokenize str_buffer for argument passing, str_buffer is modified */
    uint8_t tokenize(char **tokens, char *str_buffer, const char* delim);

    /* for parsing commands in cli context */
    void cli_ParseCommand(unsigned int cmd_list_len);

    /* behavior after keypress */
    void cli_Keypress();


#endif