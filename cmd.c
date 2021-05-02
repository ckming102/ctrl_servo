/*==============================================================================
  Source for the Command Table                                                
  =============================================================================*/

#include "cmd.h"

/* --------------------------------- */
/*  Command arguments from terminal  */
/* --------------------------------- */

char *argv[ARGV_SIZE];
uint8_t argc;

/* ----------------------------------- */
/*  Helper functions for parsing etc.  */
/* ----------------------------------- */

/* tokenize str_buffer for argument passing, str_buffer is modified */
uint8_t tokenize(char **tokens, char *str_buffer, const char* delim)
{
    /* the number of string tokens found */
    uint8_t n_tokens;

    /* first call */
    char * token = strtok(str_buffer, delim);

    /* step through */
    for(n_tokens =0; (token != NULL); n_tokens++)
    {
        tokens[n_tokens] = token;
        token = strtok(NULL, delim);
    }

    return n_tokens;
}

/* for parsing commands in cli context */
void cli_ParseCommand(unsigned int cmd_list_len)
{
    /* if command execute flag on */
    if(status.cmd_check == TRUE)
    {
        /* new line */
        uart_SendString("\n\r");

        /* tokenize UART_RxBuffer */
        argc = tokenize(argv, UART_RxBuffer, " ");

        /* loop through command list and call relevant callback */
        status.cmd_executed = FALSE;
        for( int i = 0; i < cmd_list_len; i++)
        {
            if(!strcmp(argv[0], cmd_name[i]))
            {
                /* call the relevant callback */
                err_no = (cmd_list[i])(argc, argv);
                status.cmd_executed = TRUE;

                /* report errors */
                if(err_no != 0)
                {
                    uart_SendString("Error:");
                    uart_SendInt(err_no);
                    uart_SendString(" in Cmd:");
                    uart_SendString(cmd_name[i]);
                    uart_SendString("\n\r");
                }
                break;
            }
        }
        /* unknown command */
        if(status.cmd_executed == FALSE)
        {
            uart_SendString(argv[0]);
            uart_SendString(": command not found\n\r");
        }

        /* reset */
        uart_FlushRxBuffer();
        status.cmd_check = FALSE;
    }
}

/* behavior after keypress */
void cli_Keypress()
{
    /* copy-in byte */
    char data = *UART_UDRn;

    /* if command checking flag is off */
    if(status.cmd_check == FALSE)
    {

        /* make sure not a control sequence */
        if(!iscntrl(data))
        {
            /* store in char buffer */
            UART_RxBuffer[UART_RxPtr] = data;

            /* make sure there is space in the buffer 
               always keeping 1 extra byte for '\0' */
            if(UART_RxPtr < UART_RX_BUFFER_SIZE - 2)
            {
                UART_RxPtr++;
            }
            /* max size */
            else uart_SendByte('\b');

            /* echo the char */
            uart_SendByte(data);
        }
        /* respond only to backspace and enter */
        else
        {
            switch(data)
            {
                /* backspace */
                case '\b':
                    uart_SendByte('\b');
                    uart_SendByte(' ');
                    uart_SendByte('\b');
                    if(UART_RxPtr>0) UART_RxPtr--;
                break;

                /* enter */
                case 13:
                    /* finalize by appending '\0' */
                    UART_RxBuffer[UART_RxPtr] = '\0';
                    /* set command execute flag on */
                    status.cmd_check = TRUE;
                break;
            }
        }
    }
    status.rx_int = FALSE;
}