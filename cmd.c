/*==============================================================================
  Source for the Command Table and terminal input
  =============================================================================*/

#include "cmd.h"

/* ------------------ */
/*  Static variables  */
/* ------------------ */

/* registered commands */
static const CMD *cli_cmds;
static uint8_t cli_n_cmds;

/* line being typed */
static char cli_line[CLI_LINE_SIZE];
static uint8_t cli_len;

/* previous key, so CR LF counts as one enter */
static int16_t cli_last_key = KEY_NONE;

/* ----------------------------------- */
/*  Helper functions for parsing etc.  */
/* ----------------------------------- */

/* tokenize str_buffer for argument passing, str_buffer is modified */
uint8_t tokenize(
    char **tokens, uint8_t max_tokens, char *str_buffer, const char* delim
)
{
    /* the number of string tokens found */
    uint8_t n_tokens;

    /* first call */
    char * token = strtok(str_buffer, delim);

    /* step through */
    for(n_tokens = 0; (token != NULL); n_tokens++)
    {
        /* never write past the end of tokens */
        if(n_tokens == max_tokens)
            return max_tokens + 1;

        tokens[n_tokens] = token;
        token = strtok(NULL, delim);
    }

    return n_tokens;
}

void cli_Init(const CMD *cmd_table, uint8_t n_cmds)
{
    cli_cmds = cmd_table;
    cli_n_cmds = n_cmds;
    cli_Reset();
}

void cli_Reset(void)
{
    cli_len = 0;
    cli_line[0] = '\0';
}

void cli_Prompt(void)
{
    uint8_t i;

    uart_SendString("> ");
    for(i = 0; i < cli_len; i++) uart_SendByte(cli_line[i]);
}

void cli_PrintHelp(void)
{
    uint8_t i;
    uint8_t pad;

    uart_SendString("\n\r# List of commands\n\r\n\r");
    for(i = 0; i < cli_n_cmds; i++)
    {
        uart_SendString("  ");
        uart_SendString(cli_cmds[i].name);
        for(pad = strlen(cli_cmds[i].name); pad < 12; pad++) uart_SendByte(' ');
        uart_SendString(cli_cmds[i].usage);
        uart_SendString("\n\r");
    }
    uart_SendString(
        "\n\r  Ctrl-C      stop all motion (works in every mode)\n\r"
        "  Joints are A0 B0 C0 (pins 11 12 13) and A1 B1 C1 (pins 6 7 8)\n\r\n\r"
    );
}

/* for running a typed command line */
static void cli_Execute(char *line)
{
    char *argv[ARGV_SIZE];
    uint8_t argc;
    uint8_t i;
    int err_no;

    /* tokenize the line */
    argc = tokenize(argv, ARGV_SIZE, line, " \t");

    /* empty line */
    if(argc == 0)
        return;

    if(argc > ARGV_SIZE)
    {
        uart_SendString("Too many arguments\n\r");
        return;
    }

    /* loop through command list and call relevant callback */
    for(i = 0; i < cli_n_cmds; i++)
    {
        if(!strcmp(argv[0], cli_cmds[i].name))
        {
            /* call the relevant callback */
            err_no = cli_cmds[i].callback(argc, argv);

            /* report errors */
            if(err_no != 0)
            {
                uart_SendString("Error:");
                uart_SendInt(err_no);
                uart_SendString(" in Cmd:");
                uart_SendString(cli_cmds[i].name);
                uart_SendString("\n\r");
            }
            return;
        }
    }

    /* unknown command */
    uart_SendString(argv[0]);
    uart_SendString(": command not found\n\r");
}

/* behavior after keypress */
uint8_t cli_Keypress(int16_t key)
{
    int16_t last_key = cli_last_key;
    cli_last_key = key;

    switch(key)
    {
        /* backspace; terminals send either BS or DEL */
        case '\b':
        case 0x7F:
            if(cli_len > 0)
            {
                cli_len--;
                uart_SendString("\b \b");
            }
        break;

        /* enter; CR, LF or CR LF */
        case '\n':
            if(last_key == '\r')
                break;
            /* fall through */
        case '\r':
            uart_SendString("\n\r");
            /* finalize by appending '\0' */
            cli_line[cli_len] = '\0';
            cli_Execute(cli_line);
            cli_Reset();
            return TRUE;

        default:
            /* printable characters only; drop arrow keys etc. */
            if(key >= 0x20 && key < 0x7F)
            {
                /* always keeping 1 extra byte for '\0' */
                if(cli_len < CLI_LINE_SIZE - 1)
                {
                    cli_line[cli_len++] = (char)key;
                    /* echo the char */
                    uart_SendByte((char)key);
                }
                /* line full */
                else uart_SendByte('\a');
            }
        break;
    }
    return FALSE;
}

/* ---------------------- */
/*  Escape key sequences  */
/* ---------------------- */

static int16_t _ArrowKey(uint8_t data)
{
    switch(data)
    {
        case 'A': return KEY_UP;
        case 'B': return KEY_DOWN;
        case 'C': return KEY_RIGHT;
        case 'D': return KEY_LEFT;
        default: return KEY_NONE;
    }
}

/* Arrow keys arrive as ESC [ A..D (or ESC O A..D). Other escape sequences,
   e.g. ESC [ 5 ~ for page up, are swallowed whole. */
int16_t cli_DecodeKey(uint8_t data)
{
    enum { esc_none, esc_start, esc_csi, esc_ss3 };
    static uint8_t esc_state = esc_none;

    switch(esc_state)
    {
        case esc_start:
            esc_state = esc_none;
            if(data == '[') { esc_state = esc_csi; return KEY_NONE; }
            if(data == 'O') { esc_state = esc_ss3; return KEY_NONE; }
            /* lone ESC: handle data as a normal key below */
        break;

        case esc_csi:
            /* parameter and intermediate bytes, e.g. the "1;5" in ESC [ 1;5 A */
            if(data >= 0x20 && data <= 0x3F)
                return KEY_NONE;
            esc_state = esc_none;
            return _ArrowKey(data);

        case esc_ss3:
            esc_state = esc_none;
            return _ArrowKey(data);
    }

    /* escape character */
    if(data == 0x1B)
    {
        esc_state = esc_start;
        return KEY_NONE;
    }
    return data;
}
