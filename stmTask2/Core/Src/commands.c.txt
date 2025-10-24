#include "commands.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#ifndef HAL_MAX_DELAY
#define HAL_MAX_DELAY 0xFFFFFFFFU
#endif

static Command *s_head = NULL;
static Command *s_tail = NULL;

static Command *get_new_cmd(void);
static void commands_ack(UART_HandleTypeDef *uart, const Command *cmd, uint8_t indicator);
static uint16_t parse_uint16(const char *text);
static float parse_float(const char *text);
static uint8_t parse_snap_command(Command *cmd, const char *text);

void commands_init(void)
{
    Command *node = s_head;
    while (node) {
        Command *next = node->next;
        free(node->str);
        free(node);
        node = next;
    }
    s_head = NULL;
    s_tail = NULL;
}

void commands_process(UART_HandleTypeDef *uart, const uint8_t *buf, uint16_t size)
{
    if (uart == NULL || buf == NULL || size == 0U) {
        return;
    }

    if (size > COMMAND_MAX_STRING_LEN) {
        return;
    }

    char parse_buffer[COMMAND_MAX_STRING_LEN + 1U];
    memcpy(parse_buffer, buf, size);
    parse_buffer[size] = '\0';

    if (parse_buffer[size - 1U] == CMD_END) {
        parse_buffer[size - 1U] = '\0';
    }

    char flag = parse_buffer[0];
    if (flag == '\0' || flag == CMD_TURN_IN_PLACE) {
        return;
    }

    Command *cmd = get_new_cmd();
    if (cmd == NULL) {
        return;
    }

    cmd->str_size = (uint8_t)size;
    cmd->str = (uint8_t *)malloc(size);
    if (cmd->str == NULL) {
        free(cmd);
        return;
    }
    memcpy(cmd->str, buf, size);

    // Defaults
    cmd->speed = 0U;
    cmd->angleToSteer = 0.0f;
    cmd->val = 0.0f;
    cmd->shouldSend = 1U;

    cmd->opType = COMMAND_OP_DRIVE;
    cmd->dir = 0;
    cmd->distType = COMMAND_DIST_TARGET;

    if (parse_snap_command(cmd, parse_buffer)) {
        // SNAP command handled
    } else {
        char flag = parse_buffer[0];
        if (flag == '\0' || flag == CMD_TURN_IN_PLACE) {
            free(cmd->str);
            free(cmd);
            return;
        }

        // Formats supported: <cmd><param> or legacy <cmd>|<param>
        const char *param_text = NULL;
        if (parse_buffer[1] != '\0') {
            const char *p = &parse_buffer[1];
            if (*p == CMD_SEP) {
                ++p;
            }
            while (*p != '\0' && isspace((unsigned char)*p)) {
                ++p;
            }
            if (*p != '\0') {
                param_text = p;
            }
        }

        float param_val = parse_float(param_text);

        // External reference to navigation direction variable
        extern volatile char g_rpi_direction;
        extern volatile uint32_t g_rpi_direction_seq;

        switch (flag) {
        case CMD_FULL_STOP: // 'S' - trigger default routine instead of queuing
        {
            extern volatile uint8_t g_start_default_requested;
            g_start_default_requested = 1U;
            free(cmd->str);
            free(cmd);
            return;
        }
        case 'W': // forward move with obstacle avoidance (deprecated, kept for compatibility)
            cmd->opType = COMMAND_OP_DRIVE;
            cmd->dir = 1;
            cmd->distType = COMMAND_DIST_STOP_AWAY;
            cmd->val = fabsf(param_val);
            break;
        case 'A': // Legacy: Set navigation direction to LEFT
        case 'L': // New: Set navigation direction to LEFT
            g_rpi_direction = 'A';
            g_rpi_direction_seq++;
            free(cmd->str);
            free(cmd);
            return; // Don't queue this command
        case 'B': // Legacy: Set navigation direction to RIGHT
        case 'R': // New: Set navigation direction to RIGHT
            g_rpi_direction = 'B';
            g_rpi_direction_seq++;
            free(cmd->str);
            free(cmd);
            return; // Don't queue this command
        default:
            free(cmd->str);
            free(cmd);
            return;
    }

    }


    if (s_head == NULL) {
        s_head = cmd;
        s_tail = cmd;
    } else {
        s_tail->next = cmd;
        s_tail = cmd;
    }

    // For SNAP commands, delay acknowledgment to allow robot to stop
    // This prevents camera from snapping while robot is still moving
    if (cmd->opType == COMMAND_OP_IMU_CALIBRATE) {
        // Give time for any ongoing movement to complete and robot to settle
        HAL_Delay(500);  // 500ms settling time
    }

    commands_ack(uart, cmd, CMD_RCV);
}

Command *commands_peek(void)
{
    return s_head;
}

Command *commands_peek_next_drive(void)
{
    Command *node = s_head;
    while (node) {
        if (node->opType == COMMAND_OP_DRIVE || node->opType == COMMAND_OP_TURN) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

Command *commands_pop(void)
{
    Command *node = s_head;
    if (node) {
        s_head = node->next;
        if (s_head == NULL) {
            s_tail = NULL;
        }
        node->next = NULL;
    }
    return node;
}

void commands_end(UART_HandleTypeDef *uart, Command *cmd)
{
    if (cmd == NULL) {
        return;
    }
    if (uart && cmd->shouldSend) {
        commands_ack(uart, cmd, CMD_FIN);
    }
    free(cmd->str);
    free(cmd);
}

uint8_t commands_type_match(const Command *a, const Command *b)
{
    if (a == NULL || b == NULL) {
        return 0U;
    }
    return (uint8_t)((a->dir == b->dir)
            && (a->speed == b->speed)
            && (fabsf(a->angleToSteer - b->angleToSteer) < 0.0001f)
            && (a->distType == b->distType));
}

void commands_clear_queue(UART_HandleTypeDef *uart)
{
    Command *node = commands_pop();
    while (node) {
        commands_end(uart, node);
        node = commands_pop();
    }
}

static Command *get_new_cmd(void)
{
    Command *cmd = (Command *)malloc(sizeof(Command));
    if (cmd) {
        cmd->opType = COMMAND_OP_DRIVE;
        cmd->dir = 0;
        cmd->speed = 0U;
        cmd->angleToSteer = 0.0f;
        cmd->val = 0.0f;
        cmd->distType = COMMAND_DIST_TARGET;
        cmd->shouldSend = 1U;
        cmd->str = NULL;
        cmd->str_size = 0U;
        cmd->next = NULL;
    }
    return cmd;
}

static void commands_ack(UART_HandleTypeDef *uart, const Command *cmd, uint8_t indicator)
{
    if (uart == NULL || cmd == NULL || cmd->str == NULL || cmd->str_size <= 1U) {
        return;
    }

    uint16_t buf_size = (uint16_t)(cmd->str_size + 1U);
    uint8_t *tx = (uint8_t *)malloc(buf_size);
    if (tx == NULL) {
        return;
    }

    tx[0] = indicator;
    memcpy(&tx[1], cmd->str, cmd->str_size);
    (void)HAL_UART_Transmit(uart, tx, buf_size, HAL_MAX_DELAY);
    free(tx);
}

static uint16_t parse_uint16(const char *text)
{
    uint32_t value = 0U;
    if (text == NULL || *text == '\0') {
        return 0U;
    }
    while (*text != '\0') {
        if (!isdigit((unsigned char)*text)) {
            break;
        }
        value = (value * 10U) + (uint32_t)(*text - '0');
        if (value > 0xFFFFU) {
            value = 0xFFFFU;
            break;
        }
        ++text;
    }
    return (uint16_t)value;
}

static float parse_float(const char *text)
{
    if (text == NULL || *text == '\0') {
        return 0.0f;
    }
    return strtof(text, NULL);
}

static uint8_t parse_snap_command(Command *cmd, const char *text)
{
    if (cmd == NULL || text == NULL) {
        return 0U;
    }

    if (strncmp(text, "SNAP", 4) != 0) {
        return 0U;
    }

    const char *digits = text + 4;
    char *suffix = NULL;
    long seq = strtol(digits, &suffix, 10);

    if (suffix == NULL) {
        return 0U;
    }

    // Accept _C, _L, or _R suffixes
    if (strcmp(suffix, "_C") != 0 && strcmp(suffix, "_L") != 0 && strcmp(suffix, "_R") != 0) {
        return 0U;
    }

    if (seq < 0L) {
        seq = 0L;
    }

    // Clamp to reasonable range to avoid huge floats
    if (seq > 1000000L) {
        seq = 1000000L;
    }

    cmd->opType = COMMAND_OP_IMU_CALIBRATE;
    cmd->shouldSend = 0U; // suppress FIN acknowledgement; only send initial receipt
    cmd->val = (float)seq;
    cmd->dir = 0;
    cmd->distType = COMMAND_DIST_TARGET;
    return 1U;
}
