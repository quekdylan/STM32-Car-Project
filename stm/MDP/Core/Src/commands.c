#include "commands.h"

#include <stdlib.h>
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

    char *payload = &parse_buffer[1];
    char *speed_token = NULL;
    char *angle_token = NULL;
    char *value_token = NULL;

    if (*payload != '\0') {
        speed_token = strtok(payload, "|");
        if (speed_token != NULL) {
            angle_token = strtok(NULL, "|");
        }
        if (angle_token != NULL) {
            value_token = strtok(NULL, "|");
        }
    }

    cmd->speed = (uint16_t)parse_uint16(speed_token);
    cmd->angleToSteer = parse_float(angle_token);
    cmd->val = parse_float(value_token);
    cmd->shouldSend = 1U;

    cmd->opType = COMMAND_OP_DRIVE;
    cmd->dir = 0;
    cmd->distType = COMMAND_DIST_TARGET;

    switch (flag) {
        case CMD_FULL_STOP:
            cmd->opType = COMMAND_OP_STOP;
            cmd->dir = 0;
            break;
        case CMD_FORWARD_DIST_TARGET:
            cmd->dir = 1;
            cmd->distType = COMMAND_DIST_TARGET;
            break;
        case CMD_FORWARD_DIST_AWAY:
            cmd->dir = 1;
            cmd->distType = COMMAND_DIST_STOP_AWAY;
            break;
        case CMD_BACKWARD_DIST_TARGET:
            cmd->dir = -1;
            cmd->distType = COMMAND_DIST_TARGET;
            break;
        case CMD_BACKWARD_DIST_AWAY:
            cmd->dir = -1;
            cmd->distType = COMMAND_DIST_STOP_AWAY;
            break;
        case CMD_FORWARD_DIST_L:
            cmd->dir = 1;
            cmd->distType = COMMAND_DIST_STOP_L;
            break;
        case CMD_FORWARD_DIST_R:
            cmd->dir = 1;
            cmd->distType = COMMAND_DIST_STOP_R;
            break;
        case CMD_BACKWARD_DIST_L:
            cmd->dir = -1;
            cmd->distType = COMMAND_DIST_STOP_L_LESS;
            break;
        case CMD_BACKWARD_DIST_R:
            cmd->dir = -1;
            cmd->distType = COMMAND_DIST_STOP_R_LESS;
            break;
        case CMD_INFO_DIST:
            cmd->opType = COMMAND_OP_INFO_DIST;
            break;
        case CMD_INFO_MARKER:
            cmd->opType = COMMAND_OP_INFO_MARKER;
            break;
        default:
            free(cmd->str);
            free(cmd);
            return;
    }

    if (cmd->opType == COMMAND_OP_DRIVE && fabsf(cmd->angleToSteer) > 0.01f) {
        cmd->opType = COMMAND_OP_TURN;
    }

    if (s_head == NULL) {
        s_head = cmd;
        s_tail = cmd;
    } else {
        s_tail->next = cmd;
        s_tail = cmd;
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
    if (uart == NULL || cmd == NULL || cmd->str == NULL || cmd->str_size == 0U) {
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
    if (text == NULL || *text == '\0') {
        return 0U;
    }

    uint32_t value = 0U;
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
