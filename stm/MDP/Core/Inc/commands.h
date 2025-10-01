// UART Command format
// Structure: <cmd>|<param>\n
// cmd:
//  - 'T' : move straight forward,  param = distance in cm
//  - 't' : move straight backward, param = distance in cm
//  - 'L' : turn left  (forward),   param = target yaw angle in degrees
//  - 'R' : turn right (forward),   param = target yaw angle in degrees
//  - 'l' : turn left  (backward),  param = target yaw angle in degrees
//  - 'r' : turn right (backward),  param = target yaw angle in degrees
// The terminator is a newline '\n'. Example: T|30\n, r|45\n

#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_FULL_STOP             'S'
#define CMD_TURN_IN_PLACE         'P'
#define CMD_FORWARD_DIST_TARGET   'T'
#define CMD_FORWARD_DIST_AWAY     'W'
#define CMD_BACKWARD_DIST_TARGET  't'
#define CMD_BACKWARD_DIST_AWAY    'w'
#define CMD_FORWARD_DIST_L        'L'
#define CMD_FORWARD_DIST_R        'R'
#define CMD_BACKWARD_DIST_L       'l'
#define CMD_BACKWARD_DIST_R       'r'
#define CMD_INFO_MARKER           'M'
#define CMD_INFO_DIST             'D'

#define CMD_SEP '|'               // field separator
#define CMD_END '\n'             // command terminator
#define CMD_RCV 'r'               // acknowledgement: received
#define CMD_FIN 'f'               // acknowledgement: finished

#define COMMAND_MAX_STRING_LEN 64U

typedef enum {
    COMMAND_OP_INVALID = 0,
    COMMAND_OP_DRIVE,
    COMMAND_OP_TURN,
    COMMAND_OP_STOP,
    COMMAND_OP_INFO_DIST,
    COMMAND_OP_INFO_MARKER
} command_op_e;

typedef enum {
    COMMAND_DIST_TARGET = 0,
    COMMAND_DIST_STOP_AWAY,
    COMMAND_DIST_STOP_L,
    COMMAND_DIST_STOP_R,
    COMMAND_DIST_STOP_L_LESS,
    COMMAND_DIST_STOP_R_LESS
} command_dist_e;

typedef struct Command {
    command_op_e opType;
    int8_t dir;
    uint16_t speed;
    float angleToSteer;
    float val;
    command_dist_e distType;
    uint8_t shouldSend;
    uint8_t *str;
    uint8_t str_size;
    struct Command *next;
} Command;

void commands_init(void);
void commands_process(UART_HandleTypeDef *uart, const uint8_t *buf, uint16_t size);
Command *commands_peek(void);
Command *commands_peek_next_drive(void);
Command *commands_pop(void);
void commands_end(UART_HandleTypeDef *uart, Command *cmd);
uint8_t commands_type_match(const Command *a, const Command *b);
void commands_clear_queue(UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif

