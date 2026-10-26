#ifndef S1_COMMAND_CODEC_H
#define S1_COMMAND_CODEC_H

#include <stdint.h>

typedef enum {
    S1_CMD_INVALID = 0x00,
    S1_CMD_START   = 0x01,
    S1_CMD_STOP    = 0x02,
    S1_CMD_RESET   = 0x03
} S1_COMMAND;

static inline S1_COMMAND s1_decode_command(const uint8_t *data,
                                           unsigned int length)
{
    if (data == 0 || length != 1U || (data[0] & 0xFCu) != 0U) {
        return S1_CMD_INVALID;
    }

    return (S1_COMMAND)(data[0] & 0x03u);
}

#endif