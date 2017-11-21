#pragma once

#include <stdint.h>

#define NCHANS  2

#define CMD_NONE        0x00
#define CMD_MOVE        0x10 // [nsteps:i32, bdelay:u32, edelay:u32]*NCHANS

#define CMD_NONE_SIZE   0
#define CMD_MOVE_SIZE   ((4+4+4)*NCHANS)

uint32_t cmd_size(uint8_t id);

typedef struct {

} CmdNone;

void cmd_none_load(const uint8_t *data, CmdNone *cmd);
void cmd_none_store(uint8_t *data, const CmdNone *cmd);

typedef struct {
  int32_t steps;
  uint32_t bdelay;
  uint32_t edelay;
} CmdMoveChan;

typedef struct {
  CmdMoveChan chans[NCHANS];
} CmdMove;

void cmd_move_load(const uint8_t *data, CmdMove *cmd);
void cmd_move_store(uint8_t *data, const CmdMove *cmd);
