#include <cmds.h>


uint32_t cmd_size(uint8_t id) {
  if (id == CMD_MOVE) {
    return CMD_MOVE_SIZE;
  }
  return 0;
}

void cmd_none_load(const uint8_t *data, CmdNone *cmd) {

}

void cmd_none_store(uint8_t *data, const CmdNone *cmd) {

}

void cmd_move_load(const uint8_t *data, CmdMove *cmd) {
  uint32_t i;
  for (i = 0; i < NCHANS; ++i) {
    CmdMoveChan *chan = cmd->chans + i;
    chan->steps = *((const int32_t*) (data + 4*(3*i + 0)));
    chan->bdelay = *((const uint32_t*) (data + 4*(3*i + 1)));
    chan->edelay = *((const uint32_t*) (data + 4*(3*i + 2)));
  }
}

void cmd_move_store(uint8_t *data, const CmdMove *cmd) {
  uint32_t i;
  for (i = 0; i < NCHANS; ++i) {
    const CmdMoveChan *chan = cmd->chans + i;
    *((int32_t*) (data + 4*(3*i + 0))) = chan->steps;
    *((uint32_t*) (data + 4*(3*i + 1))) = chan->bdelay;
    *((uint32_t*) (data + 4*(3*i + 2))) = chan->edelay;
  }
}
