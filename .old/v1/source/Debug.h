#define printf(...) uBit.serial.send(__VA_ARGS__)

static void on_command_receive(MicroBitEvent);
void debug_fiber();