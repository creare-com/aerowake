#include <stdint.h>

enum State_e {
    STATE_NO_DOT_FOUND = 0,
    STATE_DOT_FOUND = 1,
};

typedef struct {
    uint32_t State; // Must be a value from State_e
    float DotX; // Laser dot location, horizontal axis. 0 for the left edge of the screen, 1 for the right edge
    float DotY; // Laser dot location, vertical axis. 0 for the top edge of the screen, 1 for the bottom edge
} UpdatePacket_s;
