#pragma once

enum SYSTEM_STATES : uint8_t {
    SS_MAIN_MENU,
    SS_TEST
};

SYSTEM_STATES system_state = SS_MAIN_MENU;
uint8_t system_state_last = 0xFF;

bool isSSChange() {
    bool changed = (system_state != system_state_last);
    system_state_last = system_state;
    return changed;
}