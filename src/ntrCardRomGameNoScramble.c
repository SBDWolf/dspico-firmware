#include "common.h"
#include <stdio.h>
#include "ntrCardRom.h"
#include "powerSaving.h"
#include "ntrCardRomGameNoScramble.h"

u32 gCardResyncCount;

void ntrc_gameNoScrambleCmd1Unknown(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_noPayload(pio);
    ntrc_finishGameNoScrambleCmd1(romEmu);

    // DSpico: a desynced card stream (a soft reset cutting a transaction
    // mid-flight, then garbage data words landing in command phase) shows
    // up here as an unknown command. Upstream disabled the card IRQ until
    // the next card reset — but a PMDSky soft reset never toggles PIN_RST,
    // so that meant the card bus stayed dead forever (game freeze). The
    // transaction above is completed cleanly (no payload, wordIdx reset),
    // so just clear any stale payload state, count the resync, and keep
    // going: the game's next command re-syncs the bus. (No puts/printf
    // here: this runs in the PIO IRQ, and a ~1 ms UART write lets the
    // PIO RX FIFO overflow and desync the stream further.)
    romEmu->readDataDestination = 0;
    romEmu->readDataLimit = 0;
    romEmu->readDataCompleteHandler = 0;
    gCardResyncCount++;
}

void __time_critical_func(ntrc_gameNoScrambleCmd0Dummy)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

void __time_critical_func(ntrc_gameNoScrambleCmd1Dummy0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

void __time_critical_func(ntrc_gameNoScrambleCmd1Dummy4)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

void __scratch_y("cpu0") ntrc_gameNoScrambleReadIdCmd0(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 4);
    ntrc_writeWord(pio, romEmu->cardId);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern void ntrc_gameNoScrambleCmd0Handler(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio);
extern void ntrc_gameNoScrambleCmd1Handler(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio);

void __time_critical_func(ntrc_setGameNoScrambleMode)(void)
{
    gNtrRomEmu.cmd0Handler = ntrc_gameNoScrambleCmd0Handler;
    gNtrRomEmu.cmd1Handler = ntrc_gameNoScrambleCmd1Handler;
    gNtrRomEmu.mode = NTR_CARD_MODE_GAME_NO_SCRAMBLE;
    pwr_enableAfterBootPowerSaving();
}
