/**
 * wii_game_hashes.h
 *
 * Wii game ID -> RetroAchievements MD5 hash lookup table.
 *
 * Game ID comes from the Wii disc header at 0x80000000 (6 bytes, e.g. "RSBE01").
 * The MD5 hash is what RetroAchievements expects in rc_client_begin_load_game().
 *
 * For Wii games the hash is computed by rcheevos from the disc image
 * (rhash/hash.c, RC_CONSOLE_WII). Because hashing a full ISO is impractical
 * on the ESP32, this table provides a pre-computed map keyed by disc ID.
 *
 * How to add a game:
 *   1. Compute the hash: `retroarch --hash /path/to/game.iso --libretro ...`
 *      or check https://retroachievements.org (game info → MD5 column).
 *   2. Add an entry below: { "GAMEID", "md5hash32chars", "Human Name" }
 */

#ifndef WII_GAME_HASHES_H
#define WII_GAME_HASHES_H

#include <stdint.h>
#include <string.h>

struct WiiGameHashEntry {
    const char game_id[7];   /* 6-char Wii disc ID + null */
    const char md5_hash[33]; /* 32 hex chars + null */
    const char name[64];     /* human-readable name (for debug log) */
};

/* --- Add your Wii games here -------------------------------------------- */
static const WiiGameHashEntry WII_GAME_HASH_TABLE[] = {
    /* Wii Sports (USA) — disc ID RSPE01 */
    /* Hash: obtain from RA game page or hash tool */
    { "RSPE01", "00000000000000000000000000000000", "Wii Sports (USA)" },

    /* Super Mario Galaxy (USA) */
    { "RMGE01", "4e0d0d2f2c5d3c13d758b027bbcc059f", "Super Mario Galaxy (USA)" },

    /* The Legend of Zelda: Twilight Princess (USA, Wii) */
    { "RZDE01", "00000000000000000000000000000000", "Zelda: Twilight Princess (USA)" },

    /* Super Smash Bros. Brawl (USA) */
    { "RSBE01", "00000000000000000000000000000000", "Super Smash Bros. Brawl (USA)" },

    /* Kirby's Return to Dream Land (USA) — disc ID best-guess SUKE01.
     * If LOAD_GAME logs a different id (SUKE01 not found), update this entry. */
    { "SUKE01", "ad50325115bee56a6ec875fed32aa711", "Kirby's Return to Dream Land (USA)" },
};
/* ------------------------------------------------------------------------ */

static const int WII_GAME_HASH_TABLE_SIZE =
    sizeof(WII_GAME_HASH_TABLE) / sizeof(WiiGameHashEntry);

/** Look up the MD5 hash for a given Wii disc ID.
 *  Returns pointer to hash string, or NULL if not in table. */
static inline const char* wii_lookup_game_hash(const char *game_id) {
    for (int i = 0; i < WII_GAME_HASH_TABLE_SIZE; i++) {
        if (strncmp(game_id, WII_GAME_HASH_TABLE[i].game_id, 6) == 0)
            return WII_GAME_HASH_TABLE[i].md5_hash;
    }
    return NULL;
}

/** Look up the human-readable name for debug output.
 *  Returns "Unknown Wii Game" if not found. */
static inline const char* wii_lookup_game_name(const char *game_id) {
    for (int i = 0; i < WII_GAME_HASH_TABLE_SIZE; i++) {
        if (strncmp(game_id, WII_GAME_HASH_TABLE[i].game_id, 6) == 0)
            return WII_GAME_HASH_TABLE[i].name;
    }
    return "Unknown Wii Game";
}

#endif /* WII_GAME_HASHES_H */
