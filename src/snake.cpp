/******************************************************************************
 * @file        snake.cpp
 * @brief       Easter egg Snake for TRMNL X. See snake.h for the public API.
 *
 * Rendering strategy (1872x1404 @ 1bpp):
 *   - 36 px cells -> exact 52 x 39 grid, no remainder.
 *   - Per tick only the new head, vacated tail, and (rarely) new food cell
 *     change, so each frame is a couple of fillRect() calls followed by a
 *     flicker-free bbep.partialUpdate(false).
 *   - A fullUpdate() is forced every FULL_REFRESH_EVERY partials to clear
 *     ghosting, plus on game start and game over.
 *
 * Input strategy:
 *   - snake_run() executes inside check_channel_states(), i.e. with the IQS323
 *     task's I2C lock held, and that task is what refreshes the chip's memory
 *     map. Every wait therefore releases the lock so the task can run, then
 *     retakes it and services RDY -- the same dance as tap_mode_is_hold().
 *     Edge detection on the CH0/CH2 touch states; taps are queued (depth 2)
 *     so a fast L-then-R double-turn between ticks isn't lost.
 ******************************************************************************/

// BOARD_TRMNL_X, not BOARD_X_CLASS: the touchbar (iqs323) only exists on the
// former, and the game is unplayable without it.
#ifdef BOARD_TRMNL_X

#include "snake.h"

#include <Arduino.h>
#include <Preferences.h>

#include "FastEPD.h"
#include "IQS323.h"
#include "Inter_18.h"
#include "api-client/submit_log.h"
#include "iqs323_task.h"
#include "trmnl_log.h"

extern FASTEPD bbep;    // defined in display.cpp
extern IQS323 iqs323;   // defined in bl.cpp
extern Preferences preferences;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
#define CELL_PX                      36
#define GRID_W                       (1872 / CELL_PX) // 52
#define GRID_H                       (1404 / CELL_PX) // 39

#define TICK_START_MS                110              // initial step interval
#define TICK_MIN_MS                  40               // fastest step interval
#define TICK_SPEEDUP_MS              12               // faster by this much per food eaten
#define FOOD_EDGE_MARGIN             1                // cells of buffer between food and the outer wall
#define INPUT_POLL_MS                15               // touch poll interval inside a tick
#define FULL_REFRESH_EVERY           128              // ghost-clearing cadence (in partial updates)
#define IDLE_TIMEOUT_MS              90000            // no input on game-over screen -> exit
#define EXIT_HOLD_MS                 2000             // hold middle this long during play to quit

#define MESSAGE_BOX_W                800              // STARTING / GAME OVER box
#define MESSAGE_BOX_H                260
#define MESSAGE_LINE_H               68
#define STARTING_SHOW_MS             900              // how long STARTING... stays up before the wipe

#define ENTRY_HOLD_MS                5000             // total middle-channel hold that launches the game
#define ENTRY_MISS_TOLERANCE         10               // consecutive non-touch polls that count as a lift
#define RELEASE_TIMEOUT_MS           3000             // give up waiting for that hold to lift

#define PREFERENCES_SNAKE_HS_KEY     "snake_hs"
#define PREFERENCES_SNAKE_REPORT_KEY "snake_report"   // game result awaiting upload

// ---------------------------------------------------------------------------
// Input polling
// ---------------------------------------------------------------------------
// One poll-interval wait. The memory map behind channel_touchState() is only
// refreshed by the IQS323 background task, and snake_run() holds that task's
// I2C lock -- so free the task for the wait, retake the lock, and service RDY
// if an event is pending. Reading the chip unconditionally instead wedges the
// bus (the task has check_i2c_lockup() for exactly that), which presents as
// all input going dead mid-game.
static void input_poll_delay(void) {
  iqs323_task_i2c_unlock();
  delay(INPUT_POLL_MS);
  iqs323_task_i2c_lock();
  if (iqs323.getRDYStatus()) {
    iqs323.updateInfoFlags(STOP);
  }
}

// ---------------------------------------------------------------------------
// Easter-egg entry: one long hold on the middle channel
// ---------------------------------------------------------------------------
bool snake_entry_hold_confirmed(uint32_t already_held_ms) {
  unsigned long start = millis();

  uint8_t consecutive_misses = 0;

  while (already_held_ms + (millis() - start) < ENTRY_HOLD_MS) {
    input_poll_delay();
    // A single dropped sample must not abort a 5 s hold -- that is ~200 polls.
    // Only a sustained miss counts as a lift.
    if (iqs323.channel_touchState(IQS323_CH1)) {
      consecutive_misses = 0;
    } else if (++consecutive_misses >= ENTRY_MISS_TOLERANCE) {
      return false;
    }
  }

  Log_info("Snake: middle held %d ms, launching", ENTRY_HOLD_MS);
  return true;
}

// ---------------------------------------------------------------------------
// Game state
// ---------------------------------------------------------------------------
typedef enum { DIR_UP = 0, DIR_RIGHT, DIR_DOWN, DIR_LEFT } dir_t;

typedef struct {
  uint8_t x, y;
} cell_t;

static cell_t body[GRID_W * GRID_H]; // ring buffer, head_idx grows forward
static uint16_t head_idx, tail_idx, snake_len;
static uint8_t occupied[(GRID_W * GRID_H + 7) / 8]; // collision bitmap
static dir_t heading;
static cell_t food;
static uint16_t score;
static uint16_t partials_since_full;

static inline uint16_t cell_bit(cell_t c) { return (uint16_t)c.y * GRID_W + c.x; }
static inline bool is_occupied(cell_t c) { return occupied[cell_bit(c) >> 3] & (1 << (cell_bit(c) & 7)); }
static inline void set_occupied(cell_t c, bool on) {
  if (on)
    occupied[cell_bit(c) >> 3] |= (1 << (cell_bit(c) & 7));
  else
    occupied[cell_bit(c) >> 3] &= ~(1 << (cell_bit(c) & 7));
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------
static void draw_cell(cell_t c, uint8_t color) {
  // 1 px inset keeps adjacent snake cells visually distinct on eink.
  bbep.fillRect(c.x * CELL_PX + 1, c.y * CELL_PX + 1, CELL_PX - 2, CELL_PX - 2, color);
}

static void draw_food(cell_t c) {
  // Hollow square so food reads differently from the snake at a glance.
  bbep.drawRect(c.x * CELL_PX + 6, c.y * CELL_PX + 6, CELL_PX - 12, CELL_PX - 12, BBEP_BLACK);
  bbep.drawRect(c.x * CELL_PX + 7, c.y * CELL_PX + 7, CELL_PX - 14, CELL_PX - 14, BBEP_BLACK);
}

static void flush_partial(void) {
  bbep.partialUpdate(false);
  if (++partials_since_full >= FULL_REFRESH_EVERY) {
    bbep.fullUpdate();
    partials_since_full = 0;
  }
}

// ---------------------------------------------------------------------------
// Input: edge-detected taps with a tiny queue
// ---------------------------------------------------------------------------
static int8_t tap_queue[2];
static uint8_t tap_queue_len;
static bool prev_touch[3];

// Read the touchbar state (refreshed by input_poll_delay); enqueue newly-
// pressed L/R, return true if the middle channel is currently held.
static bool poll_touch(void) {
  for (uint8_t ch = 0; ch < 3; ch++) {
    bool now = iqs323.channel_touchState((iqs323_channel_e)ch);
    bool pressed = now && !prev_touch[ch];
    prev_touch[ch] = now;

    if (pressed && ch != 1 && tap_queue_len < sizeof(tap_queue)) {
      tap_queue[tap_queue_len++] = (ch == 0) ? -1 : +1; // -1 turn CCW, +1 turn CW
    }
  }
  return prev_touch[1];
}

static int8_t dequeue_turn(void) {
  if (tap_queue_len == 0) {
    return 0;
  }
  int8_t t = tap_queue[0];
  tap_queue[0] = tap_queue[1];
  tap_queue_len--;
  return t;
}

// Wait for every channel to read untouched, then clear the edge-detector state.
// Without this the contact already in progress -- the launching hold, or the
// finger still on the bar at the moment of death -- lands in the next loop as
// fresh input. Bounded like the release wait in handle_confirmation_flow().
static void wait_for_release(void) {
  unsigned long start = millis();
  while (millis() - start < RELEASE_TIMEOUT_MS) {
    input_poll_delay();
    if (!iqs323.channel_touchState(IQS323_CH0) && !iqs323.channel_touchState(IQS323_CH1) &&
        !iqs323.channel_touchState(IQS323_CH2)) {
      break;
    }
  }
  memset(prev_touch, 0, sizeof(prev_touch));
  tap_queue_len = 0;
}

// ---------------------------------------------------------------------------
// Game logic
// ---------------------------------------------------------------------------
// Inset from the wall so the player is never forced to run along the very edge,
// where a single late turn is fatal. The wall itself still kills; only where
// food can appear is restricted.
static void place_food(void) {
  do {
    food.x = FOOD_EDGE_MARGIN + random(GRID_W - 2 * FOOD_EDGE_MARGIN);
    food.y = FOOD_EDGE_MARGIN + random(GRID_H - 2 * FOOD_EDGE_MARGIN);
  } while (is_occupied(food));
  draw_food(food);
}

static void reset_game(void) {
  memset(occupied, 0, sizeof(occupied));
  head_idx = 2;
  tail_idx = 0;
  snake_len = 3;
  heading = DIR_RIGHT;
  score = 0;
  tap_queue_len = 0;
  partials_since_full = 0;

  for (uint8_t i = 0; i < 3; i++) {
    body[i] = (cell_t){(uint8_t)(GRID_W / 2 - 1 + i), (uint8_t)(GRID_H / 2)};
    set_occupied(body[i], true);
  }

  bbep.fillScreen(BBEP_WHITE);
  for (uint8_t i = 0; i < 3; i++) {
    draw_cell(body[i], BBEP_BLACK);
  }
  randomSeed(esp_random());
  place_food();
  bbep.fullUpdate(CLEAR_NONE); // wipe_screen() just cleared; skip a second flash
}

// Advance one step. Returns false on death.
static bool step(void) {
  int8_t turn = dequeue_turn();
  if (turn != 0) {
    heading = (dir_t)(((int)heading + turn + 4) & 3);
  }

  cell_t head = body[head_idx % (GRID_W * GRID_H)];
  cell_t next = head;
  switch (heading) {
  case DIR_UP:
    next.y--;
    break;
  case DIR_RIGHT:
    next.x++;
    break;
  case DIR_DOWN:
    next.y++;
    break;
  case DIR_LEFT:
    next.x--;
    break;
  }

  // Walls are solid. (Wrap-around is a one-line change here if preferred.)
  if (next.x >= GRID_W || next.y >= GRID_H) { // uint8_t: -1 wraps to 255
    return false;
  }

  bool eating = (next.x == food.x && next.y == food.y);

  if (!eating) {
    cell_t tail = body[tail_idx % (GRID_W * GRID_H)];
    tail_idx++;
    set_occupied(tail, false);
    draw_cell(tail, BBEP_WHITE);
  }

  if (is_occupied(next)) { // self collision (tail already vacated above)
    return false;
  }

  head_idx++;
  body[head_idx % (GRID_W * GRID_H)] = next;
  set_occupied(next, true);
  draw_cell(next, BBEP_BLACK);

  if (eating) {
    score++;
    snake_len++;
    place_food();
  }

  flush_partial();
  return true;
}

// ---------------------------------------------------------------------------
// Score persistence + deferred /api/log upload
// ---------------------------------------------------------------------------
// No POST from here: the game runs before WiFi is up (bl.cpp reaches the
// touchbar handler ~line 952, WiFi connects ~1280). The result is stashed in
// NVS and uploaded by snake_submit_pending_score() on the next connected wake.
// A second game before that wake overwrites the stash -- last game wins; the
// all-time high survives in PREFERENCES_SNAKE_HS_KEY regardless.
static void report_score(uint32_t duration_ms) {
  uint16_t high = preferences.getUShort(PREFERENCES_SNAKE_HS_KEY, 0);
  bool new_high_score = score > high;
  if (new_high_score) {
    preferences.putUShort(PREFERENCES_SNAKE_HS_KEY, score);
    high = score;
  }

  // Spliced verbatim into {"logs":[...]} by serializeApiLogRequest, so it must
  // be a JSON object; "message" is the key the server's ignore-list filter
  // reads off each element.
  char report[192];
  snprintf(report, sizeof(report),
           "{\"message\":\"snake game over\",\"event\":\"snake_game_over\","
           "\"snake\":{\"score\":%u,\"high_score\":%u,\"new_high_score\":%s,\"duration_ms\":%lu}}",
           score, high, new_high_score ? "true" : "false", (unsigned long)duration_ms);
  preferences.putString(PREFERENCES_SNAKE_REPORT_KEY, report);

  Log_info("Snake: %s", report);
}

void snake_submit_pending_score(const char *api_key, const char *api_url) {
  if (!preferences.isKey(PREFERENCES_SNAKE_REPORT_KEY)) {
    return;
  }

  String payload = preferences.getString(PREFERENCES_SNAKE_REPORT_KEY, "");
  if (payload.isEmpty()) {
    preferences.remove(PREFERENCES_SNAKE_REPORT_KEY);
    return;
  }

  LogApiInput input{String(api_key), payload.c_str()};
  if (submitLogToApi(input, api_url)) {
    preferences.remove(PREFERENCES_SNAKE_REPORT_KEY);
    Log_info("Snake: pending score submitted");
  } // on failure the stash stays for the next connected wake
}

// Centre one line of Inter_18 inside the panel at the given baseline.
static void draw_centered(const char *text, int y) {
  BB_RECT rect;
  bbep.getStringBox(text, &rect);
  bbep.setCursor((bbep.width() - rect.w) / 2, y);
  bbep.println(text);
}

// Panel-centred box, drawn over whatever is already on screen. Shared by the
// STARTING and GAME OVER screens so both stay the same size and position.
static void draw_message_box(const char *const *lines, uint8_t count) {
  const int box_x = (GRID_W * CELL_PX - MESSAGE_BOX_W) / 2;
  const int box_y = (GRID_H * CELL_PX - MESSAGE_BOX_H) / 2;

  bbep.fillRect(box_x, box_y, MESSAGE_BOX_W, MESSAGE_BOX_H, BBEP_WHITE);
  bbep.drawRect(box_x, box_y, MESSAGE_BOX_W, MESSAGE_BOX_H, BBEP_BLACK);

  bbep.setFont(Inter_18);
  bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);

  int y = box_y + (MESSAGE_BOX_H - (count - 1) * MESSAGE_LINE_H) / 2 + 10;
  for (uint8_t i = 0; i < count; i++) {
    draw_centered(lines[i], y + i * MESSAGE_LINE_H);
  }
}

// CLEAR_SLOW is 10 passes of black/white; it flushes the plugin image that was
// on the panel before the game so it cannot ghost through the board.
static void wipe_screen(void) {
  bbep.fillScreen(BBEP_WHITE);
  bbep.fullUpdate(CLEAR_SLOW);
}

static void draw_game_over(void) {
  uint16_t high = preferences.getUShort(PREFERENCES_SNAKE_HS_KEY, 0);
  char score_line[64];
  snprintf(score_line, sizeof(score_line), "score %u    best %u", score, high);

  const char *lines[] = {"GAME OVER", score_line};
  draw_message_box(lines, 2);

  Log_info("Snake: game over, score %u, best %u", score, high);
  bbep.fullUpdate();
}

// ---------------------------------------------------------------------------
// Public: blocking game loop
// ---------------------------------------------------------------------------
void snake_run(void) {
  Log_info("Snake: starting (grid %dx%d, cell %dpx)", GRID_W, GRID_H, CELL_PX);

  // The launching hold is still down; without draining it the game loop would
  // read it as the middle-hold exit and quit immediately.
  wait_for_release();

  bbep.setMode(BB_MODE_1BPP);

  // Confirm the gesture landed before the wipe, which takes a visible moment.
  const char *starting[] = {"STARTING GAME..."};
  draw_message_box(starting, 1);
  bbep.fullUpdate(CLEAR_NONE);
  delay(STARTING_SHOW_MS);

  wipe_screen();
  reset_game();
  uint32_t tick_ms = TICK_START_MS;
  unsigned long middle_hold_start = 0;
  unsigned long game_start = millis();
  bool alive = true;

  while (alive) {
    // Poll input for one tick's duration.
    unsigned long tick_start = millis();
    while (millis() - tick_start < tick_ms) {
      input_poll_delay();
      bool middle_held = poll_touch();

      if (middle_held) {
        if (middle_hold_start == 0) {
          middle_hold_start = millis();
        } else if (millis() - middle_hold_start >= EXIT_HOLD_MS) {
          Log_info("Snake: exit via middle hold");
          report_score(millis() - game_start);
          return;
        }
      } else {
        middle_hold_start = 0;
      }
    }

    alive = step();
    // Clamp the speedup, not the result: the subtraction is unsigned, so past
    // score 63 it would wrap to ~50 days instead of bottoming out at TICK_MIN_MS.
    tick_ms = TICK_START_MS - min((uint32_t)score * TICK_SPEEDUP_MS, (uint32_t)(TICK_START_MS - TICK_MIN_MS));
  }

  report_score(millis() - game_start);
  draw_game_over();

  // Game over: any touch (or the idle timeout) returns to the caller, which
  // redraws the last shown screen and sleeps. No replay UI.
  wait_for_release();
  unsigned long over_start = millis();
  while (millis() - over_start < IDLE_TIMEOUT_MS) {
    input_poll_delay();
    if (iqs323.channel_touchState(IQS323_CH0) || iqs323.channel_touchState(IQS323_CH1) ||
        iqs323.channel_touchState(IQS323_CH2)) {
      Log_info("Snake: game over dismissed");
      return;
    }
  }
  Log_info("Snake: idle timeout on game over screen");
}

#endif // BOARD_TRMNL_X
