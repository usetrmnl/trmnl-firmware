#include "touchbar_actions.h"

#ifdef BOARD_TRMNL_X

#include <display.h>
#include <trmnl_log.h>

#include "IQS323.h"
#include "cached_images.h"
#include "iqs323_task.h"
#include "menu.h"
#include "touchbar_gesture.h"

// Touchbar indicator to redraw after a full-refresh (e.g. logo screen)
static touchbar_side_t pending_indicator_side = TOUCHBAR_LEFT;
static bool pending_indicator_filled = false;
static bool has_pending_indicator = false;

// The touch bar's control scheme for the menu (menu.h): left is up, right is down, the middle selects

static const menu_hint_t touchbar_menu_hints[MENU_HINT_SLOTS] = {
    {"Up", MENU_HINT_ICON_UP},
    {"Select", MENU_HINT_ICON_NONE},
    {"Down", MENU_HINT_ICON_DOWN},
};

// Touch starts not reported yet (bit n: channel n). The menu reads the bar between display
// updates, so it goes by touch starts: a tap during an update still counts.
static uint8_t touchbar_menu_pending_starts = 0;

static void touchbar_menu_begin(void) {
  iqs323_task_set_streaming_mode(true); // fresh touch data while the menu polls
  iqs323_task_take_touch_starts();      // the middle tap that opened the menu
  touchbar_menu_pending_starts = 0;
}

static void touchbar_menu_end(void) { iqs323_task_set_streaming_mode(false); }

static menu_input_t touchbar_menu_read(void) {
  touchbar_menu_pending_starts |= iqs323_task_take_touch_starts();
  for (uint8_t channel = 0; channel < 3; channel++) {
    if (!(touchbar_menu_pending_starts & (1 << channel))) continue;
    touchbar_menu_pending_starts &= ~(1 << channel);
    switch (channel) {
    case 0:
      return menu_input_t::UP;
    case 2:
      return menu_input_t::DOWN;
    default:
      return menu_input_t::SELECT;
    }
  }
  return menu_input_t::NONE;
}

static const menu_controls_t touchbar_menu_controls = {
    touchbar_menu_hints,
    touchbar_menu_begin,
    touchbar_menu_end,
    touchbar_menu_read,
};

// Single dispatch point for both modes' resolved navigation intent.
static void dispatch_touchbar_intent(const touchbar_resolution_t &resolved) {
  switch (resolved.intent) {
  case touchbar_intent_t::BACK:
    display_draw_touchbar_indicator(TOUCHBAR_LEFT, resolved.held);
    Log_info(resolved.held ? "Back button hold" : "Back button tapped");
    show_cached_image_by_offset(-1);
    break;
  case touchbar_intent_t::NEXT:
    display_draw_touchbar_indicator(TOUCHBAR_RIGHT, resolved.held);
    Log_info(resolved.held ? "Next button hold" : "Next button tapped");
    show_cached_image_by_offset(+1);
    break;
  case touchbar_intent_t::MIDDLE:
    if (!resolved.held) {
      Log_info("Middle button tapped - opening the menu");
      menu_open(touchbar_menu_controls);
      break;
    }
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, true);
    Log_info("Middle button hold");
    pending_indicator_side = TOUCHBAR_MIDDLE;
    pending_indicator_filled = true;
    has_pending_indicator = true;
      // TODO: CH1 hold was previously reserved for an OTG toggle feature -
      // not implemented yet. See otg_state/otg_turn_on()/otg_turn_off().
    break;
  case touchbar_intent_t::NONE:
  default:
    break;
  }
}

void process_iqs323_data(void) { dispatch_touchbar_intent(touchbar_resolve_intent()); }

void touchbar_redraw_pending_indicator(void) {
  if (has_pending_indicator) {
    display_draw_touchbar_indicator(pending_indicator_side, pending_indicator_filled);
    has_pending_indicator = false;
  }
}

#endif // BOARD_TRMNL_X
