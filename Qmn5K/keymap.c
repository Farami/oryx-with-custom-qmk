#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  ST_MACRO_0,
  MAC_DND,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    ST_MACRO_0,     KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_BSLS,        
    KC_LEFT_SHIFT,  KC_Q,           MT(MOD_RALT, KC_W),KC_F,           KC_P,           KC_B,                                           KC_J,           KC_L,           KC_U,           MT(MOD_RALT, KC_Y),KC_SCLN,        KC_MINUS,       
    KC_ESCAPE,      MT(MOD_LCTL, KC_A),MT(MOD_LALT, KC_R),MT(MOD_LGUI, KC_S),MT(MOD_LSFT, KC_T),KC_G,                                           KC_M,           MT(MOD_RSFT, KC_N),MT(MOD_RGUI, KC_E),MT(MOD_LALT, KC_I),MT(MOD_RCTL, KC_O),KC_QUOTE,       
    CW_TOGG,        KC_Z,           MEH_T(KC_X),    ALL_T(KC_C),    LT(6,KC_D),     KC_V,                                           KC_K,           KC_H,           ALL_T(KC_COMMA),MEH_T(KC_DOT),  KC_SLASH,       TO(5),          
                                                    LT(1,KC_BSPC),  LT(2,KC_TAB),                                   LT(4,KC_ENTER), LT(3,KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_NO,          KC_MEDIA_PREV_TRACK,KC_MEDIA_STOP,  KC_MEDIA_PLAY_PAUSE,KC_MEDIA_NEXT_TRACK,KC_NO,                                          MAC_DND,        KC_AUDIO_MUTE,  KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_NO,          KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          LGUI(KC_TAB),   LCTL(KC_TAB),   LGUI(KC_X),                                     LGUI(LSFT(KC_D)),LGUI(KC_LBRC),  LGUI(LSFT(KC_ENTER)),LGUI(KC_T),     LGUI(KC_RBRC),  KC_NO,          
    KC_NO,          OSM(MOD_LCTL),  OSM(MOD_LALT),  OSM(MOD_LGUI),  OSM(MOD_LSFT),  LGUI(KC_C),                                     LGUI(KC_D),     KC_LEFT,        KC_UP,          KC_DOWN,        KC_RIGHT,       KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          LALT(LCTL(KC_LEFT_GUI)),QK_LLCK,        TD(DANCE_0),                                    LGUI(KC_W),     KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_NO,          
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_DELETE
  ),
  [2] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        KC_CIRC,        KC_DLR,         KC_HASH,        KC_AT,          KC_EXLM,        KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          LCTL(KC_X),                                     KC_MINUS,       KC_7,           KC_8,           KC_9,           KC_EQUAL,       KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          LCTL(KC_C),                                     KC_PLUS,        KC_4,           KC_5,           KC_6,           KC_ASTR,        KC_PERC,        
    KC_NO,          RGB_HUD,        RGB_HUI,        RGB_SAD,        RGB_SAI,        LCTL(KC_V),                                     KC_DOT,         KC_1,           KC_2,           KC_3,           KC_SLASH,       KC_NO,          
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_COMMA,       KC_0
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_EXLM,        KC_LCBR,        KC_QUOTE,       KC_DQUO,        KC_RCBR,        KC_QUES,                                        KC_PERC,        KC_LBRC,        KC_LPRN,        KC_RPRN,        KC_RBRC,        KC_GRAVE,       
    KC_HASH,        KC_CIRC,        KC_EQUAL,       KC_UNDS,        KC_DLR,         KC_ASTR,                                        KC_PLUS,        KC_BSPC,        KC_TAB,         KC_SPACE,       KC_ENTER,       KC_AMPR,        
    KC_TILD,        KC_LABK,        KC_PIPE,        KC_MINUS,       KC_RABK,        KC_SLASH,                                       KC_BSLS,        KC_AT,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_SCLN,        KC_COLN,                                        KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          
    KC_NO,          KC_MS_WH_LEFT,  KC_MS_WH_DOWN,  KC_MS_UP,       KC_MS_WH_UP,    KC_MS_WH_RIGHT,                                 KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          
    KC_NO,          LGUI(KC_LEFT),  KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    LGUI(KC_RIGHT),                                 KC_NO,          OSM(MOD_LSFT),  OSM(MOD_RCTL),  OSM(MOD_RALT),  OSM(MOD_RGUI),  KC_NO,          
    KC_NO,          TD(DANCE_1),    KC_NO,          KC_NO,          KC_NO,          KC_MS_BTN3,                                     KC_NO,          QK_LLCK,        LGUI(LCTL(KC_4)),KC_NO,          KC_NO,          KC_NO,          
                                                    KC_MS_BTN1,     KC_MS_BTN2,                                     KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [5] = LAYOUT_voyager(
    LT(6,KC_ESCAPE),KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_BSPC,        
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_TRANSPARENT, 
    KC_LEFT_CTRL,   KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                           KC_H,           KC_J,           KC_K,           KC_L,           KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_LEFT_SHIFT,  KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(0),          
                                                    KC_SPACE,       KC_LEFT_ALT,                                    KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [6] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_STOP,  KC_MEDIA_PLAY_PAUSE,KC_MEDIA_NEXT_TRACK,KC_NO,                                          KC_NO,          KC_AUDIO_MUTE,  KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_NO,          KC_NO,          
    KC_NO,          QK_BOOT,        KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_NO,          
    KC_NO,          OSM(MOD_LGUI),  OSM(MOD_LALT),  OSM(MOD_LCTL),  OSM(MOD_LSFT),  KC_NO,                                          KC_NO,          KC_F4,          KC_F5,          KC_F6,          KC_F11,         KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_TRANSPARENT, KC_NO,                                          KC_NO,          KC_F1,          KC_F2,          KC_F3,          KC_F12,         KC_NO,          
                                                    KC_NO,          KC_NO,                                          KC_NO,          KC_NO
  ),
};

const uint16_t PROGMEM combo0[] = { KC_F, KC_G, COMBO_END};
const uint16_t PROGMEM combo1[] = { LT(2,KC_TAB), LT(4,KC_ENTER), COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_ESCAPE),
    COMBO(combo1, OSL(6)),
};

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MT(MOD_LSFT, KC_T):
            return TAPPING_TERM -25;
        case LT(1,KC_BSPC):
            return TAPPING_TERM -25;
        case LT(2,KC_TAB):
            return TAPPING_TERM -25;
        case MT(MOD_RSFT, KC_N):
            return TAPPING_TERM -25;
        case LT(4,KC_ENTER):
            return TAPPING_TERM -25;
        case LT(3,KC_SPACE):
            return TAPPING_TERM -25;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [1] = { {0,0,0}, {191,125,231}, {191,125,231}, {191,125,231}, {191,125,231}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,195,255}, {25,195,255}, {245,255,255}, {0,0,0}, {245,195,249}, {25,195,255}, {135,151,220}, {191,125,231}, {245,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {175,22,244}, {245,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {25,195,255}, {25,195,255}, {25,195,255}, {25,195,255}, {25,195,255}, {0,0,0}, {25,195,255}, {25,195,255}, {25,195,255}, {25,195,255}, {25,195,255}, {0,0,0}, {0,0,0}, {25,195,255}, {25,195,255}, {25,195,255}, {25,195,255}, {0,0,0}, {0,0,0}, {0,0,0} },

    [2] = { {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {245,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {245,255,255}, {0,0,0}, {0,0,0}, {189,255,238}, {189,255,238}, {189,255,238}, {245,255,255}, {0,0,0}, {0,0,0}, {128,222,183}, {128,222,183}, {128,222,183}, {128,222,183}, {128,222,183}, {0,0,0}, {128,222,183}, {245,195,249}, {245,195,249}, {245,195,249}, {128,222,183}, {0,0,0}, {128,222,183}, {245,195,249}, {245,195,249}, {245,195,249}, {128,222,183}, {128,222,183}, {128,222,183}, {245,195,249}, {245,195,249}, {245,195,249}, {128,222,183}, {0,0,0}, {128,222,183}, {245,195,249} },

    [3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {135,151,220}, {245,255,255}, {245,255,255}, {245,255,255}, {245,255,255}, {135,151,220}, {135,151,220}, {135,151,220}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {191,125,231}, {191,125,231}, {191,125,231}, {191,125,231}, {191,125,231}, {0,0,0}, {191,125,231}, {191,125,231}, {191,125,231}, {191,125,231}, {191,125,231}, {0,0,0}, {191,125,231}, {0,0,0}, {0,0,0}, {0,0,0}, {191,125,231}, {191,125,231}, {191,125,231}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {245,195,249}, {25,195,255}, {135,151,220}, {191,125,231}, {0,0,0}, {0,0,0}, {175,22,244}, {191,125,231}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [5] = { {245,255,255}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {128,222,183}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {128,222,183}, {128,222,183}, {128,222,183}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {245,255,255}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {189,255,238}, {245,255,255}, {0,0,0}, {0,0,0} },

    [6] = { {0,0,0}, {191,125,231}, {191,125,231}, {191,125,231}, {191,125,231}, {0,0,0}, {0,0,0}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {191,125,231}, {135,151,220}, {25,195,255}, {245,195,249}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {245,195,249}, {25,195,255}, {135,151,220}, {191,125,231}, {0,0,0}, {0,0,0}, {245,195,249}, {25,195,255}, {135,151,220}, {191,125,231}, {0,0,0}, {0,0,0}, {245,195,249}, {25,195,255}, {135,151,220}, {191,125,231}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
      break;
    case 5:
      set_layer_color(5);
      break;
    case 6:
      set_layer_color(6);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LGUI(SS_TAP(X_1)));
    }
    break;
    case MAC_DND:
      HSS(0x9B);

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[2];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(KC_V));
        tap_code16(LGUI(KC_V));
        tap_code16(LGUI(KC_V));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_V));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LGUI(KC_V)); break;
        case SINGLE_HOLD: register_code16(LGUI(LSFT(KC_V))); break;
        case DOUBLE_TAP: register_code16(LGUI(KC_V)); register_code16(LGUI(KC_V)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_V)); register_code16(LGUI(KC_V));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_V)); break;
        case SINGLE_HOLD: unregister_code16(LGUI(LSFT(KC_V))); break;
        case DOUBLE_TAP: unregister_code16(LGUI(KC_V)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_V)); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(KC_Z));
        tap_code16(LGUI(KC_Z));
        tap_code16(LGUI(KC_Z));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_Z));
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(LGUI(KC_Z)); break;
        case SINGLE_HOLD: register_code16(LGUI(LSFT(KC_Z))); break;
        case DOUBLE_TAP: register_code16(LGUI(KC_Z)); register_code16(LGUI(KC_Z)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_Z)); register_code16(LGUI(KC_Z));
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_Z)); break;
        case SINGLE_HOLD: unregister_code16(LGUI(LSFT(KC_Z))); break;
        case DOUBLE_TAP: unregister_code16(LGUI(KC_Z)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_Z)); break;
    }
    dance_state[1].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
};
