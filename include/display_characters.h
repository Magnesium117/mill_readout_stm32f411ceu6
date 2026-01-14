/*
 * Bit Number to segment
 *
 *     0
 *    ---
 * 5 | 6 | 1
 *    ---
 * 4 |   | 2
 *    ---    . 7
 *     3
 */
#define SET_SEGMENT(i) (1 << i)
#define DISPLAY_CHARACTER_A                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(1) | SET_SEGMENT(2) | SET_SEGMENT(4) |          \
      SET_SEGMENT(5) | SET_SEGMENT(6)
#define DISPLAY_CHARACTER_b                                                    \
  SET_SEGMENT(2) | SET_SEGMENT(3) | SET_SEGMENT(4) | SET_SEGMENT(5) |          \
      SET_SEGMENT(6)
#define DISPLAY_CHARACTER_C                                                    \
  #define DISPLAY_CHARACTER_d SET_SEGMENT(2) | SET_SEGMENT(3) |                \
      SET_SEGMENT(4) | SET_SEGMENT(5) | SET_SEGMENT(6)
#define DISPLAY_CHARACTER_E                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(3) | SET_SEGMENT(4) | SET_SEGMENT(5) |          \
      SET_SEGMENT(6)
#define DISPLAY_CHARACTER_F                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(4) | SET_SEGMENT(5) | SET_SEGMENT(6)
#define DISPLAY_CHARACTER_G                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(2) | SET_SEGMENT(3) | SET_SEGMENT(4) |          \
      SET_SEGMENT(5)
#define DISPLAY_CHARACTER_H                                                    \
  SET_SEGMENT(1) | SET_SEGMENT(2) | SET_SEGMENT(4) | SET_SEGMENT(5) |          \
      SET_SEGMENT(6)
#define DISPLAY_CHARACTER_I (SET_SEGMENT(4) | SET_SEGMENT(5))
#define DISPLAY_CHARACTER_J                                                    \
  SET_SEGMENT(1) | SET_SEGMENT(2) | SET_SEGMENT(3) | SET_SEGMENT(4)
#define DISPLAY_CHARACTER_L SET_SEGMENT(3) | SET_SEGMENT(4) | SET_SEGMENT(5)
#define DISPLAY_CHARACTER_n SET_SEGMENT(2) | SET_SEGMENT(4) | SET_SEGMENT(6)
#define DISPLAY_CHARACTER_N                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(1) | SET_SEGMENT(2) | SET_SEGMENT(4) |          \
      SET_SEGMENT(5)
#define DISPLAY_CHARACTER_o                                                    \
  SET_SEGMENT(2) | SET_SEGMENT(3) | SET_SEGMENT(4) | SET_SEGMENT(6)
#define DISPLAY_CHARACTER_O                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(1) | SET_SEGMENT(2) | SET_SEGMENT(3) |          \
      SET_SEGMENT(4) | SET_SEGMENT(5)

#define DISPLAY_CHARACTER_P                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(1) | SET_SEGMENT(4) | SET_SEGMENT(5) |          \
      SET_SEGMENT(6)
#define DISPLAY_CHARACTER_r SET_SEGMENT(4) | SET_SEGMENT(6)
#define DISPLAY_CHARACTER_S                                                    \
  SET_SEGMENT(0) | SET_SEGMENT(2) | SET_SEGMENT(3) | SET_SEGMENT(5) |          \
      SET_SEGMENT(6)
#define DISPLAY_CHARACTER_U                                                    \
  SET_SEGMENT(1) | SET_SEGMENT(2) | SET_SEGMENT(3) | SET_SEGMENT(4) |          \
      SET_SEGMENT(5)
#define DISPLAY_CHARACTER_Y                                                    \
  SET_SEGMENT(1) | SET_SEGMENT(4) | SET_SEGMENT(5) | SET_SEGMENT(6)
