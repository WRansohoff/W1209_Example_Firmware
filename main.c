#include <stdint.h>

#define F_CPU     2000000UL

// Register definitions.
#define GPIOA_ODR *(volatile uint8_t *)(0x5000)
#define GPIOA_DDR *(volatile uint8_t *)(0x5002)
#define GPIOA_CR1 *(volatile uint8_t *)(0x5003)
#define GPIOB_ODR *(volatile uint8_t *)(0x5005)
#define GPIOB_DDR *(volatile uint8_t *)(0x5007)
#define GPIOB_CR1 *(volatile uint8_t *)(0x5008)
#define GPIOC_ODR *(volatile uint8_t *)(0x500A)
#define GPIOC_IDR *(volatile uint8_t *)(0x500B)
#define GPIOC_DDR *(volatile uint8_t *)(0x500C)
#define GPIOC_CR1 *(volatile uint8_t *)(0x500D)
#define GPIOD_ODR *(volatile uint8_t *)(0x500F)
#define GPIOD_IDR *(volatile uint8_t *)(0x5010)
#define GPIOD_DDR *(volatile uint8_t *)(0x5011)
#define GPIOD_CR1 *(volatile uint8_t *)(0x5012)
#define GPIOA_BASE      (0x5000)
#define GPIOB_BASE      (0x5005)
#define GPIOC_BASE      (0x500A)
#define GPIOD_BASE      (0x500F)
#define GPIO_ODR        (0x0000)
#define GPIO_IDR        (0x0001)
#define GPIO_DDR        (0x0002)
#define GPIO_CR1        (0x0003)
#define GPIOx_REG(addr, offset) (*(volatile uint8_t *)(addr + offset))

// Pin mappings.
// (Output)
#define RELAY_PA    (3)
#define SEG_D1_PD   (4)
#define SEG_D2_PB   (5)
#define SEG_D3_PB   (4)
#define SEG_LA_PD   (5)
#define SEG_LB_PA   (1)
#define SEG_LC_PC   (7)
#define SEG_LD_PD   (3)
#define SEG_LE_PD   (1)
#define SEG_LF_PA   (2)
#define SEG_LG_PC   (6)
#define SEG_DP_PD   (2)
// (Input)
#define KEY_SET_PC  (3)
#define KEY_PLS_PC  (4)
#define KEY_MIN_PC  (5)
#define RES_IN_PD   (6)

// Port/pin macro mappings.
#define RELAY_PIN    GPIOA_BASE, RELAY_PA
#define SEG_D1_PIN   GPIOD_BASE, SEG_D1_PD
#define SEG_D2_PIN   GPIOB_BASE, SEG_D2_PB
#define SEG_D3_PIN   GPIOB_BASE, SEG_D3_PB
#define SEG_LA_PIN   GPIOD_BASE, SEG_LA_PD
#define SEG_LB_PIN   GPIOA_BASE, SEG_LB_PA
#define SEG_LC_PIN   GPIOC_BASE, SEG_LC_PC
#define SEG_LD_PIN   GPIOD_BASE, SEG_LD_PD
#define SEG_LE_PIN   GPIOD_BASE, SEG_LE_PD
#define SEG_LF_PIN   GPIOA_BASE, SEG_LF_PA
#define SEG_LG_PIN   GPIOC_BASE, SEG_LG_PC
#define SEG_DP_PIN   GPIOD_BASE, SEG_DP_PD

// Global constants.
#define RELAY_MS    2000

uint32_t delay_i;

static inline void GPIOx_PP_ON(uint16_t gpiox, uint8_t pin) {
  GPIOx_REG(gpiox, GPIO_ODR) |= (1 << pin);
}
static inline void GPIOx_PP_OFF(uint16_t gpiox, uint8_t pin) {
  GPIOx_REG(gpiox, GPIO_ODR) &= ~(1 << pin);
}
static inline void GPIOx_PP_TOGGLE(uint16_t gpiox, uint8_t pin) {
  GPIOx_REG(gpiox, GPIO_ODR) ^= (1 << pin);
}

static inline void set_gpio_pp(uint16_t gpio_bank, uint8_t pin) {
  // Set pin to output mode.
  GPIOx_REG(gpio_bank, GPIO_DDR) |= (1 << pin);
  // Set pin to push-pull mode.
  GPIOx_REG(gpio_bank, GPIO_CR1) |= (1 << pin);
}

static inline void set_gpio_in(uint16_t gpio_bank,
                               uint8_t pin,
                               volatile uint8_t pullup_en) {
  // Set pin to input mode.
  GPIOx_REG(gpio_bank, GPIO_DDR) &= ~(1 << pin);
  // Enable internal pull-up resistor if necessary.
  if (pullup_en) {
    GPIOx_REG(gpio_bank, GPIO_CR1) |=  (1 << pin);
  }
  else {
    GPIOx_REG(gpio_bank, GPIO_CR1) &= ~(1 << pin);
  }
}

void draw_7s_digit(volatile char c,
                   volatile int dig,
                   volatile uint8_t with_dp) {
  uint32_t delay_i;
  // Set which digit to draw to.
  // For now, only one at a time. Default to 'all off.'
  if (dig == 0) {
      GPIOx_PP_OFF(SEG_D1_PIN);
      GPIOx_PP_ON(SEG_D2_PIN);
      GPIOx_PP_ON(SEG_D3_PIN);
  }
  else if (dig == 1) {
      GPIOx_PP_ON(SEG_D1_PIN);
      GPIOx_PP_OFF(SEG_D2_PIN);
      GPIOx_PP_ON(SEG_D3_PIN);
  }
  else if (dig == 2) {
      GPIOx_PP_ON(SEG_D1_PIN);
      GPIOx_PP_ON(SEG_D2_PIN);
      GPIOx_PP_OFF(SEG_D3_PIN);
  }
  else {
      GPIOx_PP_ON(SEG_D1_PIN);
      GPIOx_PP_ON(SEG_D2_PIN);
      GPIOx_PP_ON(SEG_D3_PIN);
  }

  // Draw the actual 7-segment digit.
  // LA: Top
  // LB: Top-Left
  // LC: Bottom-Right
  // LD: Bottom
  // LE: Bottom-Left
  // LF: Top-Right
  // LG: Middle
  if (c == '0') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_ON(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_OFF(SEG_LG_PIN);
  }
  if (c == '1') {
      GPIOx_PP_OFF(SEG_LA_PIN);
      GPIOx_PP_OFF(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_OFF(SEG_LD_PIN);
      GPIOx_PP_OFF(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_OFF(SEG_LG_PIN);
  }
  if (c == '2' || c == 'Z') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_OFF(SEG_LB_PIN);
      GPIOx_PP_OFF(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_ON(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  if (c == '3') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_OFF(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_OFF(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  if (c == '4') {
      GPIOx_PP_OFF(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_OFF(SEG_LD_PIN);
      GPIOx_PP_OFF(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  if (c == '5' || c == 'S') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_OFF(SEG_LE_PIN);
      GPIOx_PP_OFF(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  if (c == '6') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_ON(SEG_LE_PIN);
      GPIOx_PP_OFF(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  if (c == '7') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_OFF(SEG_LD_PIN);
      GPIOx_PP_OFF(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_OFF(SEG_LG_PIN);
  }
  else if (c == '8') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_ON(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  if (c == '9') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_OFF(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_ON(SEG_LG_PIN);
  }
  else if (c == 'V' || c == 'U') {
      GPIOx_PP_OFF(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_ON(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_ON(SEG_LE_PIN);
      GPIOx_PP_ON(SEG_LF_PIN);
      GPIOx_PP_OFF(SEG_LG_PIN);
  }
  else if (c == 'C') {
      GPIOx_PP_ON(SEG_LA_PIN);
      GPIOx_PP_ON(SEG_LB_PIN);
      GPIOx_PP_OFF(SEG_LC_PIN);
      GPIOx_PP_ON(SEG_LD_PIN);
      GPIOx_PP_ON(SEG_LE_PIN);
      GPIOx_PP_OFF(SEG_LF_PIN);
      GPIOx_PP_OFF(SEG_LG_PIN);
  }
  else if (c == 'c') {
  }

  // Draw the decimal point if appropriate.
  if (with_dp != 0) {
    GPIOx_PP_ON(SEG_DP_PIN);
  }
  else {
    GPIOx_PP_OFF(SEG_DP_PIN);
  }

  // Very brief delay to allow digit to 'set'.
  for (delay_i = 0; delay_i < (F_CPU / 18000) * 2; ++delay_i) {
    __asm__("NOP");
  }
}

void main() {
  // Setup GPIO pins.
  // Relay control pin.
  set_gpio_pp(GPIOA_BASE, RELAY_PA);
  // 7-Segment LED display pins.
  set_gpio_pp(GPIOA_BASE, SEG_LB_PA);
  set_gpio_pp(GPIOA_BASE, SEG_LF_PA);
  set_gpio_pp(GPIOB_BASE, SEG_D2_PB);
  set_gpio_pp(GPIOB_BASE, SEG_D3_PB);
  set_gpio_pp(GPIOC_BASE, SEG_LC_PC);
  set_gpio_pp(GPIOC_BASE, SEG_LG_PC);
  set_gpio_pp(GPIOD_BASE, SEG_D1_PD);
  set_gpio_pp(GPIOD_BASE, SEG_LA_PD);
  set_gpio_pp(GPIOD_BASE, SEG_LD_PD);
  set_gpio_pp(GPIOD_BASE, SEG_LE_PD);
  set_gpio_pp(GPIOD_BASE, SEG_DP_PD);
  // Resistive input element (thermistor, etc)
  set_gpio_in(GPIOD_BASE, RES_IN_PD, 0);
  // Button input pins.
  set_gpio_in(GPIOC_BASE, KEY_SET_PC, 1);
  set_gpio_in(GPIOC_BASE, KEY_PLS_PC, 1);
  set_gpio_in(GPIOC_BASE, KEY_MIN_PC, 1);

  // Main loop.
  while (1) {
    /* Test 1: Toggle LED and Relay. */
    //#define VVC_W1209_TEST1 (1)
    #undef VVC_W1209_TEST1
    #ifdef VVC_W1209_TEST1
      // Toggle the LED/Relay pin's ODR register bit.
      GPIOx_PP_TOGGLE(GPIOA_BASE, RELAY_PA);
      // Delay a bit.
      for (delay_i = 0; delay_i < (F_CPU / 18000) * RELAY_MS; ++delay_i) {
        __asm__("NOP");
      }
    #endif /* VVC_W1209_TEST1 */

    /* Test 2: Draw 'VVC' to the 7-segment display. */
    #define VVC_W1209_TEST2 (1)
    //#undef VVC_W1209_TEST2
    #ifdef VVC_W1209_TEST2
      draw_7s_digit('6', 0, 0);
      draw_7s_digit('7', 1, 1);
      draw_7s_digit('9', 2, 0);
    #endif
  }
}
