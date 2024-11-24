#define SET_BIT(REG, BIT_POSN) (REG |= (1 << BIT_POSN))
#define CLEAR_BIT(REG, BIT_POSN) (REG &= ~(1 << BIT_POSN))
#define TOGGLE_BIT(REG, BIT_POSN) (REG ^= (1 << BIT_POSN))
#define READ_BIT(REG, BIT_POSN) ((REG >> BIT_POSN) & 1)

#define LED_BLINK_PIN 8
#define LED_FADE_PIN 9
#define LED_DEBOUNCE_PIN 10
#define BUTTON_BLINK_PIN 2
#define BUTTON_FADE_PIN 3

#define TIMER_0_MULTIPLIER 100
#define TIMER_2_MULTIPLIER 15

#define OCR1A_INIT 1
#define OCR1A_AMNT 75

#define OCR0A_INIT 1
#define OCR0A_AMNT 30

uint16_t timer_0_multiplier_runner = 0;
bool enable_buttons = true;
uint8_t debounce_counter = 0;

// Custom digitalWrite implementation
void customDigitalWrite(uint8_t pin, uint8_t value) {
    if (pin >= 0 && pin <= 7) { // Port D
        if (value == HIGH) {
            SET_BIT(PORTD, pin); // Set the pin HIGH
        } else {
            CLEAR_BIT(PORTD, pin); // Set the pin LOW
        }
    } else if (pin >= 8 && pin <= 13) { // Port B
        pin -= 8; // Adjust for port B offset
        if (value == HIGH) {
            SET_BIT(PORTB, pin);
        } else {
            CLEAR_BIT(PORTB, pin);
        }
    } else if (pin >= A0 && pin <= A5) { // Port C (Analog pins)
        pin -= A0; // Adjust for port C offset
        if (value == HIGH) {
            SET_BIT(PORTC, pin);
        } else {
            CLEAR_BIT(PORTC, pin);
        }
    }
}

// Custom digitalRead implementation
uint8_t customDigitalRead(uint8_t pin) {
    if (pin >= 0 && pin <= 7) { // Port D
        return READ_BIT(PIND, pin);
    } else if (pin >= 8 && pin <= 13) { // Port B
        pin -= 8; // Adjust for port B offset
        return READ_BIT(PINB, pin);
    } else if (pin >= A0 && pin <= A5) { // Port C (Analog pins)
        pin -= A0; // Adjust for port C offset
        return READ_BIT(PINC, pin);
    }
    return LOW; // Invalid pin
}

void init_timer_counter_0() {
    TCCR0A = (1 << WGM01); // Set CTC mode
    TCCR0B = (1 << CS02) | (1 << CS00); // Set prescaler to 1024
    OCR0A = OCR0A_INIT; // Set compare value
    TCNT0 = 0; // Reset counter value
    TIMSK0 = (1 << OCIE0A); // Enable compare match interrupt
}

void init_timer_counter_1() {
    // Set Fast PWM mode 10-bit
    TCCR1A |= (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << WGM12);
    OCR1A = OCR1A_INIT; // Set compare value
    TCCR1A |= (1 << COM1A1); // Set non-inverting mode
    TCCR1B |= (1 << CS11) | (1 << CS10); // Set prescaler to 64
}

void init_timer_counter_2() {
    TCCR2A = 0; // Normal mode
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Set prescaler to 1024
    TIMSK2 = (1 << TOIE2); // Enable overflow interrupt
}

void increase_brightness() {
    OCR1A = (OCR1A + OCR1A_AMNT) % 1024;
}

void decrease_blink_speed() {
    OCR0A = (OCR0A + OCR0A_AMNT) % 256;
}

void setup() {
    // Set pin modes using direct register access
    SET_BIT(DDRB, LED_BLINK_PIN - 8); // Port B pin 8
    SET_BIT(DDRB, LED_FADE_PIN - 8); // Port B pin 9
    SET_BIT(DDRB, LED_DEBOUNCE_PIN - 8); // Port B pin 10
    CLEAR_BIT(DDRD, BUTTON_BLINK_PIN); // Port D pin 2
    CLEAR_BIT(DDRD, BUTTON_FADE_PIN); // Port D pin 3

    // Enable pull-up resistors for buttons
    SET_BIT(PORTD, BUTTON_BLINK_PIN);
    SET_BIT(PORTD, BUTTON_FADE_PIN);

    init_timer_counter_0(); // Initialize timer 0
    init_timer_counter_1(); // Initialize timer 1
    init_timer_counter_2(); // Initialize timer 2

    sei(); // Enable global interrupts
}

void loop() {
    if (customDigitalRead(BUTTON_FADE_PIN) == LOW && enable_buttons) {
        increase_brightness();
        enable_buttons = false;
    }
    if (customDigitalRead(BUTTON_BLINK_PIN) == LOW && enable_buttons) {
        decrease_blink_speed();
        enable_buttons = false;
    }
}

ISR(TIMER0_COMPA_vect) {
    if (timer_0_multiplier_runner <= TIMER_0_MULTIPLIER) {
        timer_0_multiplier_runner++;
    } else {
        timer_0_multiplier_runner = 0;
        toggle_led(LED_BLINK_PIN);
    }
}

void toggle_led(int led_pin) {
    // Use customDigitalWrite for toggling the LED
    customDigitalWrite(led_pin, !customDigitalRead(led_pin));
}

ISR(TIMER2_OVF_vect) {
    if (!enable_buttons) {
        customDigitalWrite(LED_DEBOUNCE_PIN, HIGH);
        debounce_counter++;
        if (debounce_counter >= TIMER_2_MULTIPLIER) {
            debounce_counter = 0;
            enable_buttons = true;
            customDigitalWrite(LED_DEBOUNCE_PIN, LOW);
        }
        TCNT2 = 12; // Preload timer
    }
    CLEAR_BIT(TIFR2, TOV2); // Clear overflow flag
}
