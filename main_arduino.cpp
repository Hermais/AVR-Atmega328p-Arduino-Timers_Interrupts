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
uint8_t timer_2_multiplier_debounce_counter = 0;


// Clears all provided registers by setting them to 0.
template<typename... Registers> // A variadic template (generic) that accepts many args. 
// Values here are passed by refrence (a short hand method to pointer manipulation).
void clear_all(Registers&... registers) {
    // fold expression that applies for each argument the operation of setting them to zero.
    ((registers = 0), ...);
}



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
    clear_all(TCCR0A, TCCR0B, TIMSK0, TCNT0);
    SET_BIT(TCCR0A, WGM01); // Set CTC mode
    SET_BIT(TCCR0B, CS02); // Set prescaler to 1024
    SET_BIT(TCCR0B, CS00);
    OCR0A = OCR0A_INIT; // Set compare value
    SET_BIT(TIMSK0, OCIE0A); // Enable compare match interrupt

}

void init_timer_counter_1() {
    clear_all(TCCR1A, TCCR1B);
    // Set Fast PWM mode 10-bit
    SET_BIT(TCCR1A, WGM11);
    SET_BIT(TCCR1A, WGM10);
    SET_BIT(TCCR1B, WGM12);
    OCR1A = OCR1A_INIT; // Set compare value
    SET_BIT(TCCR1A, COM1A1); // Set non-inverting mode
    SET_BIT(TCCR1B, CS11);
    SET_BIT(TCCR1B, CS10); // Set prescaler to 64
}

void init_timer_counter_2() {
    clear_all(TCCR2A, TCCR2B, TIMSK2, TCNT2);
    SET_BIT(TCCR2B, CS22); // Set prescaler to 1024
    SET_BIT(TCCR2B, CS21);
    SET_BIT(TCCR2B, CS20);
    SET_BIT(TIMSK2, TOIE2); // Enable overflow interrupt
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
        timer_2_multiplier_debounce_counter++;
        if (timer_2_multiplier_debounce_counter >= TIMER_2_MULTIPLIER) {
            timer_2_multiplier_debounce_counter = 0;
            enable_buttons = true;
            customDigitalWrite(LED_DEBOUNCE_PIN, LOW);
        }
    }
    CLEAR_BIT(TIFR2, TOV2); // Clear overflow flag
}
