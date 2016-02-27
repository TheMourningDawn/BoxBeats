#include <FastLED.h>
#include <ffft.h>
#include <Wire.h>
#include <avr/pgmspace.h>

FASTLED_USING_NAMESPACE

#define BORDER_LED_PIN    6
#define SHELF_LED_PIN    3
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_BORDER_LEDS 147
#define NUM_SHELF_LEDS 60
#define NUM_LEDS NUM_BORDER_LEDS + NUM_SHELF_LEDS

#define BRIGHTNESS         120
#define FRAMES_PER_SECOND  120

#define ADC_CHANNEL 0

CRGB leds[NUM_BORDER_LEDS];

int16_t capture[FFT_N];    // Audio capture buffer
complex_t bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t spectrum[FFT_N / 2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

int col[8];   // Column levels for the prior 10 frames
int colDiv[8];    // Used when filtering FFT output to 8 columns

// This is low-level noise that's subtracted from each FFT output column:
PROGMEM uint8_t noise[64] = { 8, 6, 6, 5, 3, 4, 4, 4, 3, 4, 4, 3, 2, 3, 3, 4,
2, 1, 2, 1, 3, 2, 3, 2, 1, 2, 3, 1, 2, 3, 4, 4,
3, 2, 2, 2, 2, 2, 2, 1, 3, 2, 2, 2, 2, 2, 2, 2,
2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4
},
// These are scaling quotients for each FFT output column, sort of a
// graphic EQ in reverse.  Most music is pretty heavy at the bass end.
eq[64] = {
255, 175, 218, 225, 220, 198, 147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
},
// When filtering down to 8 columns, these tables contain indexes
// and weightings of the FFT spectrum output values to use.  Not all
// buckets are used -- the bottom-most and several at the top are
// either noisy or out of range or generally not good for a graph.
col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
111,   8
},           // Weights for each bin
col1data[] = {  4,  1,  // 4 bins, starting at index 1
19, 186,  38,   2
}, // Weights for 4 bins.  Got it now?
col2data[] = {  5,  2,
11, 156, 118,  16,   1
},
col3data[] = {  8,  3,
5,  55, 165, 164,  71,  18,   4,   1
},
col4data[] = { 11,  5,
3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1
},
col5data[] = { 17,  7,
2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
41,  21,  10,   5,   2,   1,   1
},
col6data[] = { 25, 11,
1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
4,   2,   1,   1,   1
},
col7data[] = { 37, 16,
1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
3,   2,   2,   1,   1,   1,   1
},
// And then this points to the start of the data for each of the columns:
*colData[] = {
col0data, col1data, col2data, col3data,
col4data, col5data, col6data, col7data
};


void setup() {
    delay(3000); // 3 second delay for recovery
//    Serial.begin(9600);
//    Serial.println("doing");

    FastLED.addLeds<LED_TYPE, BORDER_LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    //The shelf strip is just a mirror of some part of the middle of the border strip...for now
    FastLED.addLeds<LED_TYPE, SHELF_LED_PIN>(leds, 42, 60).setCorrection(TypicalLEDStrip);

    FastLED.setBrightness(BRIGHTNESS);

    uint8_t i, j, nBins, binNum, *data;
//    memset(peak, 0, sizeof(peak));
    memset(col , 0, sizeof(col));

    for (i = 0; i < 8; i++) {
        data         = (uint8_t *)pgm_read_word(&colData[i]);
        nBins        = pgm_read_byte(&data[0]) + 2;
        binNum       = pgm_read_byte(&data[1]);
        for (colDiv[i] = 0, j = 2; j < nBins; j++)
            colDiv[i] += pgm_read_byte(&data[j]);
    }
    // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion
    ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
    ADCSRA = _BV(ADEN)  | // ADC enable
             _BV(ADSC)  | // ADC start
             _BV(ADATE) | // Auto trigger
             _BV(ADIE)  | // Interrupt enable
             _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
    ADCSRB = 0;                // Free run mode, no high MUX bit
    DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
    TIMSK0 = 0;                // Timer0 off

    sei(); // Enable interrupts
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

//SimplePatternList gPatterns = {rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm};
SimplePatternList gPatterns = {confetti, juggle};

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop() {
    // Call the current pattern function once, updating the 'leds' array
    gPatterns[gCurrentPatternNumber]();

    FastLED.show();
    FastLED.delay(1000 / FRAMES_PER_SECOND);

    // do some periodic updates
    EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow
    EVERY_N_SECONDS(15) { nextPattern(); } // change patterns periodically

    /**************************************************/

    uint8_t  i, x, L, *data, nBins, binNum;
    int     y, sum;

    while (ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish

    fft_input(capture, bfly_buff);   // Samples -> complex #s
    samplePos = 0;                   // Reset sample counter
    ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
    fft_execute(bfly_buff);          // Process complex data
    fft_output(bfly_buff, spectrum); // Complex -> spectrum

    // Remove noise and apply EQ levels
    for (x = 0; x < FFT_N / 2; x++) {
        L = pgm_read_byte(&noise[x]);
        spectrum[x] = (spectrum[x] <= L) ? 0 :
                      (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
    }
    // Downsample spectrum output to 8 columns:
    for (x = 0; x < 8; x++) {
        data   = (uint8_t *)pgm_read_word(&colData[x]);
        nBins  = pgm_read_byte(&data[0]) + 2;
        binNum = pgm_read_byte(&data[1]);
        for (sum = 0, i = 2; i < nBins; i++) {
            sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
        }                                                      // Average
        col[x] = sum / colDiv[x];
    }

}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern() {
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}

//void rainbow() {
//    // FastLED's built-in rainbow generator
//    fill_rainbow(leds, NUM_LEDS, gHue, 7);
//}
//
//void rainbowWithGlitter() {
//    // built-in FastLED rainbow, plus some random sparkly glitter
//    rainbow();
//    addGlitter(80);
//}
//
//void addGlitter(fract8 chanceOfGlitter) {
//    if (random8() < chanceOfGlitter) {
//        leds[random16(NUM_LEDS)] += CRGB::White;
//    }
//}

void confetti() {
//    Serial.println("I'm doin this method");
    // random colored speckles that blink in and fade smoothly
    fadeToBlackBy(leds, NUM_LEDS, 10);
    uint8_t pos = random16(NUM_LEDS);

    if(col[1] > 1) {
        leds[pos] += CHSV(gHue + random8(64), 200, 255);
    }
}

//void sinelon() {
//    // a colored dot sweeping back and forth, with fading trails
//    fadeToBlackBy(leds, NUM_LEDS, 20);
//    int pos = beatsin16(13, 0, NUM_LEDS);
//    leds[pos] += CHSV(gHue, 255, 192);
//}
//
//void bpm() {
//    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
//    uint8_t BeatsPerMinute = 62;
//    CRGBPalette16 palette = PartyColors_p;
//    uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
//    for (int i = 0; i < NUM_LEDS; i++) { //9948
//        leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
//    }
//}
//
void juggle() {
    // eight colored dots, weaving in and out of sync with each other
    fadeToBlackBy(leds, NUM_LEDS, 20);
    byte dothue = 0;
    if(col[1] > 1) {
        for (int i = 0; i < 8; i++) {
            leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
            dothue += 32;
        }
    }
}


ISR(ADC_vect) { // Audio-sampling interrupt
        static const int16_t noiseThreshold = 4;
        int16_t              sample         = ADC; // 0-1023

        capture[samplePos] =
        ((sample > (512-noiseThreshold)) &&
        (sample < (512+noiseThreshold))) ? 0 :
        sample - 512; // Sign-convert for FFT; -512 to +511

        if(++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}

