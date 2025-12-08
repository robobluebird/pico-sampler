#include <Wire.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "RPi_Pico_ISR_Timer.h"
#include "hardware/gpio.h"  // For fast GPIO access

// Forward declarations
void write_dac_sample(uint16_t sample);
uint16_t read_ram_sample(uint32_t global_address);
void write_ram_sample_sequential(uint16_t sample);
void deselect_all_chips();
void set_ram_byte_mode();
void drawOptionList();
void drawSampleList();
void updateSequencerScreen();
void updateSongScreen();
void updateRecordingScreen();

#define R1 7
#define R2 8
#define R3 9
#define R4 10

#define LED_PIN 22
#define SYNC_PIN 17
#define DAC_CS_PIN 13

#define MAX_SAMPLES 10

#define TIMER_FREQ_HZ 16000  // 22050 // 11025
#define SYNC_PULSE_DURATION 60  // ~5 ms at 12,000 Hz

#define STEPS_PER_MEASURE 16
#define MAX_LAYERED_SAMPLES 3  // Can expand later

#define SAMPLE_RATE 16000  // 11025 // 22050
// #define BPM 120
// #define SAMPLES_PER_STEP 1000 // 1378 // 689 // ((SAMPLE_RATE * 60) / (BPM * STEPS_PER_MEASURE))  // ~172

#define TANH_TABLE_SIZE 16384
#define TANH_TABLE_OFFSET 8192

int16_t tanh_table[TANH_TABLE_SIZE];

uint16_t bpm = 120;
uint32_t saved_samples_per_step = 1000;
uint32_t samples_per_step = 1000;
bool doubleTime = false;
bool stutter = false;
bool swing = false;
bool needsUpdate = false;
bool clipping = false;
uint16_t clipping_counter = 0;

typedef struct {
  uint8_t sample_id;  // Index in your sample list
  bool triggered;
} SequencerStep;

SequencerStep sequencer_map[2][STEPS_PER_MEASURE];

typedef struct {
  SequencerStep steps[STEPS_PER_MEASURE];
} Sequence;

#define MAX_SEQUENCES 16
Sequence sequence_bank[2][MAX_SEQUENCES];
int sequenceCount = 0;

void save_sequence() {
  if (sequenceCount < MAX_SEQUENCES) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < STEPS_PER_MEASURE; j++) {
        sequence_bank[i][sequenceCount].steps[j] = sequencer_map[i][j];
      }
    }

    sequenceCount++;
  }
}

void load_sequence(int slot) {
  if (slot < 0 || slot >= MAX_SEQUENCES) return;

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < STEPS_PER_MEASURE; j++) {
      sequencer_map[i][j] = sequence_bank[i][slot].steps[j];
    }
  }
}

#define MAX_SEQUENCE_CHAIN 32
int sequence_chain[MAX_SEQUENCE_CHAIN] = {
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};
int current_chain_index = 0;

typedef struct {
  uint8_t sample_id;
  uint32_t accumulator;
  bool active;
} ActiveSample;

ActiveSample active_samples[MAX_LAYERED_SAMPLES];

uint32_t sequencer_tick_counter = samples_per_step;
uint8_t current_step = 0;

enum State {
  HOME_STATE,
  RECORD_STATE,
  INDEX_STATE,
  EDIT_STATE,
  PLAY_STATE,
  DONE_PLAYING,
  SAMPLER_STATE,
  SELECT_SAMPLE_STATE,
  SAMPLER_PLAYBACK_STATE,
  SONG_STATE,
  SONG_PLAYBACK_STATE,
  SELECT_SEQUENCE_STATE,
  SELECT_MATCH_STATE
};

enum EditState {
  SLICE,
  PITCH
};

int selectedSample = 0;             // Index of selected sample in INDEX_STATE
unsigned long recordStart = 0;      // For tracking seconds in RECORD_STATE
uint16_t playback_end_address = 0;  // instead of overloading global_position

byte bars[7][8] = {
  // Slot 0: LTR 1/5
  { 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000 },
  // Slot 1: LTR 2/5
  { 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000 },
  // Slot 2: LTR 3/5
  { 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100 },
  // Slot 3: LTR 4/5
  { 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110 },
  // Slot 4: RTL 4/5
  { 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111 },
  // Slot 5: RTL 3/5
  { 0b00111, 0b00111, 0b00111, 0b00111, 0b00111, 0b00111, 0b00111, 0b00111 },
  // Slot 6: RTL 2/5
  { 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011 }
};

uint32_t playback_accumulator = 0;
uint8_t playback_pitch = 64;  // Base pitch (1.0x speed), shift amount = 6 bits

uint32_t totalSamplesInView = 0;  // default placeholder
const int TOTAL_PIXELS = 16 * 5;
const int BAR_CHARS = 16;

bool loopMode = false;
bool isPlayingSlice = false;
uint32_t sliceStart = 0;
uint32_t sliceEnd = 0;

uint32_t savedSliceStart = 0;
uint32_t savedSliceEnd = 0;
uint32_t lockedSampleLength = 0;
bool sampleLengthLocked = false;
int8_t lockedSampleIndex = -1;
int8_t sampleLengthDivision = 0; //0 == 1/1, 1 == 1/2, 2 == 1/4

// const float alpha = 0.2f;  // LPF factor
// float lpfStart = 0;
// float lpfEnd = 0;

// bool auditionMode = true;
unsigned long lastCursorUpdate = 0;
const int cursorInterval = 80;  // ms per step
int cursorPixel = 0;

typedef struct {
  char name[16];
  uint32_t start;
  uint32_t end;
  uint8_t pitch{ 64 };
  uint8_t beats{ 4 };
  bool changes_tempo{ false };
  int matched_sample_id{ -1 };
  uint8_t matched_pitch{ 0 };
} Sample;

typedef struct {
  char name[16];
} Option;

Option options[3] = { "SAMPLES", "SEQUENCE", "SONG" };
uint8_t selectedOption = 0;
Sample samples[MAX_SAMPLES] = { 0 };
uint8_t sampleCount = 0;
uint8_t optionCount = 3;

State currentState = HOME_STATE;
State nextState = INDEX_STATE;
EditState editInnerState = SLICE;

const uint8_t chip_select_pins[4] = { R1, R2, R3, R4 };
const uint32_t CHUNK_SIZE = 0x20000;          // 128KB per chip
uint32_t maxGlobalPosition = CHUNK_SIZE * 4;  // = 0x80000
uint32_t global_position = 0;                 // now in BYTES
uint32_t starting_record_position;
uint32_t playback_address = 0;
volatile int encoderPosition = 0;
int lastPosition = 0;
unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 60;
unsigned long now = 0;
uint32_t sync_pulse_counter = 0;

RPI_PICO_Timer ITimer(0);

typedef struct {
  uint8_t pin;
  unsigned long lastDebounceTime = 0;
  bool stableState = HIGH;
  bool lastStableState = HIGH;
  bool edgeFalling = false;
  bool edgeRising = false;
  unsigned long debounceDelay = 50;
  unsigned long pressStartTime = 0;
  bool isHeld = false;
  unsigned long holdThreshold = 250; // milliseconds to qualify as "held"

  void begin() {
    pinMode(pin, INPUT_PULLUP);
  }

  void update() {
    bool reading = digitalRead(pin);

    if (reading != stableState && (millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis();
      lastStableState = stableState;
      stableState = reading;
      edgeFalling = (lastStableState == HIGH && stableState == LOW);
      edgeRising = (lastStableState == LOW && stableState == HIGH);

      if (edgeFalling) {
        pressStartTime = millis();
        isHeld = false;
      }

      if (edgeRising) {
        isHeld = false;
      }
    } else {
      edgeFalling = false;
      edgeRising = false;

      if (stableState == LOW && !isHeld && (millis() - pressStartTime) > holdThreshold) {
        isHeld = true;
      }
    }
  }

  bool fell() {
    return edgeFalling;
  }

  bool rose() {
    return edgeRising;
  }

  bool isDown() {
    return stableState == LOW;
  }

  bool held() {
  return isHeld;
  }

  bool pressed() {
    return edgeFalling;
  }

  bool released() {
    return edgeRising;
  }
} DebouncedButton;

DebouncedButton recordButton = { 6 };
DebouncedButton selectButton = { 14 };
DebouncedButton upButton = { 1 };
DebouncedButton downButton = { 0 };
DebouncedButton modeButton = { 15 };

uint8_t last_step = 255;
uint8_t cursorPos = 0;
uint8_t edit_cursor_pos = 0;
uint8_t songCursorPos = 0;

inline void fastDigitalWriteHigh(uint8_t pin) {
  gpio_set_mask(1ul << pin);
}

inline void fastDigitalWriteLow(uint8_t pin) {
  gpio_clr_mask(1ul << pin);
}

bool TimerHandler(struct repeating_timer *t) {
  if (currentState == RECORD_STATE) {
    if (global_position >= maxGlobalPosition) {
      currentState = DONE_PLAYING;
      return true;
    }

    uint16_t sample = analogRead(A2);

    if (clipping_counter > 0) {
      clipping_counter--;
    } else if (sample < 500 || sample > 3500) {
      clipping = true;
      clipping_counter = 500;
      fastDigitalWriteHigh(LED_PIN);
    } else if (clipping) {
      clipping = false;
      fastDigitalWriteLow(LED_PIN);
    }

    // write_dac_sample(sample);

    write_ram_sample_sequential(sample);
  } else if (currentState == PLAY_STATE) {
    // Advance accumulator by pitch
    playback_accumulator += playback_pitch;

    // Truncate fractional bits to get sample index
    uint32_t sample_index = playback_accumulator >> 6;

    // Convert index to byte address (2 bytes per 12-bit sample)
    playback_address = sliceStart + sample_index * 2;

    // Read and write sample
    write_dac_sample(read_ram_sample(playback_address));

    // Loop or exit
    if (playback_address >= sliceEnd) {
      if (isPlayingSlice && loopMode) {
        playback_accumulator = 0;  // Reset to start
      } else {
        currentState = nextState;
        isPlayingSlice = false;
        playback_accumulator = 0;
      }
    }
  } else if (currentState == SAMPLER_PLAYBACK_STATE || currentState == SONG_PLAYBACK_STATE) {
    // Step progression
    if (++sequencer_tick_counter >= samples_per_step) {
      sequencer_tick_counter = 0;
      int free_slot = -1;

      // Reset all voices at start of new measure
      if (current_step == 0) {
        if (currentState == SONG_PLAYBACK_STATE) {
          if (sequence_chain[current_chain_index] == -1) {
            if (current_chain_index == 0) {
              // nothing input, stop
              edit_cursor_pos = songCursorPos;
              currentState = SONG_STATE;
              return true;
            } else {
              current_chain_index = 0;
            }
          }

          load_sequence(sequence_chain[current_chain_index]);

          // recalc the samples_per_step MAYBE
          if (sequencer_map[0][0].triggered && samples[sequencer_map[0][0].sample_id].changes_tempo) {
            int sampleLength = samples[sequencer_map[0][0].sample_id].end - samples[sequencer_map[0][0].sample_id].start;
            float pitchModifier = (float)64 / samples[sequencer_map[0][0].sample_id].pitch;
            float pitchModifiedSampleLength = sampleLength * pitchModifier;
            samples_per_step = (int)(pitchModifiedSampleLength / 32); // WHAT??
          }
          
          current_chain_index = (current_chain_index + 1) % MAX_SEQUENCE_CHAIN;
        }

        for (int i = 0; i < MAX_LAYERED_SAMPLES; i++) {
          active_samples[i].active = false;
        }
      }

      if (swing && sync_pulse_counter == 0) {
        // Swing pattern for 16 steps (adjusted from 32)
        if (current_step % 4 == 0 || current_step == 2 || current_step == 6 || current_step == 10 || current_step == 14 ) {
          fastDigitalWriteHigh(SYNC_PIN);
          sync_pulse_counter = SYNC_PULSE_DURATION;
        }
      } else if (((doubleTime && current_step % 1 == 0) || current_step % 2 == 0) && sync_pulse_counter == 0) {
        fastDigitalWriteHigh(SYNC_PIN);
        sync_pulse_counter = SYNC_PULSE_DURATION;
      }

      if (current_step % 4 == 0) {  // Every 4 steps = quarter note (was 8 for 32 steps)
        if (currentState == SAMPLER_PLAYBACK_STATE) {
          cursorPos = current_step;  // Show playback position only on quarter notes
        } else {
          if (current_step == 0) {
            songCursorPos = current_chain_index - 1;
          }
        }

        fastDigitalWriteHigh(LED_PIN);
        needsUpdate = true;
      } else if (current_step % 2 == 0) {  // Every 2 steps = eighth note (was 4 for 32 steps)
        fastDigitalWriteLow(LED_PIN);
      }

      // Trigger step - optimized to handle both primary and secondary in one pass
      SequencerStep *primaryStep = &sequencer_map[0][current_step];
      SequencerStep *secondaryStep = &sequencer_map[1][current_step];
      bool needPrimarySlot = primaryStep->triggered;
      bool needSecondarySlot = secondaryStep->triggered;
      int8_t primarySlot = -1;
      int8_t secondarySlot = -1;
      
      // Single pass to find slots and reset existing samples
      for (int i = 0; i < MAX_LAYERED_SAMPLES; i++) {
        if (active_samples[i].active) {
          // Check if this is a retriggered sample
          if (needPrimarySlot && active_samples[i].sample_id == primaryStep->sample_id) {
            active_samples[i].accumulator = 0;
            needPrimarySlot = false;
            continue;
          }
          if (needSecondarySlot && active_samples[i].sample_id == secondaryStep->sample_id) {
            active_samples[i].accumulator = 0;
            needSecondarySlot = false;
            continue;
          }
        } else {
          // Found free slot
          if (needPrimarySlot && primarySlot == -1) {
            primarySlot = i;
          } else if (needSecondarySlot && secondarySlot == -1) {
            secondarySlot = i;
          }
        }
      }
      
      // Activate new samples
      if (needPrimarySlot && primarySlot != -1) {
        active_samples[primarySlot].sample_id = primaryStep->sample_id;
        active_samples[primarySlot].accumulator = 0;
        active_samples[primarySlot].active = true;
      }
      if (needSecondarySlot && secondarySlot != -1) {
        active_samples[secondarySlot].sample_id = secondaryStep->sample_id;
        active_samples[secondarySlot].accumulator = 0;
        active_samples[secondarySlot].active = true;
      }

      current_step = (current_step + 1) % STEPS_PER_MEASURE;
    }

    if (sync_pulse_counter > 0) {
      sync_pulse_counter--;

      if (sync_pulse_counter == 0) {
        fastDigitalWriteLow(SYNC_PIN);
      }
    }

    int32_t mixed_sample = 0;

    for (int i = 0; i < MAX_LAYERED_SAMPLES; i++) {
      if (active_samples[i].active) {
        Sample *s = &samples[active_samples[i].sample_id];

        uint32_t idx = active_samples[i].accumulator >> 6;
        uint32_t address = s->start + idx * 2;

        if (address < s->end) {
          mixed_sample += read_ram_sample(address) - 2048;  // signed 16-bit

          // Use matched_pitch if available, otherwise use pitch (avoid branch with ternary on uint8_t)
          uint8_t pitch = s->matched_pitch ? s->matched_pitch : s->pitch;
          active_samples[i].accumulator += pitch;
        } else {
          active_samples[i].active = false;
        }
      }
    }

    // Clamp to tanh table range
    if (mixed_sample > 8191) mixed_sample = 8191;
    if (mixed_sample < -8192) mixed_sample = -8192;

    // Soft clip
    int16_t clipped = tanh_table[mixed_sample + TANH_TABLE_OFFSET];

    // Convert to DAC range
    uint16_t dac_value = clipped + 2048;

    if (stutter && current_step % 2 != 0) {
      dac_value = 2048;
    }

    write_dac_sample(dac_value);
  } else if (currentState != EDIT_STATE) {
    uint16_t sample = analogRead(A2);

    if (clipping_counter > 0) {
      clipping_counter--;
    } else if (sample < 500 || sample > 3500) {
      clipping = true;
      clipping_counter = 500;
      fastDigitalWriteHigh(LED_PIN);
    } else if (clipping) {
      clipping = false;
      fastDigitalWriteLow(LED_PIN);
    }
    
    write_dac_sample(sample);
  }

  return true;
}

void init_tanh_table() {
  for (int i = -TANH_TABLE_OFFSET; i < TANH_TABLE_OFFSET; i++) {
    float x = i / 4096.0f;  // scale to ±2.0 range
    tanh_table[i + TANH_TABLE_OFFSET] = (int16_t)(tanhf(x) * 2047.0f);
  }
}

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(115200);

  init_tanh_table();

  //button.begin();
  recordButton.begin();
  selectButton.begin();
  upButton.begin();
  downButton.begin();
  modeButton.begin();

  SPI.setRX(16);
  SPI.setSCK(18);
  SPI.setTX(19);
  SPI.begin(false);

  analogReadResolution(12);

  pinMode(LED_PIN, OUTPUT);
  pinMode(SYNC_PIN, OUTPUT);

  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(chip_select_pins[i], OUTPUT);
    fastDigitalWriteHigh(chip_select_pins[i]);
  }

  pinMode(DAC_CS_PIN, OUTPUT);
  fastDigitalWriteHigh(DAC_CS_PIN);

  ITimer.attachInterrupt(TIMER_FREQ_HZ, TimerHandler);

  deselect_all_chips();
  set_ram_byte_mode();

  for (int i = 0; i < 7; i++) {
    lcd.createChar(i, bars[i]);
  }

  lcd.begin(16, 2);
  drawOptionList();
}

void write_dac_sample(uint16_t sample) {
  fastDigitalWriteLow(DAC_CS_PIN);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.transfer16(0b0101000000000000 | sample);
  SPI.endTransaction();
  fastDigitalWriteHigh(DAC_CS_PIN);
}

// Track current chip to avoid unnecessary deselects
uint8_t current_selected_chip = 255;

void select_chip(uint8_t chip) {
  if (current_selected_chip == chip) return;  // Already selected
  
  // Deselect all and select target
  for (uint8_t i = 0; i < 4; ++i)
    fastDigitalWriteHigh(chip_select_pins[i]);
  fastDigitalWriteLow(chip_select_pins[chip]);
  
  current_selected_chip = chip;
}

void deselect_all_chips() {
  for (uint8_t i = 0; i < 4; ++i)
    fastDigitalWriteHigh(chip_select_pins[i]);
  current_selected_chip = 255;
}

void set_ram_mode(uint8_t mode) {
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(chip_select_pins[i], LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
    SPI.transfer(1);
    SPI.transfer(mode);
    SPI.endTransaction();
    digitalWrite(chip_select_pins[i], HIGH);
  }
}

void set_ram_byte_mode() {
  set_ram_mode(0);
}
void set_ram_sequential_mode() {
  set_ram_mode(64);
}

uint16_t read_ram_sample(uint32_t global_address) {
  uint8_t chip = global_address / CHUNK_SIZE;
  uint32_t local_address = global_address % CHUNK_SIZE;

  // Read first byte (COMPLETE transaction)
  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(3);
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  uint8_t upper = SPI.transfer(0);
  SPI.endTransaction();
  deselect_all_chips();  // CS HIGH - complete transaction

  // Read second byte (NEW transaction)
  global_address++;
  chip = global_address / CHUNK_SIZE;
  local_address = global_address % CHUNK_SIZE;
  
  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(3);
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  uint8_t lower = SPI.transfer(0);
  SPI.endTransaction();
  deselect_all_chips();  // CS HIGH - complete transaction

  return ((uint16_t)upper << 8) | lower;
}

// void begin_reading_ram_sequential(uint32_t global_address) {
//   uint8_t chip = global_address / CHUNK_SIZE;
//   uint32_t local_address = global_address % CHUNK_SIZE;

//   select_chip(chip);
//   SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
//   SPI.transfer(3);
//   SPI.transfer((local_address >> 16) & 0xFF);
//   SPI.transfer((local_address >> 8) & 0xFF);
//   SPI.transfer(local_address & 0xFF);
//   SPI.endTransaction();
// }

// uint16_t read_ram_sample_sequential() {
//   uint8_t chip = playback_address / CHUNK_SIZE;

//   SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
//   uint8_t upper = SPI.transfer(0);
//   uint8_t lower = SPI.transfer(0);
//   SPI.endTransaction();
//   playback_address += 2;

//   uint8_t next_chip = global_position / CHUNK_SIZE;
//   if (next_chip != chip) {
//     begin_reading_ram_sequential();
//   }

//   return ((uint16_t)upper << 8) | lower;
// }

void begin_writing_ram_sequential() {
  uint8_t chip = global_position / CHUNK_SIZE;
  uint32_t local_address = global_position % CHUNK_SIZE;

  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(2);
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  SPI.endTransaction();
}

void write_ram_sample_sequential(uint16_t sample) {
  uint8_t chip = global_position / CHUNK_SIZE;

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  // Write as two separate bytes to match how we read
  SPI.transfer((sample >> 8) & 0xFF);  // Upper byte
  SPI.transfer(sample & 0xFF);         // Lower byte
  SPI.endTransaction();

  // write_dac_sample(sample);

  global_position += 2;

  uint8_t next_chip = global_position / CHUNK_SIZE;
  if (next_chip != chip) {
    begin_writing_ram_sequential();
  }
}

void end_writing_ram_sequential() {
  deselect_all_chips();
  set_ram_byte_mode();
}

void startRecording() {
  recordStart = millis();
  starting_record_position = global_position;

  set_ram_sequential_mode();
  begin_writing_ram_sequential();

  currentState = RECORD_STATE;
}

void stopRecording() {
  currentState = INDEX_STATE;
  end_writing_ram_sequential();

  fastDigitalWriteLow(LED_PIN);

  if (global_position % 2 != 0) {
    global_position--;  // Ensure even byte alignment
  }

  if (sampleCount < MAX_SAMPLES) {
    Sample *s = &samples[sampleCount];
    snprintf(s->name, sizeof(s->name), "Sample %c", 'A' + sampleCount);
    s->start = starting_record_position;
    s->end = global_position;

    if (s->start % 2 != 0) {
      s->start--;  // Ensure even byte alignment
    }

    if (s->end % 2 != 0) {
      s->end--;  // Ensure even byte alignment
    }

    Serial.print("starting_record_position: ");
    Serial.println(starting_record_position);
    Serial.print("global_position: ");
    Serial.println(global_position);
    Serial.print("len: ");
    Serial.println(global_position - starting_record_position);
    Serial.print("start: ");
    Serial.println(s->start);
    Serial.print("end: ");
    Serial.println(s->end);

    sampleCount++;
  }

  selectedSample = sampleCount - 1;  // Auto-select new sample
  drawSampleList();
}

void updateRecordingScreen() {
  unsigned long elapsed = (millis() - recordStart) / 1000;

  lcd.setCursor(0, 0);
  lcd.print("Recording...");

  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(elapsed);
  lcd.print("s        ");  // Padding to clear leftovers
}

void updateCurrentStateIfNeeded() {
  if (currentState == DONE_PLAYING) {
    currentState = nextState;
  }
}

void enterSelectedOption() {
  switch (selectedOption) {
    case 0:
      {
        currentState = INDEX_STATE;
        drawSampleList();
      }
      break;
    case 1:
      {
        sequencer_tick_counter = samples_per_step;
        current_step = 0;
        currentState = SAMPLER_STATE;
        updateSequencerScreen();
      }
      break;
    case 2:
      {
        sequencer_tick_counter = samples_per_step;
        current_step = 0;
        currentState = SONG_STATE;
        updateSongScreen();
      }
      break;
  }
}

void updateSongScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);

  for (uint8_t i = 0; i < 16; i++) {
    if (songCursorPos == i) {
      if (currentState == SONG_PLAYBACK_STATE) {
        lcd.write("*");
      } else {
        lcd.write(255);
      }
    } else {
      if (sequence_chain[i] == -1) {
        lcd.write("-");
      } else {
        lcd.write('A' + sequence_chain[i]);
      }
    }
  }
  
  // Second row - next 16 positions
  lcd.setCursor(0, 1);
  for (uint8_t i = 16; i < 32; i++) {
    if (songCursorPos == i) {
      if (currentState == SONG_PLAYBACK_STATE) {
        lcd.write("*");
      } else {
        lcd.write(255);
      }
    } else {
      if (sequence_chain[i] == -1) {
        lcd.write("-");
      } else {
        lcd.write('A' + sequence_chain[i]);
      }
    }
  }
}

void updateSequencerScreen() {
  lcd.clear();
  
  // Top row - PRIMARY samples (sequencer_map[0])
  lcd.setCursor(0, 0);
  for (uint8_t i = 0; i < 16; i++) {
    if (cursorPos == i) {
      if (currentState == SAMPLER_PLAYBACK_STATE) {
        lcd.write("*");
      } else {
        lcd.write(255);
      }
    } else {
      if (sequencer_map[0][i].triggered) {
        lcd.write('A' + sequencer_map[0][i].sample_id);
      } else {
        lcd.write("-");
      }
    }
  }
  
  // Bottom row - SECONDARY samples (sequencer_map[1])
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < 16; i++) {
    if (cursorPos == i) {
      // Show cursor on bottom row too if on that position
      if (currentState == SAMPLER_PLAYBACK_STATE) {
        lcd.write("*");
      } else {
        lcd.write(255);
      }
    } else {
      if (sequencer_map[1][i].triggered) {
        lcd.write('A' + sequencer_map[1][i].sample_id);
      } else {
        lcd.write(" ");
      }
    }
  }
}

void drawOptionList() {
  lcd.clear();

  if (selectedOption == optionCount - 1 && optionCount > 1) {
    lcd.setCursor(0, 0);
    lcd.print("  ");
    lcd.print(options[selectedOption - 1].name);

    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.print(options[selectedOption].name);
  } else {
    lcd.setCursor(0, 0);
    lcd.print("> ");
    lcd.print(options[selectedOption].name);

    if (selectedOption + 1 < optionCount) {
      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print(options[selectedOption + 1].name);
    }
  }
}

uint8_t sequencerSelectedSample = 0;
uint8_t songSelectedSequence = 0;

void drawSelectSequenceList() {
  lcd.clear();

  if (songSelectedSequence == 0) { // "None" is selected
    // "None" is selected
    lcd.setCursor(0, 0);
    lcd.print("> None");

    if (sequenceCount > 0) {
      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print("Sequence A");  // First real sample
    }
  } else if (songSelectedSequence == sequenceCount) {
    // Last sample is selected, show previous above
    if (sequenceCount == 1) {
      lcd.setCursor(0, 0);
      lcd.print("  None");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("  ");
      lcd.printf("Sequence %c", 'A' + (songSelectedSequence - 2));  // Previous sample
    }

    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.printf("Sequence %c", 'A' + (songSelectedSequence - 1));  // Current sample
  } else {
    // Middle of the list
    lcd.setCursor(0, 0);
    lcd.print("> ");
    lcd.printf("Sequence %c", 'A' + (songSelectedSequence - 1));  // Current sample

    lcd.setCursor(0, 1);
    lcd.print("  ");
    lcd.printf("Sequence %c", 'A' + songSelectedSequence);  // Next sample
  }
}

void drawSelectSampleList() {
  lcd.clear();

  if (sequencerSelectedSample == 0) {
    // "None" is selected
    lcd.setCursor(0, 0);
    lcd.print("> None");

    if (sampleCount > 0) {
      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print(samples[0].name);  // First real sample
    }
  } else if (sequencerSelectedSample == sampleCount) {
    // Last sample is selected, show previous above
    if (sampleCount == 1) {
      lcd.setCursor(0, 0);
      lcd.print("  None");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("  ");
      lcd.print(samples[sequencerSelectedSample - 2].name);  // Previous sample
    }

    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.print(samples[sequencerSelectedSample - 1].name);  // Current sample
  } else {
    // Middle of the list
    lcd.setCursor(0, 0);
    lcd.print("> ");
    lcd.print(samples[sequencerSelectedSample - 1].name);  // Current sample

    lcd.setCursor(0, 1);
    lcd.print("  ");
    lcd.print(samples[sequencerSelectedSample].name);  // Next sample
  }
}

uint8_t selectedMatchSample = 0;

void drawSelectMatchList() {
  lcd.clear();

  //selectedMatchSample
  if (selectedMatchSample == 0) {
    lcd.setCursor(0, 0);
    lcd.print("MATCH TO???");
    lcd.setCursor(0, 1);
    lcd.print("> None");
  } else if (selectedMatchSample == sampleCount) {
    if (sampleCount == 1) {
      lcd.setCursor(0, 0);
      lcd.print("  None");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("  ");
      lcd.print(samples[selectedMatchSample - 2].name);  // Previous sample
    }

    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.print(samples[selectedMatchSample - 1].name);
  } else {
     // Middle of the list
    lcd.setCursor(0, 0);
    lcd.print("> ");
    lcd.print(samples[selectedMatchSample - 1].name);  // Current sample

    lcd.setCursor(0, 1);
    lcd.print("  ");
    lcd.print(samples[selectedMatchSample].name);  // Next sample
  }
}

void drawSampleList() {
  lcd.clear();

  if (selectedSample == sampleCount - 1 && sampleCount > 1) {
    lcd.setCursor(0, 0);
    lcd.print("  ");
    lcd.print(samples[selectedSample - 1].name);

    if (samples[selectedSample - 1].changes_tempo) {
      lcd.print("    *");
    }

    if (sampleLengthLocked && lockedSampleIndex == selectedSample - 1) {
      if (samples[selectedSample - 1].changes_tempo) {
        lcd.print("!");
      } else {
        lcd.print("     !");
      }
    }

    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.print(samples[selectedSample].name);

    if (samples[selectedSample].changes_tempo) {
      lcd.print("    *");
    }

    if (sampleLengthLocked && lockedSampleIndex == selectedSample) {
      if (samples[selectedSample].changes_tempo) {
        lcd.print("!");
      } else {
        lcd.print("     !");
      }
    }
  } else {
    lcd.setCursor(0, 0);
    lcd.print("> ");
    lcd.print(samples[selectedSample].name);

    if (samples[selectedSample].changes_tempo) {
      lcd.print("    *");
    }

    if (sampleLengthLocked && lockedSampleIndex == selectedSample) {
      if (samples[selectedSample].changes_tempo) {
        lcd.print("!");
      } else {
        lcd.print("     !");
      }
    }

    if (selectedSample + 1 < sampleCount) {
      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print(samples[selectedSample + 1].name);

      if (samples[selectedSample + 1].changes_tempo) {
        lcd.print("    *");
      }

      if (sampleLengthLocked && lockedSampleIndex == selectedSample + 1) {
        if (samples[selectedSample + 1].changes_tempo) {
          lcd.print("!");
        } else {
          lcd.print("     !");
        }
      }
    }
  }
}

void drawPitchBar() {
  lcd.setCursor(0, 1);

  int pixelEnd = map(playback_pitch, 32, 127, 0, TOTAL_PIXELS);

  for (int i = 0; i < BAR_CHARS; i++) {
    // draw a vertical guy in the relative pitch position
    int cellStart = i * 5;
    int cellEnd = cellStart + 5;
    byte glyph = ' ';

    if (pixelEnd >= cellStart && pixelEnd <= cellEnd) {
      glyph = 255;
    }

    lcd.write(glyph);
  }
}

void drawSliceBar(uint32_t start, uint32_t end) {
  lcd.setCursor(0, 1);

  int pixelStart = map(start, 0, totalSamplesInView, 0, TOTAL_PIXELS);
  int pixelEnd = map(end, 0, totalSamplesInView, 0, TOTAL_PIXELS);

  if (sampleLengthLocked) {
    int sampleLength = end - start;
    float pitchModifier = (float)64 / samples[lockedSampleIndex].pitch;
    float pitchModifiedSampleLength = sampleLength * pitchModifier;
    pixelEnd = map(start + (int)pitchModifiedSampleLength, 0, totalSamplesInView, 0, TOTAL_PIXELS);
  }

  /**
    int sampleLength = samples[sequencer_map[0].sample_id].end - samples[sequencer_map[0].sample_id].start;
            float pitchModifier = (float)64 / samples[sequencer_map[0].sample_id].pitch;
            float pitchModifiedSampleLength = sampleLength * pitchModifier;
            */

  for (int i = 0; i < BAR_CHARS; i++) {
    int cellStart = i * 5;
    int cellEnd = cellStart + 5;
    byte glyph = ' ';  // Default is empty space

    if (pixelEnd <= cellStart || pixelStart >= cellEnd) {
      glyph = ' ';  // Outside slice
    } else if (pixelStart <= cellStart && pixelEnd >= cellEnd) {
      glyph = 255;  // Full block
    } else {
      // Partial fill
      int fill = constrain(pixelEnd - cellStart, 0, 5);
      if (pixelStart > cellStart) {
        // Slice starts mid-cell → fill from right
        fill = constrain(cellEnd - pixelStart, 0, 5);
        glyph = (fill >= 5) ? 255 : 7 - fill;  // slots 4–6 are RTL
      } else {
        // Fill from left
        glyph = (fill >= 5) ? 255 : fill - 1;  // slots 0–3 are LTR
      }
    }

    lcd.write(glyph);
  }
}

uint16_t lastPot1 = 0;
uint16_t lastPot2 = 0;

void updateEditScreen() {
  uint16_t start = analogRead(A0);
  uint16_t end = analogRead(A1);

  if (abs(lastPot1 - start) < 100 && abs(lastPot2 - end) < 100) {
    return;
  }

  lastPot1 = start;
  lastPot2 = end;

  uint32_t mappedStart = map(start, 0, 4095, 0, totalSamplesInView);
  uint32_t mappedEnd = map(end, 0, 4095, 0, totalSamplesInView);

  if (sampleLengthLocked) {
    uint32_t adjustedLockedSampleLength = lockedSampleLength;

    if (sampleLengthDivision == 1) {
      adjustedLockedSampleLength = lockedSampleLength / 2;
    } else if (sampleLengthDivision == 2) {
      adjustedLockedSampleLength = lockedSampleLength / 4;
    }
    // speculative:
    // in the slice screen we could show a number in the upper right:
    // 1/1, 1/2, 1/4
    // DEFAULT TO 1/1
    // that determines what portion of the locked sample length we are using
    // if 1/1 doesn't fit, default to 1/2
    // if that doesn't fit, defualt to 1/4
    // don't switch to ones that don't fit
    // if none fit then...
    // if (lockedSampleLength > totalSamplesInView) {
    //   // make some auto adjustments to account for lack of room to fit sample
    //   // OR DIE
    // }
      // do the code to do the thing vvv
      
      // if (mappedStart + (lockedSampleLength / 2) < totalSamplesInView) {
      // } else {
      //   // not sure what to do here, we are at an impasse
      // }
      // FOR NOW, let's just ignore this case and pretend like it didn't happen.
    if (mappedStart + adjustedLockedSampleLength < totalSamplesInView) {
      // "regular" scenario
      mappedEnd = mappedStart + adjustedLockedSampleLength;
    } else {
      // can't go that far, move it back to the last "good" spot
      mappedStart = totalSamplesInView - adjustedLockedSampleLength;
      mappedEnd = totalSamplesInView;
    }
  }

  if (editInnerState == SLICE) {
    // sliceStart = map(start, 0, 4095, samples[selectedSample].start, samples[selectedSample].end);
    sliceStart = samples[selectedSample].start + mappedStart;
    if (sliceStart % 2 != 0) sliceStart++;
    sliceEnd = samples[selectedSample].start + mappedEnd;
    // sliceEnd = map(end, 0, 4095, samples[selectedSample].start, samples[selectedSample].end);
    if (sliceEnd % 2 != 0) sliceEnd++;
  } else {
    playback_pitch = map(end, 0, 4095, 32, 127);
  }

  // Serial.print("totalSamplesInView (the sample length): ");
  // Serial.println(totalSamplesInView);
  // Serial.print("locked? ");
  // Serial.println(sampleLengthLocked);
  // Serial.print("mappedStart: ");
  // Serial.println(mappedStart);
  // Serial.print("mappedEnd: ");
  // Serial.println(mappedEnd);
  // Serial.print("sliceStart: ");
  // Serial.println(sliceStart);
  // Serial.print("sliceEnd: ");
  // Serial.println(sliceEnd);

  if (mappedStart > mappedEnd) {
    uint16_t swap = mappedStart;
    mappedStart = mappedEnd;
    mappedEnd = swap;
  }

  lcd.clear();
  lcd.setCursor(0, 0);

  if (editInnerState == SLICE) {
    lcd.print("SLICE");
    if (sampleLengthLocked) {
      if (sampleLengthDivision == 0) {
        lcd.print("        1/1");
      } else if (sampleLengthDivision == 1) {
        lcd.print("        1/2");
      } else {
        lcd.print("        1/4");
      }
    }
    drawSliceBar(mappedStart, mappedEnd);
  } else {
    lcd.print("PITCH");
    drawPitchBar();
  }
}

void saveSliceFromLockedEdit() {
  if (sampleCount < MAX_SAMPLES) {
    Sample *s = &samples[sampleCount];
    snprintf(s->name, sizeof(s->name), "Sample %c", 'A' + sampleCount);
    s->start = savedSliceStart;
    s->end = savedSliceEnd;
    s->pitch = samples[lockedSampleIndex].pitch;
    sampleCount++;
  }

  selectedSample = sampleCount - 1;  // Auto-select new sample
}

void saveSliceFromEdit() {
  if (sampleCount < MAX_SAMPLES) {
    Sample *s = &samples[sampleCount];
    snprintf(s->name, sizeof(s->name), "Sample %c", 'A' + sampleCount);
    s->start = savedSliceStart;
    s->end = savedSliceEnd;
    s->pitch = playback_pitch;
    sampleCount++;
  }

  selectedSample = sampleCount - 1;  // Auto-select new sample
}

void enterEditStateForSample() {
  totalSamplesInView = samples[selectedSample].end - samples[selectedSample].start;

  sliceStart = samples[selectedSample].start;
  sliceEnd = samples[selectedSample].end;
  playback_pitch = samples[selectedSample].pitch;

  // Serial.print("selectedSample: ");
  // Serial.println(selectedSample);
  // Serial.print("totalSamplesInView: ");
  // Serial.println(totalSamplesInView);
  // Serial.print("start: ");
  // Serial.println(samples[selectedSample].start);
  // Serial.print("end: ");
  // Serial.println(samples[selectedSample].end);

  currentState = EDIT_STATE;
  updateEditScreen();
}

bool flipper = false;
long last_millis = 0;
bool modeButtonUsedAsModifier = false;
bool selectingSecondarySample = false;
bool displayingMessage = false;
int messageTime = 0;
void (*nextFunc)();

void loop() {
  now = millis();

  // Update all buttons
  recordButton.update();
  selectButton.update();
  upButton.update();
  downButton.update();
  modeButton.update();

  switch (currentState) {
    case HOME_STATE:
      {
        if (recordButton.pressed()) {
          startRecording();
        }
        if (upButton.pressed()) {
          if (selectedOption > 0) selectedOption--;
          drawOptionList();
        }
        if (downButton.pressed()) {
          if (selectedOption < optionCount - 1) selectedOption++;
          drawOptionList();
        }
        if (selectButton.pressed()) {
          enterSelectedOption();
        }
      }
      break;

    case RECORD_STATE:
      {
        updateRecordingScreen();
        if (recordButton.pressed()) {
          stopRecording();
        }
      }
      break;
    
    case SELECT_MATCH_STATE:
      {
        if (upButton.pressed()) {
          if (selectedMatchSample > 0) selectedMatchSample--;

          // Serial.println(selectedMatchSample);
          // Serial.println(sampleCount);
          // Serial.println("");

          drawSelectMatchList();
        }

        if (downButton.pressed()) {
          if (selectedMatchSample < sampleCount) selectedMatchSample++;

          // Serial.println(selectedMatchSample);
          // Serial.println(sampleCount);
          // Serial.println("");

          drawSelectMatchList();
        }

        if (selectButton.pressed()) {
          if (selectedMatchSample > 1) {
            // Serial.println(selectedMatchSample);

            uint8_t matchSample = selectedMatchSample - 1;
            uint32_t yourLen = samples[matchSample].end - samples[matchSample].start;
            uint32_t myLen = samples[selectedSample].end - samples[selectedSample].start;

            float pitchModifier = (float)64 / samples[matchSample].pitch;
            float pitchModifiedSampleLength = yourLen * pitchModifier;

            samples[selectedSample].matched_sample_id = matchSample;
            float floatLen = (float)myLen / pitchModifiedSampleLength;
            samples[selectedSample].matched_pitch = floatLen * 64;

            // Serial.println("selectedSample");
            // Serial.println(selectedSample);
            // Serial.println("matchSample");
            // Serial.println(matchSample);
            // Serial.println("selectedSample Len");
            // Serial.println(myLen);
            // Serial.println("matchSample Len");
            // Serial.println(yourLen);
            // Serial.println("matched_sample_id");
            // Serial.println(samples[selectedSample].matched_sample_id);
            // Serial.println("matched_pitch");
            // Serial.println(samples[selectedSample].matched_pitch);

            currentState = INDEX_STATE;
            drawSampleList();
          }
        }

        if (modeButton.pressed()) {
          currentState = INDEX_STATE;
          drawSampleList();
        }
      }
      break;

    case INDEX_STATE:
      {
        if (upButton.pressed()) {
          if (selectedSample > 0) selectedSample--;
          sliceStart = samples[selectedSample].start;
          sliceEnd = samples[selectedSample].end;
          drawSampleList();
        }

        if (downButton.pressed()) {
          if (modeButton.held()) {
            currentState = SELECT_MATCH_STATE;
            drawSelectMatchList();
          } else {
            if (selectedSample < sampleCount - 1) {
              selectedSample++;
              sliceStart = samples[selectedSample].start;
              sliceEnd = samples[selectedSample].end;
              drawSampleList();
            }
          }
        }

        if (selectButton.pressed()) {
          if (modeButton.held()) {
            modeButtonUsedAsModifier = true;
            samples[selectedSample].changes_tempo = !samples[selectedSample].changes_tempo;

            drawSampleList();
          } else {
            editInnerState = SLICE;
            enterEditStateForSample();
          }
        }

        if (recordButton.pressed()) {
          if (modeButton.held()) {
            modeButtonUsedAsModifier = true;

            if (sampleLengthLocked) {
              sampleLengthLocked = false;
              lockedSampleIndex = -1;
            } else {
              sampleLengthLocked = true;
              lockedSampleIndex = selectedSample;
              lockedSampleLength = samples[selectedSample].end - samples[selectedSample].start;
            }

            drawSampleList();
          } else {
            startRecording();
          }
        }

        if (modeButton.released()) {
          if (!modeButtonUsedAsModifier) {
            currentState = HOME_STATE;
            drawOptionList();
          }

          modeButtonUsedAsModifier = false;
        }
      }
      break;

    case EDIT_STATE:
      {
        updateEditScreen();

        if (recordButton.pressed()) {
          if (editInnerState == SLICE) {
            savedSliceStart = sliceStart;
            savedSliceEnd = sliceEnd;

            if (sampleLengthLocked) {
              saveSliceFromLockedEdit();
              currentState = INDEX_STATE;
              drawSampleList();
            } else {
              editInnerState = PITCH;
            }
          } else {
            saveSliceFromEdit();
            currentState = INDEX_STATE;
            drawSampleList();
          }
        }

        // if (upButton.pressed()) {
        //   loopMode = !loopMode;  // Toggle loop mode
        //   updateEditScreen();    // Update loop label
        // }

        if (downButton.pressed()) {
          currentState = INDEX_STATE;
          drawSampleList();
        }

        if (selectButton.pressed()) {
          if (modeButton.held()) {
            modeButtonUsedAsModifier = true;

            // change locked division
            if (sampleLengthLocked) {
              sampleLengthDivision++;

              if (sampleLengthDivision == 3) {
                sampleLengthDivision = 0;
              }
            }
          } else {
            playback_address = sliceStart;

            nextState = EDIT_STATE;
            currentState = PLAY_STATE;

            isPlayingSlice = true;
          }
        }

        if (modeButton.released()) {
          if (!modeButtonUsedAsModifier) {
            currentState = INDEX_STATE;
            drawSampleList();
          }

          modeButtonUsedAsModifier = false;
        }
      }
      break;

    case PLAY_STATE:
      {
        if (modeButton.pressed()) {
          playback_accumulator = 0;
          playback_address = sliceStart;
        }
      }
      break;

    case SAMPLER_STATE:
      {
        if (selectButton.pressed()) {
          edit_cursor_pos = cursorPos;  // Save current edit position
          currentState = SAMPLER_PLAYBACK_STATE;
        }

        if (upButton.pressed()) {
          cursorPos--;
          if (cursorPos == 255) cursorPos = 15;
          updateSequencerScreen();
        }

        if (downButton.pressed()) {
          if (modeButton.held()) {
            modeButtonUsedAsModifier = true;
            selectingSecondarySample = true;

            currentState = SELECT_SAMPLE_STATE;
            drawSelectSampleList();
          } else {
            cursorPos++;
            if (cursorPos == 16) cursorPos = 0;
            updateSequencerScreen();
          }
        }

        if (recordButton.pressed()) {
          if (modeButton.held()) {
            modeButtonUsedAsModifier = true;

            save_sequence();

            messageTime = now;
            nextFunc = updateSequencerScreen;
            displayingMessage = true;
            lcd.setCursor(0, 0);
            lcd.println("SEQUENCE SAVED!");
          } else {
            selectingSecondarySample = false;
            currentState = SELECT_SAMPLE_STATE;
            drawSelectSampleList();
          }
        }

        if (modeButton.released()) {
          if (!modeButtonUsedAsModifier) {
            selectingSecondarySample = false;
            currentState = HOME_STATE;
            drawOptionList();
          }

          modeButtonUsedAsModifier = false;
        }
      }
      break;

    case SELECT_SAMPLE_STATE:
      {
        // up the list
        if (upButton.pressed()) {
          if (sequencerSelectedSample > 0) sequencerSelectedSample--;

          if (sequencerSelectedSample > 0) {
            sliceStart = samples[sequencerSelectedSample - 1].start;
            sliceEnd = samples[sequencerSelectedSample - 1].end;
          }

          drawSelectSampleList();
        }

        // down the list
        if (downButton.pressed()) {
          if (sequencerSelectedSample < sampleCount) sequencerSelectedSample++;

          sliceStart = samples[sequencerSelectedSample - 1].start;
          sliceEnd = samples[sequencerSelectedSample - 1].end;

          drawSelectSampleList();
        }

        // preview the highlighted sample
        if (selectButton.pressed()) {
          if (sequencerSelectedSample > 0) {
            playback_pitch = samples[sequencerSelectedSample - 1].pitch;
            sliceStart = samples[sequencerSelectedSample - 1].start;
            sliceEnd = samples[sequencerSelectedSample - 1].end;

            playback_address = sliceStart;
            playback_accumulator = 0;

            nextState = SELECT_SAMPLE_STATE;
            currentState = PLAY_STATE;
            isPlayingSlice = true;
          }
        }

        // go back to sequencer with no changes
        if (modeButton.pressed()) {
          selectingSecondarySample = false;
          sequencer_tick_counter = samples_per_step;
          current_step = 0;
          currentState = SAMPLER_STATE;
          updateSequencerScreen();
        }

        // select the current list option ("Possibly 'NONE'"), set it in the sequence_map, return to sequencer
        if (recordButton.pressed()) {
          // select the sample
          // so modify the sequence_map here then go back
          // if NONE is selected, turn off triggering for this step
          // also need to keep track of secondary sample
          uint8_t primaryOrSecondarySample = selectingSecondarySample ? 1 : 0;

          if (sequencerSelectedSample == 0) {
            sequencer_map[primaryOrSecondarySample][cursorPos].triggered = false;

            // only reset tempo if primary sample is being modified
            if (cursorPos == 0 && !selectingSecondarySample) {
              samples_per_step = 1000;
            }
          } else {
            sequencer_map[primaryOrSecondarySample][cursorPos].sample_id = sequencerSelectedSample - 1;
            sequencer_map[primaryOrSecondarySample][cursorPos].triggered = true;
            
            if (cursorPos == 0 && !selectingSecondarySample) {
              if (samples[sequencerSelectedSample - 1].changes_tempo) {
                // length of this sample (end - start) / 32 steps per measure
                // everything operates at 16000 samples per second so record/playback/sequence all "work" together
                // if the sample rate changed from record to sequence then the calculation would be more complex
                // OOP but consider pitch
                // it's "adding to an accumulator" pitch not "simple multiplication" pitch so we have to subtract from "starting pitch"
                // all the parens here are just to visualize the situation
                int sampleLength = samples[sequencerSelectedSample - 1].end - samples[sequencerSelectedSample - 1].start;
                float pitchModifier = (float)64 / samples[sequencerSelectedSample - 1].pitch;
                float pitchModifiedSampleLength = sampleLength * pitchModifier;
                samples_per_step = (int)(pitchModifiedSampleLength / 32); // WHAT??
              } else {
                // this sample doesn't change the tempo, so we should really fall back to a global tempo setting.
                // right now that's just hardcoded at 120 bpm
                samples_per_step = 1000;
              }
            }
          }

          sequencer_tick_counter = samples_per_step;
          current_step = 0;
          currentState = SAMPLER_STATE;
          updateSequencerScreen();
        }
      }
      break;

      // for no one:
      // samples_per_measure / sample_rate => seconds_per_measure
      // seconds_per_measures / beats_per_measure => seconds_per_beat
      // beats_per_minute = 60 / seconds_per_beat

    case SAMPLER_PLAYBACK_STATE:
      { 
        if (selectButton.pressed()) {
          sequencer_tick_counter = samples_per_step;
          current_step = 0;
          currentState = SAMPLER_STATE;
          cursorPos = edit_cursor_pos;
          fastDigitalWriteLow(LED_PIN);
          updateSequencerScreen();
        }

        if (modeButton.isDown()) {
          // stutter = true;
          // swing = true;
          // if (!doubleTime) {
          //   saved_samples_per_step = samples_per_step;
          //   samples_per_step = samples_per_step / 2;
          //   doubleTime = true;
          // Serial.println(samples_per_step);
          // }
          if (!doubleTime) doubleTime = true;
        } else {
          // stutter = false;
          // swing = false;
          // if (doubleTime) {
          //   samples_per_step = saved_samples_per_step;
          //   doubleTime = false;
          //   Serial.println(samples_per_step);
          // }
          if (doubleTime) doubleTime = false;
        }

        if (needsUpdate) {
          updateSequencerScreen();
          needsUpdate = false;
        }
      }
      break;
    case SONG_STATE:
      {
        if (selectButton.pressed()) {
          edit_cursor_pos = songCursorPos;
          current_chain_index = 0;

          currentState = SONG_PLAYBACK_STATE;
        }

        if (upButton.pressed()) {
          songCursorPos--;
          if (songCursorPos == 255) songCursorPos = 31;  // Keep 32 for song chains
          updateSongScreen();
        }

        if (downButton.pressed()) {
          songCursorPos++;
          if (songCursorPos == 32) songCursorPos = 0;  // Keep 32 for song chains
          updateSongScreen();
        }

        if (recordButton.pressed()) {
          currentState = SELECT_SEQUENCE_STATE;
          drawSelectSequenceList();
        }

        if (modeButton.pressed()) {
          currentState = HOME_STATE;
          drawOptionList();
        }
      }
      break;
    case SONG_PLAYBACK_STATE:
      {
        if (selectButton.pressed()) {
          currentState = SONG_STATE;
          songCursorPos = edit_cursor_pos;
          fastDigitalWriteLow(LED_PIN);

          int sequence_id = sequence_chain[0];
          if (sequence_id != -1) {
            // when you stop playing the chain, restart from the beginning
            Sequence *seq = &sequence_bank[0][sequence_id];
            uint8_t id = seq->steps[0].sample_id;

            if (samples[id].changes_tempo) {
              int sampleLength = samples[id].end - samples[id].start;
              float pitchModifier = (float)64 / samples[id].pitch;
              float pitchModifiedSampleLength = sampleLength * pitchModifier;
              samples_per_step = (int)(pitchModifiedSampleLength / 32); // WHAT??
            } else {
              // this sample doesn't change the tempo, so we should really fall back to a global tempo setting.
              // right now that's just hardcoded at 120 bpm
              samples_per_step = 1000;
            }

            sequencer_tick_counter = samples_per_step;
            current_step = 0;
            current_chain_index = 0;
          }

          updateSongScreen();
        }

        // if (modeButton.isDown()) {
        //   // stutter = true;
        //   // swing = true;
        //   // if (!doubleTime) {
        //   //   saved_samples_per_step = samples_per_step;
        //   //   samples_per_step = samples_per_step / 2;
        //   //   doubleTime = true;
        //   // Serial.println(samples_per_step);
        //   // }
        //   if (!doubleTime) doubleTime = true;
        // } else {
        //   // stutter = false;
        //   // swing = false;
        //   // if (doubleTime) {
        //   //   samples_per_step = saved_samples_per_step;
        //   //   doubleTime = false;
        //   //   Serial.println(samples_per_step);
        //   // }
        //   if (doubleTime) doubleTime = false;
        // }

        if (needsUpdate) {
          updateSongScreen();
          needsUpdate = false;
        }
      }
      break;
    case SELECT_SEQUENCE_STATE:
      {
        // up the list
        if (upButton.pressed()) {
          if (songSelectedSequence > 0) songSelectedSequence--;

          // this type of code would be used for previewing a sample when selecting one for a sequence
          // we COULD do a mechanism to preview a sequence, it would be more involved.
          // probably a new state that is a sequence_preview_state and then modify the playback loop to
          // support it by setting the expected sequence before playing then stopping after one 
          //
          // if (songSelectedSequence > 0) {
          //   sliceStart = [songSelectedSequence - 1].start;
          //   sliceEnd = samples[songSelectedSequence - 1].end;
          // }

          drawSelectSequenceList();
        }

        // down the list
        if (downButton.pressed()) {
          if (songSelectedSequence < sequenceCount) songSelectedSequence++;

          // sliceStart = samples[sequencerSelectedSample - 1].start;
          // sliceEnd = samples[sequencerSelectedSample - 1].end;

          drawSelectSequenceList();
        }

        // None needs to be -1 for selecting a sequence

        // preview the highlighted SEQUENCE
        if (selectButton.pressed()) {
          if (songSelectedSequence > 0) {
            // playback_pitch = samples[sequencerSelectedSample - 1].pitch;
            // sliceStart = samples[sequencerSelectedSample - 1].start;
            // sliceEnd = samples[sequencerSelectedSample - 1].end;

            // playback_address = sliceStart;
            // playback_accumulator = 0;

            // nextState = SELECT_SAMPLE_STATE;
            // currentState = PLAY_STATE;
            // isPlayingSlice = true;
          }
        }

        // go back to song screen with no changes
        if (modeButton.pressed()) {
          sequencer_tick_counter = samples_per_step;
          current_step = 0;
          currentState = SONG_STATE;
          current_chain_index = 0;
          updateSongScreen();
        }

        // select the current list option ("Possibly 'NONE'"), set it in the sequence_map, return to sequencer
        if (recordButton.pressed()) {
          if (songSelectedSequence > 0) {
            sequence_chain[songCursorPos] = songSelectedSequence - 1; // have to subtract 1 because of the None option

            if (songCursorPos == 0) {
              Sequence *seq = &sequence_bank[0][songSelectedSequence - 1];
              uint8_t id = seq->steps[0].sample_id;

              if (samples[id].changes_tempo) {
                int sampleLength = samples[id].end - samples[id].start;
                float pitchModifier = (float)64 / samples[id].pitch;
                float pitchModifiedSampleLength = sampleLength * pitchModifier;
                samples_per_step = (int)(pitchModifiedSampleLength / 32); // WHAT??
              } else {
                // since this sample doesn't change the tempo we should really fall back to a global bpm
                // for now it's hardcoded to 120 bpm
                samples_per_step = 1000;
              }

              sequencer_tick_counter = samples_per_step;
              current_step = 0;
            }
          } else {
            sequence_chain[songCursorPos] = -1; // -1 is "no sequence" because it's just an index instead of a struct with an "enabled" flag or whatever
          }

          currentState = SONG_STATE;
          current_chain_index = 0;

          updateSongScreen();
        }
      }
      break;
  }

  updateCurrentStateIfNeeded();  // Leave as-is if useful
}