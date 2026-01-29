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
void write_ram_sample(uint32_t global_address, uint16_t sample);
void write_ram_sample_sequential(uint16_t sample);
void compressSample(uint32_t start, uint32_t end);
void select_chip(uint8_t chip);
void deselect_all_chips();
void set_ram_byte_mode();
void drawOptionList();
void drawSampleList();
void updateSequencerScreen();
void updateSongScreen();
void updateRecordingScreen();
void drawPlaybackScreen();
bool updateSliceAndPitchFromPots(bool forceUpdate = false);
void drawEditScreen();
void updateEditScreen(bool forceUpdate = false);

// Sequence cache system forward declarations
void initSequenceCache();
void requestSequenceRender(int sequence_id);
void renderSequenceToCache(int sequence_id);
bool isSequenceCached(int sequence_id);
void startCachedPlayback(int sequence_id);
void checkRenderRequests();  // Called from main loop
bool needsCaching(int sequence_id);  // Intelligent cache detection
bool currentSequenceNeedsCaching();  // Check if working sequencer_map needs caching

// Persistent metadata system forward declarations
void saveMetadataToSRAM();
bool loadMetadataFromSRAM();
void write_ram_bytes(uint32_t addr, uint8_t* data, uint32_t len);
void read_ram_bytes(uint32_t addr, uint8_t* data, uint32_t len);
uint8_t read_ram_byte(uint32_t global_address);
void write_ram_byte(uint32_t global_address, uint8_t value);

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
#define MAX_REALTIME_SAMPLES 2  // Safe limit for realtime SPI playback in interrupt

// Cache system constants (must be defined early for use in functions)
// With 512KB cache space and 112KB per sequence: 512KB / 112KB = 4.5, so we can fit 4 sequences comfortably
// But since we have extra headroom, let's support more sequences
#define MAX_CACHED_SEQUENCES 9  // 8 saved sequences + 1 temp slot
#define TEMP_CACHE_SLOT (MAX_CACHED_SEQUENCES - 1)  // Last slot (slot 8) reserved for temp/preview

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
    Serial.print("*** Saving sequence ");
    Serial.println(sequenceCount);
    
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < STEPS_PER_MEASURE; j++) {
        sequence_bank[i][sequenceCount].steps[j] = sequencer_map[i][j];
      }
    }

    // Request render for newly saved sequence if it's in cache slots
    // (but not the temp slot which is reserved for preview)
    if (sequenceCount < (MAX_CACHED_SEQUENCES - 1)) {
      Serial.print("*** Sequence ");
      Serial.print(sequenceCount);
      Serial.println(" is cacheable, requesting render");
      requestSequenceRender(sequenceCount);
    } else {
      Serial.print("*** Sequence ");
      Serial.print(sequenceCount);
      Serial.println(" is NOT cacheable (out of cache slots or reserved for temp)");
    }

    sequenceCount++;
    
    // Save metadata to persistent SRAM
    saveMetadataToSRAM();
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

ActiveSample active_samples[MAX_REALTIME_SAMPLES];

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

// ============================================================================
// SEQUENCE CACHE SYSTEM - Pre-rendered sequence playback
// ============================================================================

enum PlaybackMode {
  CACHE_MODE,      // Playing pre-rendered from cache
  REALTIME_MODE,   // Mixing on-the-fly (current method)
  RENDERING_MODE   // Cache is being built
};

// Cache metadata for each sequence slot
typedef struct {
  PlaybackMode mode;
  bool needs_render;
  uint32_t cache_address;  // Where in SPI RAM this sequence is cached
  uint32_t sample_count;   // How many samples in the cached sequence
} SequenceCacheInfo;

SequenceCacheInfo sequence_cache[MAX_SEQUENCES];

// Current playback state
PlaybackMode current_playback_mode = REALTIME_MODE;
uint32_t cached_playback_address = 0;
uint32_t cached_playback_tick = 0;
int currently_rendering_sequence = -1;
int currently_playing_sequence = -1;  // Track which sequence is playing
bool sequencer_map_modified = false;  // Track if sequencer_map changed since last cache render

// Communication between cores (for dual-core rendering)
// Core 0 (main) sets render_request and render_sequence_id
// Core 1 picks up the request and does the rendering
volatile bool render_request = false;
volatile int render_sequence_id = -1;
volatile bool core1_initialized = false;

// Mutex for safe SPI RAM access between cores
mutex_t spi_mutex;

// ============================================================================

uint32_t totalSamplesInView = 0;  // default placeholder
const int TOTAL_PIXELS = 16 * 5;
const int BAR_CHARS = 16;

bool loopMode = false;
uint32_t sliceStart = 0;
uint32_t sliceEnd = 0;

uint8_t playheadCharPos = 0;
uint8_t lastPlayheadCharPos = 255;

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

// Persistent metadata structures (stored in battery-backed SRAM)
typedef struct {
  uint32_t magic;  // Magic number to verify valid data (0xCAFEBABE)
  uint8_t version; // Metadata format version
  uint8_t sample_count;
  uint8_t sequence_count;
  uint8_t reserved;
} MetadataHeader;  // 8 bytes

typedef struct {
  char name[16];     // 16 bytes
  uint32_t start;    // 4 bytes
  uint32_t end;      // 4 bytes
  uint8_t pitch;     // 1 byte
  uint8_t beats;     // 1 byte
  uint8_t changes_tempo;  // 1 byte (bool)
  int8_t matched_sample_id;  // 1 byte
  uint8_t matched_pitch;     // 1 byte
  uint8_t reserved[3];       // 3 bytes padding for alignment
} SampleMetadata;  // 32 bytes per sample

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
const uint32_t CHUNK_SIZE = 0x80000;          // 512KB per chip (23LCV04M = 4Mbit)

// Memory Layout for Pre-rendering Cache System with 4× 23LCV04M chips
// Total RAM: 2MB (4 × 512KB)
const uint32_t SAMPLE_MEMORY_SIZE = 0x180000;  // 1536KB (1.5MB) for recorded samples (~48 seconds @ 16kHz)
const uint32_t CACHE_START = 0x180000;         // Cache starts at 1.5MB
const uint32_t CACHE_SIZE = 0x80000;           // 512KB for sequence cache
const uint32_t BYTES_PER_CACHED_SEQUENCE = 0xE000;  // 56KB per sequence slot (enough for 40 BPM: 1500 samples/step × 16 steps × 2 bytes = 48KB)

// Persistent metadata storage (with battery backup)
const uint32_t METADATA_SIZE = 0x2000;        // 8KB for metadata
const uint32_t METADATA_START = 0x200000 - METADATA_SIZE;  // At end of RAM: 0x1FE000
const uint32_t METADATA_MAGIC = 0xCAFEBABE;
const uint8_t METADATA_VERSION = 1;

uint32_t maxGlobalPosition = SAMPLE_MEMORY_SIZE;  // Limit recordings to 1.5MB
uint32_t global_position = 0;                     // now in BYTES
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

    // Optimized clipping detection - single branch for common case
    if (clipping_counter > 0) {
      clipping_counter--;
    } else {
      bool isClipping = (sample < 500) | (sample > 3500);  // Bitwise OR avoids branch
      if (isClipping) {
        clipping = true;
        clipping_counter = 500;
        fastDigitalWriteHigh(LED_PIN);
      } else if (clipping) {
        clipping = false;
        fastDigitalWriteLow(LED_PIN);
      }
    }

    write_ram_sample_sequential(sample);
    
  } else if (currentState == PLAY_STATE) {
    // Advance accumulator by pitch
    playback_accumulator += playback_pitch;

    // Truncate fractional bits to get sample index (shift is faster than divide)
    uint32_t sample_index = playback_accumulator >> 6;

    // Convert index to byte address - use shift instead of multiply
    playback_address = sliceStart + (sample_index << 1);

    // Calculate playhead position for display
    uint32_t sliceLength = sliceEnd - sliceStart;
    uint32_t playbackOffset = playback_address - sliceStart;
    playheadCharPos = map(playbackOffset, 0, sliceLength, 0, 15);
    if (playheadCharPos > 15) playheadCharPos = 15;

    // Read and write sample
    write_dac_sample(read_ram_sample(playback_address));

    // Loop or exit
    if (playback_address >= sliceEnd) {
      if (loopMode) {
        playback_accumulator = 0;
      } else {
        currentState = nextState;
        playback_accumulator = 0;
        needsUpdate = true;
      }
    }
    
  } else if (currentState == SAMPLER_PLAYBACK_STATE || currentState == SONG_PLAYBACK_STATE) {
    // Check if we should use cached playback
    bool use_cache = false;
    
    if (currentState == SAMPLER_PLAYBACK_STATE && currently_playing_sequence >= 0) {
      // Check if the current sequence is cached
      if (isSequenceCached(currently_playing_sequence)) {
        use_cache = true;
        
        // Debug output on first tick
        if (cached_playback_tick == 0) {
          Serial.print(">>> Using CACHED playback for sequence ");
          Serial.println(currently_playing_sequence);
        }
      } else if (cached_playback_tick == 0) {
        // Debug output on first tick for realtime mode
        Serial.print(">>> Using REALTIME playback for sequence ");
        Serial.println(currently_playing_sequence);
      }
    }
    
    // FAST PATH: Cached playback - just read pre-rendered audio!
    if (use_cache) {
      // Simple sequential read from cache
      SequenceCacheInfo *cache = &sequence_cache[currently_playing_sequence];
      
      // Pre-advance (matches realtime playback pattern)
      cached_playback_tick++;
      
      // Check for loop
      if (cached_playback_tick >= cache->sample_count) {
        cached_playback_tick = 0;
      }
      
      // Update sync/LED at appropriate intervals (match normal playback)
      uint32_t current_tick_in_step = cached_playback_tick % samples_per_step;
      if (current_tick_in_step == 0) {
        uint8_t step = (cached_playback_tick / samples_per_step) % STEPS_PER_MEASURE;
        
        // Debug output ONLY on the beat (every 4 steps when LED flashes)
        if ((step & 3) == 0) {
          Serial.print("Beat (step ");
          Serial.print(step);
          Serial.print("): tick=");
          Serial.print(cached_playback_tick);
          Serial.print("/");
          Serial.print(cache->sample_count);
          Serial.print(", addr=0x");
          Serial.println(cache->cache_address + (cached_playback_tick << 1), HEX);
        }
        
        // Generate SYNC_PIN pulses (same logic as realtime playback)
        if (swing && sync_pulse_counter == 0) {
          uint8_t step_mod_4 = step & 3;
          if (step_mod_4 == 0 || step_mod_4 == 2) {
            fastDigitalWriteHigh(SYNC_PIN);
            sync_pulse_counter = SYNC_PULSE_DURATION;
          }
        } else if (sync_pulse_counter == 0) {
          bool shouldPulse = doubleTime ? true : ((step & 1) == 0);
          if (shouldPulse) {
            fastDigitalWriteHigh(SYNC_PIN);
            sync_pulse_counter = SYNC_PULSE_DURATION;
          }
        }
        
        // LED and cursor updates
        if ((step & 3) == 0) {
          cursorPos = step;
          fastDigitalWriteHigh(LED_PIN);
          needsUpdate = true;
        } else if ((step & 3) == 2) {
          fastDigitalWriteLow(LED_PIN);
        }
      }
      
      // Decrement sync pulse counter and pull SYNC_PIN low when done
      if (sync_pulse_counter > 0) {
        sync_pulse_counter--;
        if (sync_pulse_counter == 0) {
          fastDigitalWriteLow(SYNC_PIN);
        }
      }
      
      // Now read and output the sample at current position
      uint32_t read_address = cache->cache_address + (cached_playback_tick << 1);
      uint16_t sample = read_ram_sample(read_address);
      write_dac_sample(sample);
      
      return true;  // Early exit - cached path is done!
    }
    
    // SLOW PATH: Real-time mixing (existing code)
    // Step progression
    sequencer_tick_counter++;
    if (sequencer_tick_counter >= samples_per_step) {
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
          
          // Update currently playing sequence for cache system
          currently_playing_sequence = sequence_chain[current_chain_index];
          if (currently_playing_sequence >= 0) {
            cached_playback_tick = sequence_cache[currently_playing_sequence].sample_count;  // Reset for new sequence, pre-advance pattern
          } else {
            cached_playback_tick = 0;  // No sequence
          }

          // recalc the samples_per_step MAYBE
          if (sequencer_map[0][0].triggered && samples[sequencer_map[0][0].sample_id].changes_tempo) {
            int sampleLength = samples[sequencer_map[0][0].sample_id].end - samples[sequencer_map[0][0].sample_id].start;
            float pitchModifier = (float)64 / samples[sequencer_map[0][0].sample_id].pitch;
            float pitchModifiedSampleLength = sampleLength * pitchModifier;
            samples_per_step = (int)(pitchModifiedSampleLength / 32); // WHAT??
          }
          
          current_chain_index = (current_chain_index + 1) % MAX_SEQUENCE_CHAIN;
        }

        for (int i = 0; i < MAX_REALTIME_SAMPLES; i++) {
          active_samples[i].active = false;
        }
      }

      if (swing && sync_pulse_counter == 0) {
        // Swing pattern - use bitwise AND instead of modulo
        // Steps 0,2,6,10,14 = steps where (step & 3) == 0 or 2, but not 4,8,12
        uint8_t step_mod_4 = current_step & 3;  // Fast modulo 4
        if (step_mod_4 == 0 || step_mod_4 == 2) {
          fastDigitalWriteHigh(SYNC_PIN);
          sync_pulse_counter = SYNC_PULSE_DURATION;
        }
      } else if (sync_pulse_counter == 0) {
        // Use bitwise AND instead of modulo for power-of-2 divisors
        bool shouldPulse = doubleTime ? true : ((current_step & 1) == 0);
        if (shouldPulse) {
          fastDigitalWriteHigh(SYNC_PIN);
          sync_pulse_counter = SYNC_PULSE_DURATION;
        }
      }

      // Use bitwise AND instead of modulo
      uint8_t step_mod_4 = current_step & 3;
      if (step_mod_4 == 0) {  // Every 4 steps = quarter note
        if (currentState == SAMPLER_PLAYBACK_STATE) {
          cursorPos = current_step;  // Show playback position only on quarter notes
        } else {
          if (current_step == 0) {
            songCursorPos = current_chain_index - 1;
          }
        }

        fastDigitalWriteHigh(LED_PIN);
        needsUpdate = true;
      } else if (step_mod_4 == 2) {  // Every 2 steps = eighth note (reuse step_mod_4)
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
      for (int i = 0; i < MAX_REALTIME_SAMPLES; i++) {
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

    // Optimized voice mixing loop
    for (int i = 0; i < MAX_REALTIME_SAMPLES; i++) {
      if (active_samples[i].active) {
        Sample *s = &samples[active_samples[i].sample_id];

        // Use shift instead of divide for index calculation
        uint32_t idx = active_samples[i].accumulator >> 6;
        // Use shift instead of multiply for address calculation
        uint32_t address = s->start + (idx << 1);

        if (address < s->end) {
          mixed_sample += read_ram_sample(address) - 2048;

          // Use matched_pitch if available, otherwise use pitch
          // Conditional move is faster than branch
          uint8_t pitch = s->matched_pitch ? s->matched_pitch : s->pitch;
          active_samples[i].accumulator += pitch;
        } else {
          active_samples[i].active = false;
        }
      }
    }

    // Clamp using branchless min/max pattern
    mixed_sample = (mixed_sample > 8191) ? 8191 : mixed_sample;
    mixed_sample = (mixed_sample < -8192) ? -8192 : mixed_sample;

    // Soft clip via lookup table
    int16_t clipped = tanh_table[mixed_sample + TANH_TABLE_OFFSET];

    // Convert to DAC range
    uint16_t dac_value = clipped + 2048;

    // Stutter effect - use bitwise AND instead of modulo
    if (stutter && (current_step & 1) != 0) {
      dac_value = 2048;
    }

    write_dac_sample(dac_value);
    
  } else if (currentState != EDIT_STATE) {
    uint16_t sample = analogRead(A2);

    // Optimized clipping detection - same as recording path
    if (clipping_counter > 0) {
      clipping_counter--;
    } else {
      bool isClipping = (sample < 500) | (sample > 3500);  // Bitwise OR avoids branch
      if (isClipping) {
        clipping = true;
        clipping_counter = 500;
        fastDigitalWriteHigh(LED_PIN);
      } else if (clipping) {
        clipping = false;
        fastDigitalWriteLow(LED_PIN);
      }
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

// ============================================================================
// PERSISTENT METADATA SYSTEM (Battery-Backed SRAM)
// ============================================================================

// Helper: Read single byte from SRAM
uint8_t read_ram_byte(uint32_t global_address) {
  uint8_t chip = global_address / CHUNK_SIZE;
  uint32_t local_address = global_address % CHUNK_SIZE;
  
  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x03);  // READ command
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  uint8_t value = SPI.transfer(0x00);
  SPI.endTransaction();
  deselect_all_chips();
  
  return value;
}

// Helper: Write single byte to SRAM
void write_ram_byte(uint32_t global_address, uint8_t value) {
  uint8_t chip = global_address / CHUNK_SIZE;
  uint32_t local_address = global_address % CHUNK_SIZE;
  
  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x02);  // WRITE command
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  SPI.transfer(value);
  SPI.endTransaction();
  deselect_all_chips();
}

// Helper: Write multiple bytes to SRAM
void write_ram_bytes(uint32_t addr, uint8_t* data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    write_ram_byte(addr + i, data[i]);
  }
}

// Helper: Read multiple bytes from SRAM
void read_ram_bytes(uint32_t addr, uint8_t* data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    data[i] = read_ram_byte(addr + i);
  }
}

// Save all sample and sequence metadata to persistent SRAM
void saveMetadataToSRAM() {
  Serial.println("*** Saving metadata to persistent SRAM...");
  
  uint32_t addr = METADATA_START;
  
  // Write header
  MetadataHeader header;
  header.magic = METADATA_MAGIC;
  header.version = METADATA_VERSION;
  header.sample_count = sampleCount;
  header.sequence_count = sequenceCount;
  header.reserved = 0;
  
  write_ram_bytes(addr, (uint8_t*)&header, sizeof(MetadataHeader));
  addr += sizeof(MetadataHeader);
  
  Serial.print("  Saving ");
  Serial.print(sampleCount);
  Serial.println(" samples...");
  
  // Write samples metadata
  for (int i = 0; i < sampleCount; i++) {
    SampleMetadata meta;
    strncpy(meta.name, samples[i].name, 16);
    meta.start = samples[i].start;
    meta.end = samples[i].end;
    meta.pitch = samples[i].pitch;
    meta.beats = samples[i].beats;
    meta.changes_tempo = samples[i].changes_tempo ? 1 : 0;
    meta.matched_sample_id = samples[i].matched_sample_id;
    meta.matched_pitch = samples[i].matched_pitch;
    memset(meta.reserved, 0, 3);
    
    write_ram_bytes(addr, (uint8_t*)&meta, sizeof(SampleMetadata));
    addr += sizeof(SampleMetadata);
  }
  
  Serial.print("  Saving ");
  Serial.print(sequenceCount);
  Serial.println(" sequences...");
  
  // Write sequences metadata
  for (int seq = 0; seq < sequenceCount; seq++) {
    for (int row = 0; row < 2; row++) {
      write_ram_bytes(addr, (uint8_t*)&sequence_bank[row][seq], sizeof(Sequence));
      addr += sizeof(Sequence);
    }
  }
  
  Serial.println("*** Metadata saved! (will persist with battery backup)");
}

// Load sample and sequence metadata from persistent SRAM
bool loadMetadataFromSRAM() {
  Serial.println("*** Checking for persistent metadata...");
  
  uint32_t addr = METADATA_START;
  
  // Read and verify header
  MetadataHeader header;
  read_ram_bytes(addr, (uint8_t*)&header, sizeof(MetadataHeader));
  addr += sizeof(MetadataHeader);
  
  if (header.magic != METADATA_MAGIC) {
    Serial.println("*** No valid metadata found (cold boot or first use)");
    Serial.print("    Expected magic: 0x");
    Serial.print(METADATA_MAGIC, HEX);
    Serial.print(", got: 0x");
    Serial.println(header.magic, HEX);
    return false;
  }
  
  if (header.version != METADATA_VERSION) {
    Serial.println("*** Metadata version mismatch, ignoring");
    Serial.print("    Expected version: ");
    Serial.print(METADATA_VERSION);
    Serial.print(", got: ");
    Serial.println(header.version);
    return false;
  }
  
  Serial.println("*** Valid metadata found! Restoring...");
  
  // Restore samples
  sampleCount = header.sample_count;
  Serial.print("  Restoring ");
  Serial.print(sampleCount);
  Serial.println(" samples...");
  
  for (int i = 0; i < sampleCount; i++) {
    SampleMetadata meta;
    read_ram_bytes(addr, (uint8_t*)&meta, sizeof(SampleMetadata));
    addr += sizeof(SampleMetadata);
    
    strncpy(samples[i].name, meta.name, 16);
    samples[i].name[15] = '\0';  // Ensure null termination
    samples[i].start = meta.start;
    samples[i].end = meta.end;
    samples[i].pitch = meta.pitch;
    samples[i].beats = meta.beats;
    samples[i].changes_tempo = meta.changes_tempo != 0;
    samples[i].matched_sample_id = meta.matched_sample_id;
    samples[i].matched_pitch = meta.matched_pitch;
  }
  
  // Restore sequences
  sequenceCount = header.sequence_count;
  Serial.print("  Restoring ");
  Serial.print(sequenceCount);
  Serial.println(" sequences...");
  
  for (int seq = 0; seq < sequenceCount; seq++) {
    for (int row = 0; row < 2; row++) {
      read_ram_bytes(addr, (uint8_t*)&sequence_bank[row][seq], sizeof(Sequence));
      addr += sizeof(Sequence);
    }
  }
  
  Serial.print("*** Successfully restored ");
  Serial.print(sampleCount);
  Serial.print(" samples and ");
  Serial.print(sequenceCount);
  Serial.println(" sequences from battery-backed SRAM!");
  
  return true;
}

// ============================================================================
// DUAL-CORE SUPPORT - Core 1 handles background rendering
// ============================================================================

// TEMPORARILY DISABLED: Core 1 async rendering
// Moving to synchronous rendering on Core 0 for initial testing
/*
void setup1() {
  // Core 1 initialization
  mutex_init(&spi_mutex);
  core1_initialized = true;
  
  Serial.println(">>> Core 1 initialized and ready for rendering!");
}

void loop1() {
  // Core 1 main loop - checks for render requests
  if (render_request && render_sequence_id >= 0) {
    int seq_id = render_sequence_id;
    
    Serial.print(">>> Core 1: Starting render for sequence ");
    Serial.println(seq_id);
    
    // Acquire mutex for safe SPI access
    mutex_enter_blocking(&spi_mutex);
    
    // Do the actual rendering
    renderSequenceToCache(seq_id);
    
    // Update cache status
    sequence_cache[seq_id].mode = CACHE_MODE;
    sequence_cache[seq_id].needs_render = false;
    
    // Release mutex
    mutex_exit(&spi_mutex);
    
    Serial.print(">>> Core 1: Finished rendering sequence ");
    Serial.println(seq_id);
    
    // Clear the request
    render_request = false;
    render_sequence_id = -1;
  }
  
  // Small yield to avoid busy-waiting
  tight_loop_contents();
}
*/

// ============================================================================

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
  
  // Initialize sequence cache system
  initSequenceCache();
  
  // Try to restore samples and sequences from battery-backed SRAM
  if (loadMetadataFromSRAM()) {
    // Metadata restored successfully! Optionally re-render cached sequences
    Serial.println("*** Checking which restored sequences need caching...");
    for (int i = 0; i < min(sequenceCount, MAX_CACHED_SEQUENCES); i++) {
      if (needsCaching(i)) {
        Serial.print("  Sequence ");
        Serial.print(i);
        Serial.println(" needs cache, requesting render...");
        requestSequenceRender(i);
      }
    }
  } else {
    Serial.println("*** Starting fresh (no persistent data found)");
  }

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

// Byte mode write: writes a single 16-bit sample at the specified address
// Requires two separate transactions (one per byte) to respect byte mode
void write_ram_sample(uint32_t global_address, uint16_t sample) {
  uint8_t chip = global_address / CHUNK_SIZE;
  uint32_t local_address = global_address % CHUNK_SIZE;

  // Write first byte (upper byte) - COMPLETE transaction
  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(2);  // WRITE opcode
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  SPI.transfer((sample >> 8) & 0xFF);  // Upper byte
  SPI.endTransaction();
  deselect_all_chips();  // CS HIGH - complete transaction

  // Write second byte (lower byte) - NEW transaction
  global_address++;
  chip = global_address / CHUNK_SIZE;
  local_address = global_address % CHUNK_SIZE;
  
  select_chip(chip);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(2);  // WRITE opcode
  SPI.transfer((local_address >> 16) & 0xFF);
  SPI.transfer((local_address >> 8) & 0xFF);
  SPI.transfer(local_address & 0xFF);
  SPI.transfer(sample & 0xFF);  // Lower byte
  SPI.endTransaction();
  deselect_all_chips();  // CS HIGH - complete transaction
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

  // if (global_position % 2 != 0) {
  //   global_position++;  // Ensure even byte alignment
  // }

  if (sampleCount < MAX_SAMPLES) {
    Sample *s = &samples[sampleCount];
    snprintf(s->name, sizeof(s->name), "Sample %c", 'A' + sampleCount);
    s->start = starting_record_position;
    s->end = global_position;

    // if (s->start % 2 != 0) {
    //   s->start++;  // Ensure even byte alignment
    // }

    // if (s->end % 2 != 0) {
    //   s->end++;  // Ensure even byte alignment
    // }

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

    // Apply compression to the recorded sample
    // Serial.println(">>> Applying compression...");
    // ITimer.disableTimer();
    // compressSample(s->start, s->end);
    // ITimer.enableTimer();
    
    // Save metadata to persistent SRAM
    saveMetadataToSRAM();
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

// Upward compression: brings quiet sounds up while leaving loud sounds unchanged
// Uses square root curve for smooth, natural-sounding compression
void compressSample(uint32_t start, uint32_t end) {
  for (uint32_t addr = start; addr < end; addr += 2) {
    uint16_t sample = read_ram_sample(addr);
    
    // Convert to signed 12-bit (-2048 to 2047)
    int16_t signedSample = sample - 2048;
    
    // Normalize to -1.0 to 1.0
    float normalized = signedSample / 2048.0f;
    
    // Apply compression curve using square root
    // Quiet sounds get boosted more than loud sounds
    float sign = (normalized >= 0.0f) ? 1.0f : -1.0f;
    float absNormalized = (normalized >= 0.0f) ? normalized : -normalized;
    
    // Square root compression: 0.1 → 0.316 (3x), 0.5 → 0.707 (1.4x), 1.0 → 1.0 (1x)
    float compressed = sign * sqrtf(absNormalized);
    
    // Convert back to signed 12-bit
    signedSample = (int16_t)(compressed * 2047.0f);
    
    // Clamp to valid range (safety)
    if (signedSample > 2047) signedSample = 2047;
    if (signedSample < -2048) signedSample = -2048;
    
    // Convert back to unsigned and write
    write_ram_sample(addr, (uint16_t)(signedSample + 2048));
  }
}

// ============================================================================
// SEQUENCE CACHE SYSTEM IMPLEMENTATION
// ============================================================================

// Intelligent detection: Does this sequence need caching?
// Returns true if the sequence would exceed MAX_REALTIME_SAMPLES simultaneous voices
bool needsCaching(int sequence_id) {
  if (sequence_id < 0 || sequence_id >= MAX_SEQUENCES) return false;
  
  Serial.println("=== needsCaching() Analysis ===");
  Serial.print("Sequence ID: ");
  Serial.println(sequence_id);
  Serial.print("samples_per_step: ");
  Serial.println(samples_per_step);
  
  int max_simultaneous = 0;
  int max_simultaneous_step = -1;
  
  // Simulate each step in the sequence
  for (int step = 0; step < STEPS_PER_MEASURE; step++) {
    int active_count = 0;
    
    // Check all previous steps to see if their samples are still playing
    for (int check_step = 0; check_step <= step; check_step++) {
      for (int row = 0; row < 2; row++) {
        if (sequence_bank[row][sequence_id].steps[check_step].triggered) {
          uint8_t sample_id = sequence_bank[row][sequence_id].steps[check_step].sample_id;
          
          // How many steps ago did this sample start?
          uint32_t steps_elapsed = step - check_step;
          uint32_t samples_elapsed = steps_elapsed * samples_per_step;
          
          // How long is this sample (in samples)?
          uint32_t sample_length_bytes = samples[sample_id].end - samples[sample_id].start;
          uint32_t sample_length = sample_length_bytes / 2;  // Convert bytes to samples
          
          // Adjust for pitch (pitch > 64 plays faster = shorter duration)
          uint8_t pitch = samples[sample_id].pitch;
          uint32_t actual_duration = (sample_length << 6) / pitch;  // Multiply by 64, divide by pitch
          
          // Is this sample still playing at the current step?
          if (samples_elapsed < actual_duration) {
            active_count++;
            
            // Log first few overlaps for debugging
            if (step < 4 || active_count > 2) {
              Serial.print("  Step ");
              Serial.print(step);
              Serial.print(": Sample ");
              Serial.print(sample_id);
              Serial.print(" from step ");
              Serial.print(check_step);
              Serial.print(" still playing (elapsed: ");
              Serial.print(samples_elapsed);
              Serial.print(", duration: ");
              Serial.print(actual_duration);
              Serial.print(", length_bytes: ");
              Serial.print(sample_length_bytes);
              Serial.print(", pitch: ");
              Serial.print(pitch);
              Serial.println(")");
            }
          }
        }
      }
    }
    
    if (active_count > max_simultaneous) {
      max_simultaneous = active_count;
      max_simultaneous_step = step;
    }
  }
  
  Serial.print("Max simultaneous voices: ");
  Serial.print(max_simultaneous);
  Serial.print(" (at step ");
  Serial.print(max_simultaneous_step);
  Serial.println(")");
  Serial.print("MAX_REALTIME_SAMPLES limit: ");
  Serial.println(MAX_REALTIME_SAMPLES);
  Serial.print("Needs caching: ");
  Serial.println(max_simultaneous > MAX_REALTIME_SAMPLES ? "YES" : "NO");
  Serial.println("=== End Analysis ===");
  
  // Need caching if we ever exceed the realtime voice limit
  return max_simultaneous > MAX_REALTIME_SAMPLES;
}

// Check if the current working sequencer_map needs caching (for preview)
bool currentSequenceNeedsCaching() {
  Serial.println("=== currentSequenceNeedsCaching() Analysis ===");
  Serial.print("samples_per_step: ");
  Serial.println(samples_per_step);
  
  int max_simultaneous = 0;
  int max_simultaneous_step = -1;
  
  // Simulate each step in the current sequencer_map
  for (int step = 0; step < STEPS_PER_MEASURE; step++) {
    int active_count = 0;
    
    // Check all previous steps to see if their samples are still playing
    for (int check_step = 0; check_step <= step; check_step++) {
      for (int row = 0; row < 2; row++) {
        if (sequencer_map[row][check_step].triggered) {
          uint8_t sample_id = sequencer_map[row][check_step].sample_id;
          
          // How many steps ago did this sample start?
          uint32_t steps_elapsed = step - check_step;
          uint32_t samples_elapsed = steps_elapsed * samples_per_step;
          
          // How long is this sample (in samples)?
          uint32_t sample_length_bytes = samples[sample_id].end - samples[sample_id].start;
          uint32_t sample_length = sample_length_bytes / 2;  // Convert bytes to samples
          
          // Adjust for pitch (pitch > 64 plays faster = shorter duration)
          uint8_t pitch = samples[sample_id].pitch;
          uint32_t actual_duration = (sample_length << 6) / pitch;  // Multiply by 64, divide by pitch
          
          // Is this sample still playing at the current step?
          if (samples_elapsed < actual_duration) {
            active_count++;
            
            // Log first few overlaps for debugging
            if (step < 4 || active_count > 2) {
              Serial.print("  Step ");
              Serial.print(step);
              Serial.print(": Sample ");
              Serial.print(sample_id);
              Serial.print(" from step ");
              Serial.print(check_step);
              Serial.print(" still playing (elapsed: ");
              Serial.print(samples_elapsed);
              Serial.print(", duration: ");
              Serial.print(actual_duration);
              Serial.print(", length_bytes: ");
              Serial.print(sample_length_bytes);
              Serial.print(", pitch: ");
              Serial.print(pitch);
              Serial.println(")");
            }
          }
        }
      }
    }
    
    if (active_count > max_simultaneous) {
      max_simultaneous = active_count;
      max_simultaneous_step = step;
    }
  }
  
  Serial.print("Max simultaneous voices: ");
  Serial.print(max_simultaneous);
  Serial.print(" (at step ");
  Serial.print(max_simultaneous_step);
  Serial.println(")");
  Serial.print("MAX_REALTIME_SAMPLES limit: ");
  Serial.println(MAX_REALTIME_SAMPLES);
  Serial.print("Needs caching: ");
  Serial.println(max_simultaneous > MAX_REALTIME_SAMPLES ? "YES" : "NO");
  Serial.println("=== End Analysis ===");
  
  // Need caching if we ever exceed the realtime voice limit
  return max_simultaneous > MAX_REALTIME_SAMPLES;
}

// Initialize cache metadata for all sequences
void initSequenceCache() {
  for (int i = 0; i < MAX_SEQUENCES; i++) {
    sequence_cache[i].mode = REALTIME_MODE;
    sequence_cache[i].needs_render = false;
    sequence_cache[i].cache_address = CACHE_START + (i * BYTES_PER_CACHED_SEQUENCE);
    sequence_cache[i].sample_count = 0;
  }
  
  Serial.println("=== Sequence Cache Initialized ===");
  Serial.print("Sample memory: 0x0 - 0x");
  Serial.println(SAMPLE_MEMORY_SIZE, HEX);
  Serial.print("Cache start: 0x");
  Serial.println(CACHE_START, HEX);
  Serial.print("Cache size: ");
  Serial.print(CACHE_SIZE / 1024);
  Serial.println(" KB");
  Serial.print("Max cached sequences: ");
  Serial.println(MAX_CACHED_SEQUENCES);
}

// Check if a sequence is cached and ready to play
bool isSequenceCached(int sequence_id) {
  if (sequence_id < 0 || sequence_id >= MAX_SEQUENCES) return false;
  return sequence_cache[sequence_id].mode == CACHE_MODE;
}

// Get human-readable cache status
const char* getCacheStatus(int sequence_id) {
  if (sequence_id < 0 || sequence_id >= MAX_SEQUENCES) return "???";
  if (sequence_id >= MAX_CACHED_SEQUENCES) return "N/A";
  
  switch (sequence_cache[sequence_id].mode) {
    case CACHE_MODE: return "CACHED";
    case RENDERING_MODE: return "RENDER";
    case REALTIME_MODE: return "LIVE";
    default: return "???";
  }
}

// Request a sequence to be rendered (will happen in background)
void requestSequenceRender(int sequence_id) {
  if (sequence_id < 0 || sequence_id >= MAX_SEQUENCES) return;
  if (sequence_id >= MAX_CACHED_SEQUENCES) return;  // Only cache first N sequences
  
  Serial.print("*** Requesting render for sequence ");
  Serial.println(sequence_id);
  
  sequence_cache[sequence_id].mode = RENDERING_MODE;
  sequence_cache[sequence_id].needs_render = true;
  
  Serial.print("*** Sequence ");
  Serial.print(sequence_id);
  Serial.println(" marked for rendering");
}

// Render the current working sequence to temp cache slot (for preview)
void renderCurrentSequenceToCache() {
  Serial.println("*** Rendering current working sequence to temp cache...");
  
  // CRITICAL: Stop the timer interrupt to avoid SPI bus conflicts!
  ITimer.stopTimer();
  Serial.println("*** Timer interrupt STOPPED for rendering");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Rendering...");
  
  // Temporarily save current sequencer_map to temp slot
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < STEPS_PER_MEASURE; j++) {
      sequence_bank[i][TEMP_CACHE_SLOT].steps[j] = sequencer_map[i][j];
    }
  }
  
  // Render it
  renderSequenceToCache(TEMP_CACHE_SLOT);

  // Mark as cached and ready
  sequence_cache[TEMP_CACHE_SLOT].mode = CACHE_MODE;
  sequence_cache[TEMP_CACHE_SLOT].needs_render = false;
  
  // Clear the modified flag since we just rendered
  sequencer_map_modified = false;
  
  // CRITICAL: Restart the timer interrupt
  ITimer.restartTimer();
  Serial.println("*** Timer interrupt RESTARTED");
  
  Serial.println("*** Temp cache render complete!");
}

// Check for pending render requests (called from main loop)
void checkRenderRequests() {
  // Check if Core 1 is available and not busy
  if (!core1_initialized || render_request) {
    return;  // Core 1 not ready or already rendering
  }
  
  // Find a sequence that needs rendering
  for (int i = 0; i < MAX_CACHED_SEQUENCES; i++) {
    if (sequence_cache[i].needs_render && sequence_cache[i].mode == RENDERING_MODE) {
      // Send render request to Core 1 (non-blocking!)
      render_sequence_id = i;
      render_request = true;
      
      Serial.print(">>> Core 0: Requesting Core 1 to render sequence ");
      Serial.println(i);
      
      break;  // Only request one at a time
    }
  }
}

// Render a complete sequence to cache memory
void renderSequenceToCache(int sequence_id) {
  if (sequence_id < 0 || sequence_id >= MAX_SEQUENCES) {
    Serial.println("ERROR: Invalid sequence_id!");
    return;
  }
  
  Serial.println("========================================");
  Serial.print("=== Rendering sequence ");
  Serial.print(sequence_id);
  Serial.print(" to cache (TEMP_CACHE_SLOT=");
  Serial.print(TEMP_CACHE_SLOT);
  Serial.println(") ===");
  
  unsigned long start_time = millis();
  
  // Load the sequence we want to render
  load_sequence(sequence_id);
  Serial.println("Sequence loaded into sequencer_map");
  
  // Calculate samples_per_step based on changes_tempo flag (like normal playback does)
  if (sequencer_map[0][0].triggered && samples[sequencer_map[0][0].sample_id].changes_tempo) {
    int sampleLength = samples[sequencer_map[0][0].sample_id].end - samples[sequencer_map[0][0].sample_id].start;
    float pitchModifier = (float)64 / samples[sequencer_map[0][0].sample_id].pitch;
    float pitchModifiedSampleLength = sampleLength * pitchModifier;
    samples_per_step = (int)(pitchModifiedSampleLength / 32);
    Serial.print("*** Tempo from sample: samples_per_step = ");
    Serial.println(samples_per_step);
  } else {
    samples_per_step = 1000;  // Default 120 BPM
    Serial.println("*** Using default tempo: samples_per_step = 1000");
  }
  
  SequenceCacheInfo *cache = &sequence_cache[sequence_id];
  uint32_t write_address = cache->cache_address;
  uint32_t sample_count = 0;
  
  Serial.print("Cache address: 0x");
  Serial.println(write_address, HEX);
  Serial.print("samples_per_step: ");
  Serial.println(samples_per_step);
  
  set_ram_byte_mode();
  
  // Simulate the entire sequence playback
  uint32_t tick = 0;
  uint8_t step = 0;
  
  // Reset active samples at start
  for (int i = 0; i < MAX_REALTIME_SAMPLES; i++) {
    active_samples[i].active = false;
  }
  Serial.println("Active samples reset");
  
  // Render each tick of the sequence
  uint32_t total_ticks = STEPS_PER_MEASURE * samples_per_step;
  Serial.print("Total ticks to render: ");
  Serial.println(total_ticks);
  
  uint16_t first_5_written[5] = {0, 0, 0, 0, 0};  // Track first 5 for debugging
  
  for (tick = 0; tick < total_ticks; tick++) {
    // Step progression
    if (tick % samples_per_step == 0) {
      step = tick / samples_per_step;
      
      // Trigger samples for this step (match realtime code logic)
      SequencerStep *primaryStep = &sequencer_map[0][step];
      SequencerStep *secondaryStep = &sequencer_map[1][step];
      bool needPrimarySlot = primaryStep->triggered;
      bool needSecondarySlot = secondaryStep->triggered;
      int8_t primarySlot = -1;
      int8_t secondarySlot = -1;
      
      // Single pass to find slots and handle retriggering
      for (int i = 0; i < MAX_REALTIME_SAMPLES; i++) {
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
    }
    
    // Mix all active voices
    int32_t mixed_sample = 0;
    int active_voice_count = 0;  // Track how many voices are active
    
    for (int i = 0; i < MAX_REALTIME_SAMPLES; i++) {
      if (active_samples[i].active) {
        active_voice_count++;
        Sample *s = &samples[active_samples[i].sample_id];
        
        uint32_t idx = active_samples[i].accumulator >> 6;
        uint32_t address = s->start + (idx << 1);
        
        if (address < s->end) {
          mixed_sample += read_ram_sample(address) - 2048;
          
          uint8_t pitch = s->matched_pitch ? s->matched_pitch : s->pitch;
          active_samples[i].accumulator += pitch;
        } else {
          // Sample finished
          if (tick < total_ticks - 100) {  // Only log if not near the end
            Serial.print("Voice ");
            Serial.print(i);
            Serial.print(" (sample ");
            Serial.print(active_samples[i].sample_id);
            Serial.print(") finished at tick ");
            Serial.print(tick);
            Serial.print(" (step ");
            Serial.print(step);
            Serial.println(")");
          }
          active_samples[i].active = false;
          active_voice_count--;
        }
      }
    }
    
    // Debug: Check if samples are still active at various points
    if (tick == total_ticks / 4) {
      Serial.print("At 25% (tick ");
      Serial.print(tick);
      Serial.print("): ");
      Serial.print(active_voice_count);
      Serial.println(" voices active");
    }
    if (tick == total_ticks / 2) {
      Serial.print("At 50% (tick ");
      Serial.print(tick);
      Serial.print("): ");
      Serial.print(active_voice_count);
      Serial.println(" voices active");
    }
    if (tick == (total_ticks * 3) / 4) {
      Serial.print("At 75% (tick ");
      Serial.print(tick);
      Serial.print("): ");
      Serial.print(active_voice_count);
      Serial.println(" voices active");
    }
    
    // Clamp to tanh table range
    mixed_sample = (mixed_sample > 8191) ? 8191 : mixed_sample;
    mixed_sample = (mixed_sample < -8192) ? -8192 : mixed_sample;
    
    // Soft clip
    int16_t clipped = tanh_table[mixed_sample + TANH_TABLE_OFFSET];
    
    // Convert to DAC range
    uint16_t dac_value = clipped + 2048;
    
    // Track first few values for debugging
    if (sample_count < 5) {
      first_5_written[sample_count] = dac_value;
    }
    
    // Write to cache using byte-mode (not sequential)
    uint32_t cache_write_address = write_address + (sample_count << 1);
    
    // Debug first few writes
    if (sample_count < 5) {
      Serial.print("Write #");
      Serial.print(sample_count);
      Serial.print(": addr=0x");
      Serial.print(cache_write_address, HEX);
      Serial.print(" val=");
      Serial.println(dac_value);
    }
    
    write_ram_sample(cache_write_address, dac_value);
    sample_count++;
  }
  
  Serial.println("Byte-mode writes complete");
  
  // Update cache metadata
  cache->sample_count = sample_count;
  
  unsigned long render_time = millis() - start_time;
  Serial.print(">>> Rendered ");
  Serial.print(sample_count);
  Serial.print(" samples in ");
  Serial.print(render_time);
  Serial.println(" ms");
  Serial.print(">>> Expected duration: ");
  Serial.print((sample_count * 1000) / SAMPLE_RATE);
  Serial.println(" ms of audio");
  Serial.print(">>> Final cache address: 0x");
  Serial.println(cache->cache_address, HEX);
  Serial.print(">>> Cache sample count: ");
  Serial.println(cache->sample_count);
  
  // Verify cache metadata
  Serial.print(">>> Cache mode: ");
  Serial.println(cache->mode == CACHE_MODE ? "CACHE_MODE" : (cache->mode == RENDERING_MODE ? "RENDERING_MODE" : "REALTIME_MODE"));
  Serial.print(">>> Cache address stored: 0x");
  Serial.println(cache->cache_address, HEX);
  
  // Show what we wrote vs what we read back
  Serial.print(">>> First 5 WRITTEN: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(first_5_written[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Verify first few samples written
  Serial.print(">>> First 5 READ BACK: ");
  for (int i = 0; i < 5 && i < sample_count; i++) {
    uint32_t addr = cache->cache_address + (i << 1);
    uint16_t val = read_ram_sample(addr);
    Serial.print("[@0x");
    Serial.print(addr, HEX);
    Serial.print("]=");
    Serial.print(val);
    Serial.print(" ");
  }
  Serial.println();
  
  // Also check values at beat 3 (step 8) where audio cuts off
  uint32_t beat3_tick = samples_per_step * 8;  // Start of step 8
  if (beat3_tick < sample_count) {
    Serial.print(">>> Beat 3 (step 8) first 5 samples at tick ");
    Serial.print(beat3_tick);
    Serial.print(": ");
    for (int i = 0; i < 5; i++) {
      uint32_t tick = beat3_tick + i;
      if (tick < sample_count) {
        uint32_t addr = cache->cache_address + (tick << 1);
        uint16_t val = read_ram_sample(addr);
        Serial.print("[@0x");
        Serial.print(addr, HEX);
        Serial.print("]=");
        Serial.print(val);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  
  Serial.println("========================================");
}

// ============================================================================

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

// Calculation-only function: reads pots and updates slice/pitch globals
// Returns true if values changed significantly, false otherwise
bool updateSliceAndPitchFromPots(bool forceUpdate) {
  uint16_t start = analogRead(A0);
  uint16_t end = analogRead(A1);

  if (abs(lastPot1 - start) < 80 && abs(lastPot2 - end) < 80 && !forceUpdate) {
    return false;
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
    sliceStart = samples[selectedSample].start + mappedStart;
    if (sliceStart % 2 != 0) sliceStart++;
    sliceEnd = samples[selectedSample].start + mappedEnd;
    if (sliceEnd % 2 != 0) sliceEnd++;
  } else {
    playback_pitch = map(end, 0, 4095, 32, 127);
  }

  return true;
}

// Display-only function: draws the edit screen based on current globals
void drawEditScreen() {
  // Recalculate mapped values from current sliceStart/sliceEnd
  uint32_t mappedStart, mappedEnd;
  
  if (editInnerState == SLICE) {
    mappedStart = sliceStart - samples[selectedSample].start;
    mappedEnd = sliceEnd - samples[selectedSample].start;
  } else {
    // For pitch screen, recalculate from lastPot1 and lastPot2
    mappedStart = map(lastPot1, 0, 4095, 0, totalSamplesInView);
    mappedEnd = map(lastPot2, 0, 4095, 0, totalSamplesInView);
  }

  if (mappedStart > mappedEnd) {
    uint32_t swap = mappedStart;
    mappedStart = mappedEnd;
    mappedEnd = swap;
  }

  lcd.clear();
  lcd.setCursor(0, 0);

  if (editInnerState == SLICE) {
    lcd.print("SLICE");
    if (sampleLengthLocked) {
      if (loopMode) {
        lcd.print("      L");
      } else {
        lcd.print("       ");
      }

      if (sampleLengthDivision == 0) {
        lcd.print(" 1/1");
      } else if (sampleLengthDivision == 1) {
        lcd.print(" 1/2");
      } else {
        lcd.print(" 1/4");
      }
    } else {
      if (loopMode) {
        lcd.print("      L    ");
      } else {
        lcd.print("           ");
      }
    }
    drawSliceBar(mappedStart, mappedEnd);
  } else {
    lcd.print("PITCH");
    lcd.print("           ");
    drawPitchBar();
  }
}

// Combined function for backwards compatibility: reads pots AND updates display
void updateEditScreen(bool forceUpdate) {
  if (updateSliceAndPitchFromPots(forceUpdate)) {
    drawEditScreen();
  }
}

void drawPlaybackScreen() {
  lcd.setCursor(0, 0);
  
  // Top row - same as normal edit screen
  if (editInnerState == SLICE) {
    lcd.print("SLICE");
    if (sampleLengthLocked) {
      if (loopMode) {
        lcd.print("      L");
      } else {
        lcd.print("       ");
      }

      if (sampleLengthDivision == 0) {
        lcd.print(" 1/1");
      } else if (sampleLengthDivision == 1) {
        lcd.print(" 1/2");
      } else {
        lcd.print(" 1/4");
      }
    } else {
      if (loopMode) {
        lcd.print("      L    ");
      } else {
        lcd.print("           ");
      }
    }
  } else {
    lcd.print("PITCH");
    lcd.print("           ");
  }
  
  // Bottom row - playhead visualization
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; i++) {
    if (i == playheadCharPos) {
      lcd.write(255);  // Solid block at playhead
    } else {
      lcd.write('-');  // Dash for rest
    }
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
uint16_t potStart = 0;
uint16_t potEnd = 0;

void loop() {
  now = millis();
  
  // Check for pending sequence render requests and process synchronously
  // (Moved from Core 1 to Core 0 for initial testing)
  if (render_request && render_sequence_id >= 0) {
    int seq_id = render_sequence_id;
    
    Serial.print(">>> Rendering sequence ");
    Serial.print(seq_id);
    Serial.println(" synchronously...");
    
    // Do the actual rendering (synchronously on Core 0)
    renderSequenceToCache(seq_id);
    
    // Update cache status
    sequence_cache[seq_id].mode = CACHE_MODE;
    sequence_cache[seq_id].needs_render = false;
    
    Serial.print(">>> Finished rendering sequence ");
    Serial.println(seq_id);
    
    // Clear the request
    render_request = false;
    render_sequence_id = -1;
  }

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

        if (downButton.pressed()) {
          loopMode = !loopMode;
          updateEditScreen(true);
        }

        if (upButton.pressed()) {
          playback_pitch = 64; // reset pitch to default
          updateEditScreen(true);
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
          }
        }

        if (modeButton.released()) {
          if (!modeButtonUsedAsModifier) {
            if (editInnerState == SLICE) {
              currentState = INDEX_STATE;
              drawSampleList();
            } else {
              editInnerState = SLICE;
            }
          }

          modeButtonUsedAsModifier = false;
        }

        if (needsUpdate) {
          Serial.println("forcing update after playback");
          updateEditScreen(true);
          needsUpdate = false;
        }
      }
      break;

    case PLAY_STATE:
      {
        // Update playback visualization
        if (playheadCharPos != lastPlayheadCharPos) {
          drawPlaybackScreen();
          lastPlayheadCharPos = playheadCharPos;
        }

        if (modeButton.pressed()) {
          playback_accumulator = 0;
          playback_address = sliceStart;
          lastPlayheadCharPos = 255;  // Force redraw on restart
        }

        if (selectButton.pressed()) {
          // Stop playback and return to previous state
          playback_accumulator = 0;
          playback_address = sliceStart;

          currentState = nextState;
          if (currentState == EDIT_STATE) {
            updateEditScreen(true);
          } else if (currentState == SELECT_SAMPLE_STATE) {
            drawSelectSampleList();
          }
        }

        // Allow pot adjustments during playback without updating display
        updateSliceAndPitchFromPots(false);

      }
      break;

    case SAMPLER_STATE:
      {
        if (selectButton.pressed()) {
          edit_cursor_pos = cursorPos;  // Save current edit position
          
          // Check if the current sequence needs caching
          bool needs_cache = currentSequenceNeedsCaching();
          
          if (needs_cache) {
            // Only render if sequencer_map has been modified since last render
            if (sequencer_map_modified) {
              Serial.println("*** Preview: Rendering to cache (modified + needs caching)");
              // Render current sequence to temp cache for smooth playback
              renderCurrentSequenceToCache();
            } else {
              Serial.println("*** Preview: Using existing cache (not modified)");
            }
            
            // Start playback using the temp cache
            currentState = SAMPLER_PLAYBACK_STATE;
            currently_playing_sequence = TEMP_CACHE_SLOT;
            cached_playback_tick = sequence_cache[TEMP_CACHE_SLOT].sample_count;  // Start "before" position 0 for pre-advance pattern
          } else {
            Serial.println("*** Preview: Using realtime playback (no caching needed)");
            // Simple sequences can play in realtime - no cache needed!
            currentState = SAMPLER_PLAYBACK_STATE;
            currently_playing_sequence = -1;  // -1 means realtime playback
            sequencer_tick_counter = samples_per_step;
            current_step = 0;
          }
          
          updateSequencerScreen();  // Restore screen after potential "Rendering..."
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
          
          // Mark that sequencer_map has been modified
          sequencer_map_modified = true;
          
          // Intelligently decide if this sequence needs caching
          int current_sequence_id = sequenceCount > 0 ? sequenceCount - 1 : -1;
          if (current_sequence_id >= 0 && current_sequence_id < MAX_CACHED_SEQUENCES) {
            if (needsCaching(current_sequence_id)) {
              Serial.print("*** Sequence ");
              Serial.print(current_sequence_id);
              Serial.println(" NEEDS caching (detected voice overlap)");
              requestSequenceRender(current_sequence_id);
            } else {
              Serial.print("*** Sequence ");
              Serial.print(current_sequence_id);
              Serial.println(" does NOT need caching (can play in realtime)");
              // Keep it in REALTIME_MODE - no render needed
              sequence_cache[current_sequence_id].mode = REALTIME_MODE;
              sequence_cache[current_sequence_id].needs_render = false;
            }
          }
          
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
        
        // Manual cache render trigger: MODE + UP button
        if (modeButton.held() && upButton.pressed()) {
          if (sequenceCount > 0) {
            int seq_to_render = sequenceCount - 1;
            if (seq_to_render < MAX_CACHED_SEQUENCES) {
              requestSequenceRender(seq_to_render);
              
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Rendering seq ");
              lcd.print(seq_to_render);
              lcd.setCursor(0, 1);
              lcd.print(getCacheStatus(seq_to_render));
              
              delay(500);
              updateSequencerScreen();
            }
          }
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
          // Initialize cached playback tracking - will update when each sequence starts
          currently_playing_sequence = sequence_chain[0];
          if (currently_playing_sequence >= 0) {
            cached_playback_tick = sequence_cache[currently_playing_sequence].sample_count;  // Start "before" position 0
          } else {
            cached_playback_tick = 0;  // No sequence, doesn't matter
          }
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
      
      potStart = analogRead(A0);
      potEnd = analogRead(A1);
  }

  updateCurrentStateIfNeeded();
}
