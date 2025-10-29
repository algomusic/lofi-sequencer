// M8 LoFi Sequencer
// Andrew R. Brown September 2025

#include "M8.h"
#include "M8Osc.h"
#include "M8Filt.h"
#include "M8Env.h"

M8Osc osc1, osc2, noiseOsc;
M8Filt filter1;
M8Env env1;
unsigned long msNow = millis();
bool autoPlay = false; // for testing
// audio
unsigned long followTime = msNow;
int followDelta = 5;
float baseCutoff = 0.1;
float followAmnt = 0.4;
int volume = 1000; // 0-1024
// buttons, 1 Green Play, 2 Blue Generate
unsigned long buttonTime = msNow;
int buttonDelta = 21;
bool prevPlayButtonState = 1;
bool prevGenerateButtonState = 1;
bool shift = false;
// dials, 1 Yellow Timbre, 2 Red Range, 3 White Volume, 4 Green Density
unsigned long potTime = msNow;
int potDelta = 131;
int pot1Value = 0;
int pot2Value = 0;
int pot3Value = 0;
int pot4Value = 0;
uint8_t oscType = 0; // 0 = saw, 1 = sqr, 2 = noise
// seq
uint8_t pitchSequence[16];
int currPC[5];
unsigned long pitchTime = msNow;
int stepDelta = 125; // 120 BPM 16th notes
int stepCounter = 0;
int pcPent [] = {0,2,4,7,9};
int pcTriad [] = {0,4,7,0,0};
bool playing = false; //false;
int density = 1024; // 0 - 1024
int transpose = 12;
float range = 1; // 0.0 - 2.0 octaves
int seqLength = 16; // 1 - 16

// sync
#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_event.h>
#include <esp_netif.h>

#define WIFI_CHANNEL 1
uint32_t prevCount = 0;
unsigned long prevTimeStamp = 0;

typedef struct {
  uint32_t counter;
  uint32_t timestamp_ms;
} payload_t;

static esp_err_t initWifiSta(int chan) {
  esp_err_t err;
  if ((err = esp_netif_init()) != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
  if ((err = esp_event_loop_create_default()) != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  if ((err = esp_wifi_init(&cfg)) != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_ps(WIFI_PS_NONE));

  // Clear any saved STA config so it won’t try to join an AP
  wifi_config_t empty = {};
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &empty));

  if ((err = esp_wifi_start()) != ESP_OK && err != ESP_ERR_WIFI_CONN) return err;

  if (chan > 0) {
    if ((err = esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_disconnect());
  return ESP_OK;
}

static void printChannel(const char* tag) {
  uint8_t primary; wifi_second_chan_t second;
  if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
    Serial.printf("%s channel: %u (second=%d)\n", tag, primary, (int)second);
  }
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info) return;
  Serial.printf("From %02X:%02X:%02X:%02X:%02X:%02X len=%d",
  info->src_addr[0], info->src_addr[1], info->src_addr[2],
  info->src_addr[3], info->src_addr[4], info->src_addr[5], len);
  if (len == sizeof(payload_t)) {
    payload_t p;
    memcpy(&p, data, sizeof(p));
    Serial.printf(" | counter=%lu ts=%lu", (unsigned long)p.counter, (unsigned long)p.timestamp_ms);
    int countDelta = (unsigned long)p.counter - (uint32_t)prevCount;
    Serial.printf(" | count delta=%lu", (unsigned long)countDelta);
    prevCount = (unsigned long)p.counter;
    //
    // unsigned long now = millis();
    int timestampDelta = p.timestamp_ms - prevTimeStamp;
    Serial.printf(" Message delta: %lu", timestampDelta);
    // lastReceiveTime = now;

    int stepDelta = timestampDelta / countDelta;
    Serial.printf(" | Step delta: %d", stepDelta);

    prevTimeStamp = p.timestamp_ms;
  } else {
    Serial.print(" | unexpected size");
  }
  Serial.println();
}

void generateSequence() {
  pitchSequence[0] = 60; // start on root in C
  // int stepAve = 0;
  for (int i=1; i<16; i++) {
    int stepSize = random(12) - 6; // over an octave
    // stepAve += stepSize;
    // Serial.println("stepSize = " + String(stepSize));
    pitchSequence[i] = pitchSequence[i-1] + stepSize; //random walk
    if (pitchSequence[i] == pitchSequence[i-1]) {
      // pitchSequence[i] = pitchSequence[i-1] + random(range) - (range/2); // try to minimise repeated notes
      pitchSequence[i] = pitchSequence[i-1] + random(12) - 6; // try to minimise repeated notes
    }
    if (pitchSequence[i-1] < 24) pitchSequence[i-1] = 24 + random(5); // bounds check
    if (pitchSequence[i-1] > 84) pitchSequence[i-1] = 84 - random(5);

  }
  // Serial.println("stepAve = " + String (stepAve/15.0f));
}

int calcStepDelta(float bpm) {
  if (bpm > 0) {
    // return 60.0f / bpm * 1000.0f / 1 / stepDiv;
    return 60.0f / bpm * 1000.0f / 4;
  } else return 125;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  // sync
  esp_err_t err = initWifiSta(WIFI_CHANNEL);
  Serial.printf("initWifiSta -> %s\n", esp_err_to_name(err));
  printChannel("Receiver");
  err = esp_now_init();
  Serial.printf("esp_now_init -> %s\n", esp_err_to_name(err));
  esp_now_register_recv_cb(onRecv);
  Serial.println("ESPNOW Receiver ready");
  // Button setup
  pinMode(5, INPUT_PULLUP); // Play
  pinMode(6, INPUT_PULLUP); // Generate
  // seq setup
  currPC[5];
  for (int i=0; i<5; i++) {
    if (stepCounter%4 == 0) {
      currPC[i] = pcPent[i];
    } else currPC[i] = pcTriad[i];
  }
  generateSequence();
  // BPM LED
  pinMode(10, OUTPUT);
  // Audio setup
  setM8PwmPin(8); // any PWM capable GPIO
  osc1.setWave("sawtooth"); // "sine", "triangle", "square", "sawtooth", "noise"
  osc2.setWave("square");
  noiseOsc.setWave("noise");
  osc1.setPitch(69); // A 440
  osc1.setDetune(0.7); // 0.0 - 1.0
  osc2.setPitch(69); // A 440
  osc2.setDetune(0.7); // 0.0 - 1.0
  filter1.setCutoff(baseCutoff); // 0.0 - 1.0
  env1.setAttack(0.03); // 0.0 - 1.0
  env1.setDecay(0.5); // 0.0 - 1.0
  // Start audio with timer-based callback
  startM8Audio(audioCallback);
  delay(200);
  Serial.println("LoFi Generative Sequencer");
}

void loop() {
  // check if it's time for a new note, if so then generate one
  msNow = millis();

  // sequence step
  if ((unsigned long)(msNow - pitchTime) >= stepDelta) {
    pitchTime += stepDelta;
    if (playing) {
      // flash LED
      if (stepCounter % 4 == 0) {
        digitalWrite(10, HIGH);
      } else digitalWrite(10, LOW);
      // Serial.println(stepCounter);
      int r = random(1024);
      if ((stepCounter % seqLength) == 0 || ((stepCounter % (seqLength/2)) == 0 && density > 127) || 
            ((stepCounter % (seqLength/4)) == 0 && density > 512) || ((stepCounter % (seqLength/8)) == 0 && density > 768) || 
            (stepCounter % 2 == 0 && r < density) || (stepCounter % 2 == 1 && r < density / 2) || density > 1000) {  
        // int p = pitchQuantize(pitchSequence[stepCounter] + transpose, currPC, 0);
        int p = round((pitchSequence[stepCounter] - 60) * range) + 60;
        p = pitchQuantize(p + transpose, currPC, 0);
        if (p > 91) p -= 12;
        if (p > 91) p -= 12; // check again
        if (p < 36) p += 12;
        if (p < 36) p += 12; // check again
        // Serial.println(p);
        osc1.setPitch(p);
        osc2.setPitch(p);
        filter1.setCutoff(baseCutoff);
        if (stepCounter % 2 == 0) {
          env1.setGain(random(30) * 0.01 + 0.7);
        } else env1.setGain(random(30) * 0.01 + 0.5);
        env1.start();
      }
      stepCounter = (stepCounter + 1)%seqLength;
    }
  }

  // env follow
  if ((unsigned long)(msNow - followTime) >= followDelta) {
    followTime += followDelta;
    filter1.setCutoff(env1.getValue() * followAmnt + baseCutoff);
  }

  // button state
  if (!autoPlay && (unsigned long)(msNow - buttonTime) >= buttonDelta) {
    buttonTime += buttonDelta;
    bool playButtonState = digitalRead(5);
    bool generateButtonState = digitalRead(6);
    if (prevPlayButtonState != playButtonState && playButtonState == 0) { // down
      if (!playing) {
        playing = true;
        stepCounter = 0;
      } else {
        playing = false;
        digitalWrite(10, LOW);
      }
      // if (playing && !prevPlayButtonState) playing = false;
      Serial.print("playButtonState = " + String(playButtonState));
      Serial.print(" generateButtonState = " + String(generateButtonState));
      Serial.println(" playing = " + String(playing));
    }
    if (prevGenerateButtonState != generateButtonState && generateButtonState == 0) { // down
      shift = true;
      Serial.println("shift");
    }
    if (prevGenerateButtonState != generateButtonState && generateButtonState == 1) { // up
      shift = false;
      generateSequence();
    }
    prevPlayButtonState = playButtonState;
    prevGenerateButtonState = generateButtonState;
  }

  // potentiometer state
  if (!autoPlay && (unsigned long)(msNow - potTime) >= potDelta) {
    potTime += potDelta;

    int p1 = analogRead(0)>>2; // timbre - envelope
    if (abs(p1 - pot1Value) > 20) {
      pot1Value = p1;
      if (shift) {
        float norm = pot1Value * 0.0009775171065;
        float attack = min(0.9, pow(norm, 5.1)); //0.03;
        float decay = norm; //0.5;
        env1.setAttack(attack); // 0.0 - 1.0
        env1.setDecay(decay); // 0.0 - 1.0
        if (pot1Value > 1000) {
          oscType = (oscType + 1)%3;
        }
        Serial.print("pot1Value = " + String(pot1Value));
        Serial.print(" attack = " + String(attack));
        Serial.print(" decay = " + String(decay));
        Serial.println(" oscType = " + String(oscType));
      } else {
        baseCutoff = pot1Value * 0.0009765625;
        Serial.print("pot1Value = " + String(pot1Value));
        Serial.println(" cutoff = " + String(baseCutoff));
      }
    }

    int p2 = analogRead(1)>>2; // range - octave
    if (abs(p2 - pot2Value) > 20) {
      pot2Value = p2;
      if (shift) {
        transpose = (int)(pot2Value * 0.005859375f - 2) * 12;
        Serial.print("pot2Value = " + String(pot2Value));
        Serial.println(" transpose = " + String(transpose));
      } else {
        range = max(0, pot2Value - 30) * 0.002076843198; // 0.0 - 2.0 and ensure we get zero
        Serial.print("pot2Value = " + String(pot2Value));
        Serial.println(" range = " + String(range));
      }
    }
 
    int p3 = analogRead(2)>>2; // volume & tempo
    if (abs(p3 - pot3Value) > 20) {
      pot3Value = p3;
      float normPotVal = pow(pot3Value * 0.0009775171065, 1.6);
      if (shift) {
        // float normPotVal = pow(pot3Value * 0.0009775171065, 1.6);
        float tempo = 60 + normPotVal * 120;// pot3Value * 0.1171875; // 60 to 180 bpm // TO DO - Check for linear or exp potentiometer
        tempo = (int)(tempo / 10) * 10; // quantise to the closest 10 bpm 
        stepDelta = calcStepDelta(tempo);
        Serial.print("pot3Value = " + String(pot3Value));
        Serial.println(" tempo = " + String(tempo));
      } else {
        volume = max(0, (int)(normPotVal * 1024)); 
        Serial.print("pot3Value = " + String(pot3Value));
        Serial.println(" volume = " + String(volume));
      }
    }

    int p4 = analogRead(3)>>2; // density & length
    if (abs(p4 - pot4Value) > 20) {
      pot4Value = p4;
      if (shift) {
        seqLength = round(pot4Value * 0.0146627566) + 1; // 1 - 16
        Serial.print("pot4Value = " + String(pot4Value));
        Serial.println(" seqLength = " + String(seqLength));
      } else {
        density = pot4Value;
        Serial.print("pot4Value = " + String(pot4Value));
        Serial.println(" density = " + String(density));
      }
    }
  }

}

// Audio callback function - called automatically by the M8 timer
uint8_t audioCallback() {
  // low pass gate
  if (oscType == 0) {
    return  ((env1.next(filter1.next(osc1.nextDual()))) * volume)>>10;
  } else if (oscType == 1) {
    return  ((env1.next(filter1.next(osc2.nextDual()))) * volume)>>10;
  } else if (oscType == 2) {
    return  ((env1.next(filter1.next(noiseOsc.next()))) * volume)>>10;
  }
}

