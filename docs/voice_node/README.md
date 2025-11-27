# Voice Node Documentation

## Overview

The `voice_node` enables natural voice interaction with the Optimus robot. It handles continuous wake-word listening, speech-to-text conversion, LLM-based query processing with Ollama, text-to-speech output, and optional vision system integration.

## Node Name

`voice_interaction_node`

## Purpose

- Continuously listen for wake word ("optimus")
- Convert speech to text using Google Speech Recognition (online) or Vosk (offline)
- Process queries through Ollama LLM running on remote server
- Generate natural speech responses using Piper TTS
- Integrate with vision system for "what do you see" queries
- Maintain conversation context and history

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ollama_host` | string | `192.168.1.184` | IP address of Ollama LLM server |
| `ollama_port` | int | `11434` | Ollama API port |
| `ollama_model` | string | `llama3.2` | LLM model to use |
| `wake_word` | string | `optimus` | Wake word to activate voice interaction |
| `use_online_stt` | bool | `True` | Use Google STT (online) vs Vosk (offline) |
| `vision_timeout` | float | `10.0` | Timeout (seconds) for vision description requests |
| `tts_voice` | string | `en-us` | TTS voice (for espeak fallback) |
| `tts_speed` | int | `150` | TTS speed (for espeak fallback) |

### Audio Device Configuration

Defined as module constants in `voice_node.py`:

```python
AUDIO_DEVICE = "plughw:1,0"      # JOUNIVO USB Microphone
PLAYBACK_DEVICE = "hw:0,0"       # USB Speaker (stereo, 48kHz)
SAMPLE_RATE = 44100              # Recording sample rate
CHANNELS = 1                     # Mono recording
RECORD_SECONDS = 5               # Recording duration per cycle
```

## ROS 2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/voice/state` | `std_msgs/String` | Current conversation state: `idle`, `listening`, `processing`, `speaking` |
| `/voice/speech_text` | `std_msgs/String` | Transcribed speech text (user input) |
| `/voice/llm_response` | `std_msgs/String` | LLM-generated response text |
| `/vision/describe_request` | `std_msgs/String` | Request to vision system (payload: `"describe"`) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vision/description` | `std_msgs/String` | Vision description response (from Jetson vision node) |
| `/voice/trigger` | `std_msgs/String` | External trigger to start listening (optional) |

## Architecture

### State Machine

The node operates in four states:

```
┌──────────┐
│   IDLE   │ <─────────────┐
└────┬─────┘                │
     │                      │
     │ Wake word detected   │
     ▼                      │
┌────────────┐              │
│ LISTENING  │ ─────────────┤
└────┬───────┘              │
     │                      │
     │ Speech recognized    │
     ▼                      │
┌────────────┐    Response  │
│ PROCESSING │ ─────────────┤
└────┬───────┘   generated  │
     │                      │
     │                      │
     ▼                      │
┌────────────┐              │
│  SPEAKING  │ ─────────────┘
└────────────┘   Complete
```

### Listen Loop

Continuous background thread (`_listen_loop`):

```python
while running:
    1. Record 5 seconds of audio from microphone
    2. Convert speech to text (STT)
    3. Check for wake word in transcription
    4. If wake word detected:
        a. Extract query (remove wake word)
        b. If query empty, ask "How can I help?" and listen again
        c. Otherwise, process_query()
    5. Sleep 0.5 seconds, repeat
```

### Query Processing Flow

```
_process_query(text)
├─> Check if vision query
│    ├─> If yes: request_vision_description()
│    │    └─> Publish to /vision/describe_request
│    │    └─> Wait up to vision_timeout for response
│    └─> Set include_vision flag
│
├─> query_ollama(text, include_vision)
│    ├─> Build message array:
│    │    ├─> System prompt
│    │    ├─> Vision context (if available)
│    │    └─> Conversation history (last 12 messages)
│    ├─> POST to Ollama /api/chat
│    └─> Update conversation history
│
└─> speak(response)
     ├─> Generate audio with Piper TTS
     ├─> Convert to stereo 48kHz with sox
     ├─> Normalize volume
     └─> Play through speaker via aplay
```

## Speech-to-Text (STT)

### Google Speech Recognition (Online - Default)

**Requirements:**
```bash
pip install SpeechRecognition
```

**Advantages:**
- High accuracy
- No model download required
- Supports multiple languages

**Disadvantages:**
- Requires internet connection
- Privacy: audio sent to Google servers

**Implementation:**
```python
import speech_recognition as sr

recognizer = sr.Recognizer()
with sr.AudioFile(wav_path) as source:
    audio = recognizer.record(source)
text = recognizer.recognize_google(audio)
```

### Vosk (Offline - Optional)

**Requirements:**
```bash
pip install vosk
# Download model:
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
```

**Advantages:**
- Fully offline
- Privacy-preserving
- No API rate limits

**Disadvantages:**
- Lower accuracy than Google
- Requires model download (~40MB)
- CPU intensive

**Implementation:**
```python
from vosk import Model, KaldiRecognizer

model = Model("~/vosk-model-small-en-us-0.15")
rec = KaldiRecognizer(model, wf.getframerate())

while True:
    data = wf.readframes(4000)
    if rec.AcceptWaveform(data):
        result = json.loads(rec.Result())
        text = result.get('text')
```

## LLM Integration (Ollama)

### Ollama Server Requirements

Install Ollama on a separate machine (Windows PC, Linux server, etc.):

```bash
# On server machine
ollama pull llama3.2
ollama serve  # Runs on port 11434
```

### API Communication

**Endpoint:**
```
POST http://{ollama_host}:{ollama_port}/api/chat
```

**Request Payload:**
```json
{
  "model": "llama3.2",
  "messages": [
    {"role": "system", "content": "You are a helpful assistant..."},
    {"role": "user", "content": "What is the weather?"}
  ],
  "stream": false,
  "options": {
    "temperature": 0.7,
    "num_predict": 150
  }
}
```

**Response:**
```json
{
  "message": {
    "role": "assistant",
    "content": "I don't have real-time access..."
  },
  "done": true
}
```

### Conversation History Management

- Maintains rolling history of last 20 messages (10 exchanges)
- Includes last 12 messages in each LLM request for context
- Cleared automatically when history exceeds 20 messages

### System Prompt

```python
system_prompt = """You are a helpful assistant. Give short, direct answers. Do not roleplay or add sound effects."""
```

## Text-to-Speech (TTS)

### Piper TTS (Primary)

**Voice:** Alan (British male, deep, natural)

**Requirements:**
```bash
# Install piper
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_amd64.tar.gz
tar -xzf piper_amd64.tar.gz
sudo mv piper /usr/local/bin/

# Download Alan voice
mkdir -p ~/.local/share/piper
cd ~/.local/share/piper
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_GB/alan/low/en_GB-alan-low.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_GB/alan/low/en_GB-alan-low.onnx.json
```

**Audio Pipeline:**
```bash
echo "Text to speak" | piper --model ~/.local/share/piper/en_GB-alan-low.onnx --output_file /tmp/out.wav

# Convert mono 16kHz to stereo 48kHz and normalize
sox /tmp/out.wav -r 48000 -c 2 /tmp/out_stereo.wav norm -1

# Play through USB speaker
aplay -D hw:0,0 /tmp/out_stereo.wav
```

**Why Stereo Conversion?**
- USB speaker device `hw:0,0` requires stereo (2 channels)
- Piper outputs mono by default
- `sox` handles conversion and normalization

**Volume Normalization:**
- `norm -1` normalizes audio to maximum volume without clipping
- Provides clear, loud output
- PCM volume set to 90% in ALSA mixer

### Espeak (Fallback)

If Piper is unavailable, falls back to espeak:

```bash
espeak -v en-us -s 150 -w output.wav "Text to speak"
```

## Vision Integration

### Query Detection

Vision-related queries trigger camera description request:

**Keywords:**
- "what do you see"
- "what can you see"
- "describe what you see"
- "look at"
- "tell me what you see"
- "identify"
- "recognize"

### Vision Request Flow

```python
1. Detect vision keywords in user query
2. Publish to /vision/describe_request: "describe"
3. Wait for response on /vision/description (timeout: 10s)
4. Include description in LLM context:
   [Current camera view: {vision_description}]
5. LLM generates response incorporating vision
```

### Vision Timeout Handling

If vision system doesn't respond within `vision_timeout`:
- Continue with query processing
- Inform user: "I'm having trouble seeing right now. Let me try to answer anyway."

## Audio Configuration

### Recording

**Device:** `plughw:1,0` (JOUNIVO USB Microphone)
- Sample rate: 44100 Hz
- Channels: 1 (mono)
- Format: S16_LE (16-bit PCM)
- Duration: 5 seconds per recording

**Command:**
```bash
arecord -D plughw:1,0 -f S16_LE -r 44100 -c 1 -d 5 output.wav
```

### Playback

**Device:** `hw:0,0` (USB Speaker)
- Sample rate: 48000 Hz
- Channels: 2 (stereo)
- Format: S16_LE

**ALSA Mixer Settings:**
```bash
amixer -c 0 sset 'PCM' 90%  # Set volume to 90%
alsactl store               # Save settings
```

## Dependencies

### Python Packages

```bash
# Required
pip install rclpy requests

# Speech Recognition (choose one)
pip install SpeechRecognition  # For Google STT (online)
pip install vosk               # For offline STT

# Audio (pre-installed on most systems)
sudo apt install alsa-utils sox

# TTS
# Install piper manually (see TTS section)
```

### External Services

- **Ollama LLM Server:** Running on `ollama_host:ollama_port`
- **Vision Node (Optional):** For vision integration queries

## Error Handling

### STT Failures

```python
try:
    text = recognizer.recognize_google(audio)
except Exception as e:
    self.get_logger().debug(f"Google STT: {e}")
    return None
```

- Logged at debug level (not errors)
- Returns None, listen loop continues
- Does not crash node

### LLM Connection Failures

```python
except requests.exceptions.ConnectionError:
    return "I'm having trouble connecting to my brain. Is the server running?"
```

- User-friendly error message spoken aloud
- Node continues running
- Automatic retry on next query

### TTS Failures

1. **Piper fails:** Falls back to espeak
2. **Sox fails:** Uses original mono audio
3. **Aplay fails:** Tries espeak direct output as last resort

### Audio Device Issues

- Recording failures logged as warnings
- Node continues listen loop
- No crash on temporary USB device disconnection

## Troubleshooting

### No Speech Recognition

1. **Check microphone:**
   ```bash
   arecord -D plughw:1,0 -f S16_LE -r 44100 -c 1 -d 5 test.wav
   aplay test.wav
   ```

2. **Verify STT dependencies:**
   ```bash
   python3 -c "import speech_recognition"
   ```

3. **Check internet connection** (for Google STT)

### No Audio Output

1. **Check speaker device:**
   ```bash
   aplay -l  # List playback devices
   speaker-test -D hw:0,0 -c 2
   ```

2. **Verify volume:**
   ```bash
   amixer -c 0 sget 'PCM'
   ```

3. **Test TTS pipeline:**
   ```bash
   echo "test" | piper --model ~/.local/share/piper/en_GB-alan-low.onnx --output_file /tmp/test.wav
   sox /tmp/test.wav -r 48000 -c 2 /tmp/test_stereo.wav norm -1
   aplay -D hw:0,0 /tmp/test_stereo.wav
   ```

### LLM Not Responding

1. **Check Ollama server:**
   ```bash
   curl http://192.168.1.184:11434/api/tags
   ```

2. **Test chat endpoint:**
   ```bash
   curl http://192.168.1.184:11434/api/chat -d '{
     "model": "llama3.2",
     "messages": [{"role": "user", "content": "hi"}],
     "stream": false
   }'
   ```

3. **Verify network connectivity:**
   ```bash
   ping 192.168.1.184
   ```

### Wake Word Not Detecting

1. **Check transcription:**
   ```bash
   ros2 topic echo /voice/speech_text
   ```

2. **Verify wake word parameter:**
   ```bash
   ros2 param get /voice_interaction_node wake_word
   ```

3. **Test with louder/clearer pronunciation**

## Performance Considerations

### CPU Usage

- **Vosk (offline):** High CPU during transcription (~30-50%)
- **Google STT (online):** Low CPU, network dependent
- **Piper TTS:** Moderate CPU during synthesis (~20-30%)

### Latency

| Operation | Typical Latency |
|-----------|-----------------|
| Recording | 5 seconds (fixed) |
| Google STT | 0.5-2 seconds |
| Vosk STT | 1-3 seconds |
| Ollama query | 0.5-5 seconds (model dependent) |
| Piper TTS | 1-3 seconds |
| Total (wake to response) | **8-18 seconds** |

### Memory Usage

- Node base: ~50 MB
- Vosk model loaded: +150 MB
- No significant memory growth over time

## Code Structure

```
voice_node.py
├─ VoiceInteractionNode (class)
│   ├─ __init__()
│   │   ├─ Load parameters
│   │   ├─ Initialize STT (Vosk or Google)
│   │   ├─ Start listen_loop thread
│   │   └─> speak("Optimus voice system online")
│   │
│   ├─ _listen_loop() [Background Thread]
│   │   └─> Continuous:
│   │       ├─> record_audio()
│   │       ├─> speech_to_text()
│   │       ├─> Check for wake_word
│   │       └─> process_query()
│   │
│   ├─ _record_audio() -> wav_path
│   │   └─> arecord command
│   │
│   ├─ _speech_to_text(wav_path) -> text
│   │   ├─> speech_to_text_google() OR
│   │   └─> speech_to_text_vosk()
│   │
│   ├─ _process_query(text)
│   │   ├─> Check if vision query
│   │   ├─> request_vision_description() [if needed]
│   │   ├─> query_ollama()
│   │   └─> speak(response)
│   │
│   ├─> _query_ollama(message, include_vision) -> response
│   │   ├─> Build message array with history
│   │   ├─> POST to Ollama API
│   │   └─> Update conversation_history
│   │
│   └─> _speak(text)
│       ├─> Generate with Piper TTS
│       ├─> Convert/normalize with sox
│       └─> Play with aplay
│
└─ main()
```

## See Also

- [Ollama Documentation](https://github.com/ollama/ollama)
- [Piper TTS](https://github.com/rhasspy/piper)
- [SpeechRecognition Library](https://pypi.org/project/SpeechRecognition/)
- [Vosk Speech Recognition](https://alphacephei.com/vosk/)
