#!/usr/bin/env python3
"""
Optimus Voice Interaction Node
- Records voice from USB microphone
- Converts speech to text using Vosk (offline) or Google Speech (online)
- Sends queries to Ollama LLM on Windows PC
- Plays responses through USB speaker
- Provides hooks for vision integration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import threading
import subprocess
import tempfile
import os
import json
import time
import wave
import requests
from typing import Optional, Callable
from enum import Enum

# Audio recording config
AUDIO_DEVICE = "plughw:1,0"  # JOUNIVO Microphone
PLAYBACK_DEVICE = "hw:0,0"  # USB Speaker (direct hardware access for better volume)
SAMPLE_RATE = 44100
CHANNELS = 1
RECORD_SECONDS = 5


class ConversationState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    SPEAKING = "speaking"


class VoiceInteractionNode(Node):
    def __init__(self):
        super().__init__('voice_interaction_node')
        
        # --- Parameters ---
        self.declare_parameter('ollama_host', '192.168.1.184')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('ollama_model', 'llama3.2')
        self.declare_parameter('wake_word', 'optimus')
        self.declare_parameter('use_online_stt', True)  # Use Google STT (needs internet)
        self.declare_parameter('vision_timeout', 10.0)
        self.declare_parameter('tts_voice', 'en-us')
        self.declare_parameter('tts_speed', 150)
        
        self.ollama_host = self.get_parameter('ollama_host').value
        self.ollama_port = self.get_parameter('ollama_port').value
        self.ollama_model = self.get_parameter('ollama_model').value
        self.wake_word = self.get_parameter('wake_word').value.lower()
        self.use_online_stt = self.get_parameter('use_online_stt').value
        self.vision_timeout = self.get_parameter('vision_timeout').value
        
        # --- State ---
        self.state = ConversationState.IDLE
        self.conversation_history = []
        self.last_vision_description = None
        self.vision_request_pending = False
        self.vision_request_event = threading.Event()
        
        # --- System Prompt ---
        self.system_prompt = """You are a helpful assistant. Give short, direct answers. Do not roleplay or add sound effects."""

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, 'voice/state', 10)
        self.speech_text_pub = self.create_publisher(String, 'voice/speech_text', 10)
        self.response_pub = self.create_publisher(String, 'voice/llm_response', 10)
        self.vision_request_pub = self.create_publisher(String, 'vision/describe_request', 10)
        
        # --- Subscribers ---
        self.create_subscription(
            String, 
            'vision/description', 
            self.vision_description_callback, 
            10
        )
        self.create_subscription(
            String,
            'voice/trigger',  # External trigger to start listening
            self.external_trigger_callback,
            10
        )
        
        # --- STT Setup ---
        self.vosk_model = None
        if not self.use_online_stt:
            self._init_vosk()
        
        # --- Main listening loop ---
        self.running = True
        self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listen_thread.start()
        
        self.get_logger().info(f"Voice Interaction Node started")
        self.get_logger().info(f"  Ollama: {self.ollama_host}:{self.ollama_port} model={self.ollama_model}")
        self.get_logger().info(f"  Wake word: '{self.wake_word}'")
        
        # Announce startup
        self._speak("Optimus voice system online")
        self._publish_state()

    def _init_vosk(self):
        """Initialize Vosk for offline speech recognition"""
        try:
            from vosk import Model, KaldiRecognizer
            model_path = os.path.expanduser("~/vosk-model-small-en-us-0.15")
            if os.path.exists(model_path):
                self.vosk_model = Model(model_path)
                self.get_logger().info("Vosk model loaded for offline STT")
            else:
                self.get_logger().warn(f"Vosk model not found at {model_path}")
                self.get_logger().warn("Download from: https://alphacephei.com/vosk/models")
                self.use_online_stt = True
        except ImportError:
            self.get_logger().warn("Vosk not installed, using online STT")
            self.use_online_stt = True

    def _publish_state(self):
        """Publish current state"""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def _record_audio(self, duration: float = RECORD_SECONDS) -> Optional[str]:
        """Record audio from microphone using ALSA, return path to WAV file"""
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                wav_path = f.name
            
            cmd = [
                'arecord',
                '-D', AUDIO_DEVICE,
                '-f', 'S16_LE',
                '-r', str(SAMPLE_RATE),
                '-c', str(CHANNELS),
                '-d', str(int(duration)),
                '-q',  # Quiet mode
                wav_path
            ]
            
            self.get_logger().info(f"Recording for {duration}s...")
            result = subprocess.run(cmd, capture_output=True, timeout=duration + 10)
            
            if result.returncode != 0:
                self.get_logger().warn(f"arecord failed: {result.stderr.decode()}")
                return None
            
            if os.path.exists(wav_path) and os.path.getsize(wav_path) > 1000:
                self.get_logger().info(f"Recorded {os.path.getsize(wav_path)} bytes")
                return wav_path
            else:
                self.get_logger().warn(f"Recording too short or missing: {wav_path}")
                return None
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Recording timed out after {duration + 10}s")
            return None
        except Exception as e:
            self.get_logger().error(f"Record error: {e}")
            return None

    def _speech_to_text_google(self, wav_path: str) -> Optional[str]:
        """Convert speech to text using Google Speech Recognition"""
        try:
            import speech_recognition as sr
            recognizer = sr.Recognizer()
            
            with sr.AudioFile(wav_path) as source:
                audio = recognizer.record(source)
            
            text = recognizer.recognize_google(audio)
            return text.strip()
            
        except Exception as e:
            self.get_logger().debug(f"Google STT: {e}")
            return None

    def _speech_to_text_vosk(self, wav_path: str) -> Optional[str]:
        """Convert speech to text using Vosk (offline)"""
        if not self.vosk_model:
            return None
            
        try:
            from vosk import KaldiRecognizer
            
            wf = wave.open(wav_path, "rb")
            rec = KaldiRecognizer(self.vosk_model, wf.getframerate())
            rec.SetWords(True)
            
            results = []
            while True:
                data = wf.readframes(4000)
                if len(data) == 0:
                    break
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    if result.get('text'):
                        results.append(result['text'])
            
            final = json.loads(rec.FinalResult())
            if final.get('text'):
                results.append(final['text'])
            
            wf.close()
            return ' '.join(results).strip() if results else None
            
        except Exception as e:
            self.get_logger().error(f"Vosk STT error: {e}")
            return None

    def _speech_to_text(self, wav_path: str) -> Optional[str]:
        """Convert speech to text using configured method"""
        if self.use_online_stt:
            return self._speech_to_text_google(wav_path)
        else:
            return self._speech_to_text_vosk(wav_path)

    def _query_ollama(self, user_message: str, include_vision: bool = False) -> Optional[str]:
        """Send query to Ollama LLM"""
        try:
            url = f"http://{self.ollama_host}:{self.ollama_port}/api/chat"
            
            # Build messages
            messages = [{"role": "system", "content": self.system_prompt}]
            
            # Add vision context if available
            if include_vision and self.last_vision_description:
                vision_context = f"[Current camera view: {self.last_vision_description}]"
                messages.append({
                    "role": "system", 
                    "content": vision_context
                })
            
            # Add conversation history (last 6 exchanges)
            messages.extend(self.conversation_history[-12:])
            
            # Add current message
            messages.append({"role": "user", "content": user_message})
            
            payload = {
                "model": self.ollama_model,
                "messages": messages,
                "stream": False,
                "options": {
                    "temperature": 0.7,
                    "num_predict": 150  # Keep responses short for speech
                }
            }
            
            self.get_logger().info(f"Querying Ollama: {user_message[:50]}...")
            
            response = requests.post(url, json=payload, timeout=30)
            response.raise_for_status()
            
            result = response.json()
            assistant_message = result.get('message', {}).get('content', '')
            
            if assistant_message:
                # Update conversation history
                self.conversation_history.append({"role": "user", "content": user_message})
                self.conversation_history.append({"role": "assistant", "content": assistant_message})
                
                # Keep history bounded
                if len(self.conversation_history) > 20:
                    self.conversation_history = self.conversation_history[-20:]
                
                return assistant_message.strip()
            
            return None
            
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f"Cannot connect to Ollama at {self.ollama_host}:{self.ollama_port}")
            return "I'm having trouble connecting to my brain. Is the server running?"
        except Exception as e:
            self.get_logger().error(f"Ollama error: {e}")
            return "Sorry, I had trouble thinking about that."

    def _speak(self, text: str):
        """Convert text to speech and play through speaker using Piper TTS"""
        if not text:
            return
            
        self.state = ConversationState.SPEAKING
        self._publish_state()
        
        self.get_logger().info(f"Speaking: {text[:60]}...")
        
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                wav_path = f.name
            
            # Try Piper TTS first - using Alan deep British male voice
            piper_model = os.path.expanduser("~/.local/share/piper/en_GB-alan-low.onnx")
            
            # Fallback voices
            if not os.path.exists(piper_model):
                piper_model = os.path.expanduser("~/.local/share/piper/en_US-ryan-high.onnx")
            if not os.path.exists(piper_model):
                piper_model = os.path.expanduser("~/.local/share/piper/en_US-lessac-medium.onnx")
            
            if os.path.exists(piper_model):
                # Use Piper for high-quality TTS
                # Escape quotes in text for shell
                safe_text = text.replace('"', '\\"').replace("'", "\\'")
                piper_cmd = f'echo "{safe_text}" | piper --model {piper_model} --output_file {wav_path}'
                result = subprocess.run(piper_cmd, shell=True, capture_output=True, timeout=60)
                
                if result.returncode != 0:
                    self.get_logger().warn(f"Piper failed: {result.stderr.decode()}")
                    raise Exception("Piper failed")
                
                # Convert to stereo 48kHz and normalize volume (hw:0,0 requires stereo)
                try:
                    amp_path = wav_path + ".amp.wav"
                    # norm -1 normalizes to max volume without clipping
                    sox_cmd = ['sox', wav_path, '-r', '48000', '-c', '2', amp_path, 'norm', '-1']
                    sox_result = subprocess.run(sox_cmd, capture_output=True, timeout=10)
                    if sox_result.returncode == 0 and os.path.exists(amp_path):
                        os.unlink(wav_path)
                        wav_path = amp_path
                except Exception as e:
                    self.get_logger().warn(f"Sox conversion failed: {e}")
            else:
                # Fallback to espeak
                espeak_cmd = [
                    'espeak',
                    '-v', 'en-us',
                    '-s', '150',
                    '-w', wav_path,
                    text
                ]
                subprocess.run(espeak_cmd, capture_output=True, timeout=30)
            
            # Play through speaker
            if os.path.exists(wav_path) and os.path.getsize(wav_path) > 0:
                play_cmd = ['aplay', '-D', PLAYBACK_DEVICE, wav_path]
                subprocess.run(play_cmd, capture_output=True, timeout=60)
                os.unlink(wav_path)
                
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")
            # Last resort fallback: espeak directly
            try:
                subprocess.run(['espeak', text], capture_output=True, timeout=30)
            except:
                pass
        
        self.state = ConversationState.IDLE
        self._publish_state()

    def _request_vision_description(self) -> Optional[str]:
        """Request vision description from Jetson and wait for response"""
        self.get_logger().info("Requesting vision description...")
        
        self.vision_request_pending = True
        self.vision_request_event.clear()
        self.last_vision_description = None
        
        # Publish request
        msg = String()
        msg.data = "describe"
        self.vision_request_pub.publish(msg)
        
        # Wait for response with timeout
        got_response = self.vision_request_event.wait(timeout=self.vision_timeout)
        self.vision_request_pending = False
        
        if got_response and self.last_vision_description:
            return self.last_vision_description
        else:
            self.get_logger().warn("Vision description timed out")
            return None

    def vision_description_callback(self, msg: String):
        """Handle vision description from Jetson"""
        self.last_vision_description = msg.data
        self.get_logger().info(f"Received vision: {msg.data[:50]}...")
        
        if self.vision_request_pending:
            self.vision_request_event.set()

    def external_trigger_callback(self, msg: String):
        """Handle external trigger to start listening"""
        if self.state == ConversationState.IDLE:
            self.get_logger().info(f"External trigger: {msg.data}")
            # Could trigger immediate listen cycle

    def _is_vision_query(self, text: str) -> bool:
        """Check if the query is asking about what the robot sees"""
        vision_keywords = [
            'what do you see',
            'what can you see',
            'what are you looking at',
            'describe what you see',
            'look at',
            'tell me what you see',
            'what\'s in front of you',
            'what is in front of you',
            'show me',
            'identify',
            'recognize'
        ]
        text_lower = text.lower()
        return any(kw in text_lower for kw in vision_keywords)

    def _process_query(self, text: str):
        """Process user query and generate response"""
        self.state = ConversationState.PROCESSING
        self._publish_state()
        
        # Publish transcribed text
        msg = String()
        msg.data = text
        self.speech_text_pub.publish(msg)
        
        # Check if this is a vision query
        include_vision = False
        if self._is_vision_query(text):
            self.get_logger().info("Vision query detected")
            vision_desc = self._request_vision_description()
            if vision_desc:
                include_vision = True
            else:
                self._speak("I'm having trouble seeing right now. Let me try to answer anyway.")
        
        # Query LLM
        response = self._query_ollama(text, include_vision=include_vision)
        
        if response:
            # Publish response
            resp_msg = String()
            resp_msg.data = response
            self.response_pub.publish(resp_msg)
            
            # Speak response
            self._speak(response)
        
        self.state = ConversationState.IDLE
        self._publish_state()

    def _listen_loop(self):
        """Main loop: continuously listen for wake word, then process commands"""
        self.get_logger().info("Starting listen loop...")
        
        while self.running and rclpy.ok():
            try:
                if self.state != ConversationState.IDLE:
                    time.sleep(0.1)
                    continue
                
                self.state = ConversationState.LISTENING
                self._publish_state()
                
                # Record audio
                wav_path = self._record_audio(duration=5)
                
                if wav_path:
                    try:
                        # Convert to text
                        text = self._speech_to_text(wav_path)
                        
                        if text:
                            self.get_logger().info(f"Heard: {text}")
                            
                            # Check for wake word
                            if self.wake_word in text.lower():
                                self.get_logger().info("Wake word detected!")
                                
                                # Process the query (remove wake word for cleaner query)
                                query = text.lower().replace(self.wake_word, '').strip()
                                query = query.strip(',').strip()
                                
                                if query:
                                    self._process_query(query if query else text)
                                else:
                                    # Just wake word, ask what they need
                                    self._speak("Yes? How can I help?")
                                    
                                    # Listen for follow-up
                                    time.sleep(0.5)
                                    followup_path = self._record_audio(duration=5)
                                    if followup_path:
                                        followup_text = self._speech_to_text(followup_path)
                                        if followup_text:
                                            self._process_query(followup_text)
                                        os.unlink(followup_path)
                    finally:
                        # Clean up temp file
                        if os.path.exists(wav_path):
                            os.unlink(wav_path)
                
                self.state = ConversationState.IDLE
                self._publish_state()
                
                # Small delay before next listen cycle
                time.sleep(0.5)
                
            except Exception as e:
                self.get_logger().error(f"Listen loop error: {e}")
                self.state = ConversationState.IDLE
                time.sleep(1)

    def clear_history(self):
        """Clear conversation history"""
        self.conversation_history = []
        self.get_logger().info("Conversation history cleared")

    def destroy_node(self):
        self.running = False
        if self.listen_thread.is_alive():
            self.listen_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInteractionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
