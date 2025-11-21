#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pyttsx3
import threading
import os
from openai import OpenAI

class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__('voice_assistant_node')
        
        self.declare_parameter('openai_api_key', '')
        self.api_key = self.get_parameter('openai_api_key').value
        
        self.client = None
        if self.api_key:
            self.client = OpenAI(api_key=self.api_key)
        else:
            self.get_logger().warn("No OpenAI API key provided. LLM features disabled.")

        # Audio setup
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # TTS setup
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        
        # Publishers
        self.speech_pub = self.create_publisher(String, 'voice/speech_text', 10)
        self.response_pub = self.create_publisher(String, 'voice/llm_response', 10)
        
        # Start listening thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()
        
        self.get_logger().info("Voice Assistant Node started")
        self.speak("Optimus Voice System Online")

    def speak(self, text):
        self.get_logger().info(f"Speaking: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

    def process_llm(self, text):
        if not self.client:
            return "I heard you, but I don't have a brain connected yet."
            
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are Optimus, a helpful robot assistant connected to a 3D printer. Keep answers short."},
                    {"role": "user", "content": text}
                ]
            )
            return response.choices[0].message.content
        except Exception as e:
            self.get_logger().error(f"LLM Error: {e}")
            return "Sorry, I had trouble thinking."

    def listen_loop(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            
            while self.listening and rclpy.ok():
                try:
                    self.get_logger().info("Listening...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                    
                    try:
                        text = self.recognizer.recognize_google(audio)
                        self.get_logger().info(f"Heard: {text}")
                        
                        msg = String()
                        msg.data = text
                        self.speech_pub.publish(msg)
                        
                        # Process with LLM
                        if "optimus" in text.lower():
                            response = self.process_llm(text)
                            
                            resp_msg = String()
                            resp_msg.data = response
                            self.response_pub.publish(resp_msg)
                            
                            self.speak(response)
                            
                    except sr.UnknownValueError:
                        pass
                    except sr.RequestError as e:
                        self.get_logger().error(f"Speech service error: {e}")
                        
                except Exception as e:
                    # Timeout or other error, just continue
                    pass

    def destroy_node(self):
        self.listening = False
        if self.listen_thread.is_alive():
            self.listen_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistantNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
