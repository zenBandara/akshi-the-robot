#!/usr/bin/env python3
import os
import time
import pygame

def main():
    # Path to audio.mp3 in the same directory as this script
    base_dir = os.path.dirname(os.path.abspath(__file__))
    audio_path = os.path.join(base_dir, "audio_intro.mp3")

    if not os.path.exists(audio_path):
        print(f"File not found: {audio_path}")
        return

    # Init mixer
    pygame.mixer.init()
    print("Audio system initialised.")

    # Load and play
    try:
        pygame.mixer.music.load(audio_path)
        pygame.mixer.music.play()
        print("Playing audio.mp3 ...")
    except Exception as e:
        print(f"Error playing audio: {e}")
        return

    # Keep script alive while sound is playing
    try:
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
    finally:
        pygame.mixer.music.stop()
        pygame.mixer.quit()
        print("Playback finished.")

if __name__ == "__main__":
    main()
