#!/usr/bin/python3

# ----------------------------------------------
# Title: relax.py
# Description: Muscle Relaxation Exercise
# Author: Jimmy Wang
# Date created: Jan 4, 2024
# Date modified: Feb 20, 2024
# ----------------------------------------------

import threading
import time
from actuators.node_actuators import NodeActuators #class
from actuators.led_controller import LEDController #class
from actuators.joints_controller import JointsController #class
from actuators.joints_movement import JointsMovement #class
from actuators.cosmetics_controller import CosmeticsController #class
from actuators.cosmetics_movement import CosmeticsMovement #class
from actuators.play_audio import AudioPlayer  # class
from IS_modules.node_detect_aruco import *
from IS_modules.detect_touch import *
from actuators.remote import connect_remote, receive_data, send_data, close_connection
from actuators.status_handler import status_handler
from actuators.speech_to_text import SpeechToText

class RelaxBehavior:
    def __init__(self):
        # Initialize all actuators
        self.led_controller = LEDController()
        self.joints_controller = JointsController()
        self.joints_movement = JointsMovement()
        self.cosmetics_controller = CosmeticsController()
        self.cosmetics_movement = CosmeticsMovement()
        self.aruco_detect = NodeDetectAruco()
        self.audio_player = AudioPlayer()

        self.stop_flag = False
        self.parent_thread = None  # Store parent thread reference
        self.parent_intro_thread= None
        self.parent_head_thread=None
        self.timers = []  # List to track timers

        # Initialize touch detection
        self.touch_detect = see_touch()
        self.remote_data=[0,0,0,0,0]

        # Initialize speech recognition
        self.speech_to_text = SpeechToText()


    def run(self):
        status_handler.update_status("Relax behavior started.")
        print("\n\n")
        print("****************************\n\n")
        print("RELAX BEHAVIOUR STARTED\n\n")
        print("****************************\n\n")
        # Start the check_exit_flag thread
        
        self.parent_intro_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
              
        status_handler.update_status("Playing relaxation choice audio.")
        relax_prompt = 'mp3_files_slushy/relax/relax_choice.mp3'
        play_relax_thread = threading.Thread(target=self.audio_player.play_audio, args=(relax_prompt,))
        play_relax_thread.start()

        # Wait for the first set of actions to finish
        play_relax_thread.join()
        time.sleep(2.0)

        status_handler.update_status("Waiting for user input or timeout.")
        start_time = time.time()

        # Start speech to text
        self.speech_to_text.stop = False
        speech_to_text_thread = threading.Thread(target=self.speech_to_text.loop)
        speech_to_text_thread.daemon = True
        speech_to_text_thread.start()

        ex_activated = False
        while  ((time.time() - start_time) < 15) and (self.stop_flag == False):
            exercise_keywords = {
            "full": ["all", "full"],
            "arms": ["arms", "arm"],
            "back": ["back"],
            "tummy": ["tummy"],
            "legs": ["legs", "leg"],
            "kluge": ["or", "your"]
            }

            # Check for matching keywords in the last_text
            matched_exercises = [key for key, words in exercise_keywords.items() if any(word in self.speech_to_text.last_text.lower() for word in words)]

            if len(matched_exercises) > 1:
                status_handler.update_status("Multiple relaxation commands detected. Ignoring input.")
            elif "full" in matched_exercises:
                ex_activated = True
                status_handler.update_status("Full relaxation command received.")
                self.audio_player.stop()
                self.full_relaxation()
            elif "arms" in matched_exercises:
                ex_activated = True
                status_handler.update_status("Arms relaxation command received.")
                self.audio_player.stop()
                self.relax_arms()
            elif "back" in matched_exercises:
                ex_activated = True
                status_handler.update_status("Back relaxation command received.")
                self.audio_player.stop()
                self.relax_back()
            elif "tummy" in matched_exercises:
                ex_activated = True
                status_handler.update_status("Tummy relaxation command received.")
                self.audio_player.stop()
                self.relax_tummy()
            elif "legs" in matched_exercises:
                ex_activated = True
                status_handler.update_status("Legs relaxation command received.")
                self.audio_player.stop()
                self.relax_legs()

            time.sleep(0.1)

        if not ex_activated:
            status_handler.update_status("Timeout reached or user input received. Proceeding.")
            # Make sure to stop the speech_to_text thread
            self.speech_to_text.stop = True
            self.stop_flag = True
            time_out_thread= threading.Thread(target=self.audio_player.play_audio, args=('mp3_files_slushy/breath_ex/BrEx_Timeout_1.mp3',))
            time_out_thread.start()
            time_out_thread.join()

            # Send a signal to the remote to indicate that timeout has occurred
            try:
                send_data(b'\x01\x00\x00\x00\x02')  
            except Exception as e:
                pass

        # make sure to stop the speech_to_text thread
        self.speech_to_text.stop = True
        time.sleep(0.1)

        
        

    def wait_for_head_touch(self, audio_path):
        status_handler.update_status("Waiting for head touch to start relaxation.")
        self.parent_head_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("Playing relaxation prompt audio...")
        status_handler.update_status("Playing relaxation prompt audio.")

        audio_file = audio_path
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()
        # Wait for touch or timeout
        start_time = time.time()
        while ((time.time() - start_time) < 20) and (self.stop_flag == False):
            
            self.touch_detect.check_touch()
            if self.touch_detect.head_touched:
                status_handler.update_status("Head touch detected. Starting relaxation.")
                print("Head touched! Starting relaxation.")
                return True
            time.sleep(0.1)
        status_handler.update_status("Timeout reached. No head touch detected.")
        self.stop_flag = True
        print("Timeout. No head touch detected. Exiting relaxation.")
        time_out_thread= threading.Thread(target=self.audio_player.play_audio, args=('mp3_files_slushy/breath_ex/BrEx_Timeout_1.mp3',))
        time_out_thread.start()
        time_out_thread.join()
        
        return False

    def relax_back(self):
        status_handler.update_status("Starting back relaxation.")
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting back relaxation")
        if not self.wait_for_head_touch('mp3_files_slushy/relax/relax_back_intro.mp3'):
            status_handler.update_status("Back relaxation skipped due to no head touch.")
            return  # Exit if no head touch detected within 10 seconds

        
        status_handler.update_status("Playing back relaxation audio and scheduling movements.")
        audio_file = 'mp3_files_slushy/relax/relax_back.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(8, self.cosmetics_movement.wagging_tail, (2, 3))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(15, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(16, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (0, 0, 255), (0, 0, 255), (0, 0, 255), 250))
        self.add_timer(21, self.led_controller.toggle_led_sections, (1, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[RELAXATION] Stopping relax...")
                self.speech_to_text.stop = True
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)


    def relax_arms(self):
        status_handler.update_status("Starting arms relaxation.")
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting arms relaxation")
        if not self.wait_for_head_touch('mp3_files_slushy/relax/relax_arms_intro.mp3'):
            status_handler.update_status("Arms relaxation skipped due to no head touch.")
            return  # Exit if no head touch detected within 10 seconds

        status_handler.update_status("Playing arms relaxation audio and scheduling movements.")
        audio_file = 'mp3_files_slushy/relax/relax_arms.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(8, self.cosmetics_movement.wagging_tail, (2, 3))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(15, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(16, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (0, 0, 255), (0, 0, 255), (0, 0, 255), 250))
        self.add_timer(21, self.led_controller.toggle_led_sections, (1, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                self.speech_to_text.stop = True
                print("[RELAXATION] Stopping relax...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)



    def relax_tummy(self):
        status_handler.update_status("Starting tummy relaxation.")
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting tummy relaxation")
        if not self.wait_for_head_touch('mp3_files_slushy/relax/relax_tummy_intro.mp3'):
            status_handler.update_status("Tummy relaxation skipped due to no head touch.")
            return  # Exit if no head touch detected within 10 seconds

        status_handler.update_status("Playing tummy relaxation audio and scheduling movements.")
        audio_file = 'mp3_files_slushy/relax/relax_tummy.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(8, self.cosmetics_movement.wagging_tail, (2, 3))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(15, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(16, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (0, 0, 255), (0, 0, 255), (0, 0, 255), 250))
        self.add_timer(21, self.led_controller.toggle_led_sections, (1, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                self.speech_to_text.stop = True
                print("[RELAXATION] Stopping relax...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

    

    def relax_legs(self):
        status_handler.update_status("Starting legs relaxation.")
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        print("[RELAXATION] Starting legs relaxation")
        if not self.wait_for_head_touch('mp3_files_slushy/relax/relax_legs_intro.mp3'):
            status_handler.update_status("Legs relaxation skipped due to no head touch.")
            return  # Exit if no head touch detected within 10 seconds

        status_handler.update_status("Playing legs relaxation audio and scheduling movements.")
        audio_file = 'mp3_files_slushy/relax/relax_legs.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(5, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(8, self.cosmetics_movement.wagging_tail, (2, 3))
        self.add_timer(10, self.joints_movement.nod, (2, 2))
        self.add_timer(15, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(16, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(17, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (0, 0, 0), (0, 0, 255), 250))
        self.add_timer(18, self.led_controller.toggle_led_sections, (1, (0, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(19, self.led_controller.toggle_led_sections, (1, (255, 0, 0), (255, 0, 0), (0, 0, 255), 250))
        self.add_timer(20, self.led_controller.toggle_led_sections, (1, (0, 0, 255), (0, 0, 255), (0, 0, 255), 250))
        self.add_timer(21, self.led_controller.toggle_led_sections, (1, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
        self.speech_to_text.stop = True
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                self.speech_to_text.stop = True
                print("[RELAXATION] Stopping relax...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
 

    def full_relaxation(self):
        status_handler.update_status("Starting full relaxation sequence.")
        print("[RELAXATION] Starting full relaxation")
        if self.stop_flag==False:
            self.relax_arms()
        if self.stop_flag==False:
            self.relax_back()
        if self.stop_flag==False:
            self.relax_tummy()
        if self.stop_flag ==False:
            self.relax_legs()
        status_handler.update_status("Full relaxation sequence completed.")
        self.relax_complete()

    def relax_complete(self):
        status_handler.update_status("Relaxation complete. Playing end audio.")
        self.speech_to_text.stop = True
        audio_file = 'mp3_files_slushy/relax/relax_end.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audio_file,))
        play_thread.start()


        # Schedule all movements
        self.add_timer(1, self.led_controller.fade_in_led, (2, (0, 255, 0), 150))
        self.add_timer(5, self.led_controller.fade_out_led, (2,))
        self.add_timer(4, self.cosmetics_movement.eye_wink, (1, 1, "left"))
        self.add_timer(4, self.cosmetics_movement.eye_wink, (1, 1, "right"))
        self.add_timer(6, self.cosmetics_movement.wagging_tail, (2, 3))
        self.stop_flag = True

        # Wait for the first set of actions to finish
        play_thread.join()


    ###Exit###
    def check_exit_flag(self):
        status_handler.update_status("Monitoring for exit or relaxation commands.")
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            try:
                self.remote_data = receive_data()
            except Exception as e:
                pass
            if self.aruco_detect.exit_behaviour or self.remote_data[4]==2:
                status_handler.update_status("Exit behavior detected. Stopping relaxation.")
                self.stop_flag = True
                self.speech_to_text.stop = True
                self.audio_player.stop()
                exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files_slushy/i_will_stop.mp3',))
                
                exit_behaviour_thread.start()
                exit_behaviour_thread.join()
                print("[RELAXATION] Exit behaviour detected, stopping relaxation exercise.")
            elif self.remote_data[4]==8:
                status_handler.update_status("Full relaxation command received.")
                self.stop_flag = True
                self.audio_player.stop()
                self.stop_flag = False
                self.full_relaxation()
            elif self.remote_data[4]==3:
                self.stop_flag = True
                self.audio_player.stop()
                self.stop_flag = False
                self.relax_arms()
            elif self.remote_data[4]==5:
                self.stop_flag = True
                self.audio_player.stop()
                self.stop_flag = False
                self.relax_back()
            elif self.remote_data[4]==6:
                self.stop_flag = True
                self.audio_player.stop()
                self.stop_flag = False
                self.relax_tummy()
            elif self.remote_data[4]==7:
                self.stop_flag = True
                self.audio_player.stop()
                self.stop_flag = False
                self.relax_legs()
                

            time.sleep(0.1)

    def safe_execute(self, func, args):
        """Execute function only if stop flag is not set and parent thread is still alive."""
        if not self.stop_flag and (self.parent_thread and self.parent_thread.is_alive()) or (self.parent_head_thread and self.parent_head_thread.is_alive()):
            func(*args)
        else:
            print(f"[RELAXATION] Skipping {func.__name__} because parent thread has exited.")

    # Function to safely start actions
    def add_timer(self, delay, func, args=()):
        timer = threading.Timer(delay, lambda: self.safe_execute(func, args))
        self.timers.append(timer)
        timer.start()
