#!/usr/bin/python3

# ----------------------------------------------
# Title: audiobooks.py
# Description: Play ch1 of alice in wonderland with body movement
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Feb 5, 2024
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



class AudiobooksBehavior:
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
        self.timers = []  # List to track timers

    def run(self): # Through command prompt
        print("[AUDIOBOOK] Running audiobooks behavior")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        
        # Taking input from the user
        user_input = int(input("Enter a number: \n0= exit \n1= Rumpelstiltskin \n2= The Emperor's New Clothes \n3= Aladdin\n"))
        
        if user_input == 0:
            print("[AUDIOBOOK] Exiting program...")
        elif user_input == 1:
            print("1) Rumpelstiltskin")
            self.book1()
        elif user_input == 2:
            print("2) The Emperor's New Clothes")
            self.book2()
        elif user_input == 3:
            print("3) Aladdin")
            self.book3()
        else:
            print("[AUDIOBOOK] Invalid choice or unrecognized number.")
    
        
    def intro(self):
        print("\n\n")
        print("****************************\n\n")
        print("AUDIOBOOK BEHAVIOUR STARTED")
        print("****************************\n\n")
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
                
        audiobook_confirm = 'mp3_files/audiobooks/audiobook_confirmation.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_confirm,))
        play_thread.start()
        play_thread.join()
        audiobook_select = 'mp3_files/audiobooks/audiobook_selection.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_select,))
        play_thread.start()
        play_thread.join()
        
        start_time = time.time()  # Record the start time
        while True:
            self.aruco_detect.tick_camera()

            if self.aruco_detect.emperor:
                print("[AUDIOBOOK] Aruco code detected, proceeding to Emperor's new clothes.")
                self.aruco_detect.emperor = False
                self.book2()
                break
            if self.aruco_detect.rumpelstiltskin:
                print("[AUDIOBOOK] Head touch detected, proceeding to rumpelstiltskin.")
                self.aruco_detect.rumpelstiltskin = False
                self.book1()
                break
            if self.aruco_detect.aladdin:
                print("[AUDIOBOOK] Head touch detected, proceeding to aladdin.")
                self.aruco_detect.aladdin = False
                self.book3()
                break
            # Check if 10 seconds have passed
            if time.time() - start_time > 10:
                print("No Aruco code detected")
                break
            time.sleep(0.1)




    def book1_solo(self): #Book 1 can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.book1()
    def book2_solo(self): #Book 2 can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.book2()
    def book3_solo(self): #Book 3 can run with exit
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
        self.book3()

    def book1(self): # Rumpelstiltskin

        print("[AUDIOBOOK] Playing opening line Rumpelstiltskin")
        audiobook_open_file = 'mp3_files/audiobooks/rumpelstiltskin_open.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_open_file,))
        play_thread.start()
        self.add_timer(5, self.joints_movement.nod, (1,2))
        play_thread.join()
               
        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing Rumpelstiltskin")
        audiobook_file = 'mp3_files/audiobooks/rumpelstiltskinV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        
        
        # Schedule all movements
        self.add_timer(4, self.joints_controller.move_neck, (4, 34))
        self.add_timer(4, self.cosmetics_controller.move_eyes, (4, 0.2))
        
        self.add_timer(8, self.cosmetics_controller.move_ears, (2, 0.6))
        self.add_timer(13, self.led_controller.fade_in_led, (5, (255,255,0), 250))
        self.add_timer(20, self.joints_movement.nod, (4,3))
        self.add_timer(24, self.led_controller.turn_off_led)
        self.add_timer(24, self.cosmetics_controller.move_ears, (2, 0))
        
        self.add_timer(26, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(26, self.joints_controller.move_neck, (3, 20))
        self.add_timer(30, self.joints_controller.move_pitch, (2, -15))
        self.add_timer(32, self.joints_controller.move_yaw, (4, 25))
        self.add_timer(33, self.led_controller.turn_on_led, ((255, 165, 0), 250))
        self.add_timer(38, self.led_controller.turn_off_led)

        self.add_timer(43, self.joints_controller.move_neck, (4, 34))
        self.add_timer(43, self.cosmetics_movement.open_eyes, (3,))
        self.add_timer(47, self.joints_controller.move_pitch, (2, 7))
        self.add_timer(49, self.joints_movement.shake, (4,3))

        self.add_timer(50, self.led_controller.fade_in_led, (2, (0, 0, 255), 250))
        self.add_timer(52, self.led_controller.fade_out_led, (2,))
        self.add_timer(54, self.led_controller.fade_in_led, (2, (0, 0, 255), 250))
        self.add_timer(56, self.led_controller.fade_out_led, (2,))

        self.add_timer(58, self.cosmetics_movement.blink_eyes, (2,3))
        self.add_timer(60, self.joints_controller.move_pitch, (4, -15))
        self.add_timer(64, self.joints_controller.move_neck, (4, 34))
        self.add_timer(70, self.cosmetics_controller.move_ears, (2, 0.6))
        self.add_timer(75, self.joints_controller.move_neck, (4, 28))
        self.add_timer(79, self.led_controller.fade_in_led, (2, (255,65,0), 250))
        self.add_timer(83, self.joints_movement.nod, (1.5,3))

        self.add_timer(88, self.joints_movement.nod, (1.5,3))
        self.add_timer(94, self.joints_controller.move_pitch, (4, -10))
        self.add_timer(94, self.joints_controller.move_yaw, (4, 25))
        self.add_timer(98, self.led_controller.turn_on_led, ((255, 255, 0), 250))
        self.add_timer(100, self.joints_controller.move_yaw, (4, 0))

        self.add_timer(105, self.cosmetics_movement.ear_outwards, (2,))
        self.add_timer(108, self.cosmetics_movement.ears_inwards, (2,))
        self.add_timer(111, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(111, self.joints_controller.move_neck, (3, 20))
        self.add_timer(112, self.led_controller.fade_in_led, (2, (255, 165, 0), 250))

        self.add_timer(115, self.joints_controller.move_neck, (4, 34))
        self.add_timer(122, self.joints_controller.move_yaw, (3, 0))
        self.add_timer(124, self.cosmetics_controller.move_ears, (2, 0.6))
        self.add_timer(127, self.joints_controller.move_neck, (4, 28))
        self.add_timer(129, self.joints_movement.nod, (3, 3))
        self.add_timer(133, self.led_controller.fade_in_led, (2, (255, 255, 0), 250))

        self.add_timer(137, self.cosmetics_movement.ear_outwards, (2,))
        self.add_timer(141, self.cosmetics_movement.ears_inwards, (2,))
        self.add_timer(143, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(143, self.joints_controller.move_neck, (3, 20))

        self.add_timer(151, self.cosmetics_movement.blink_eyes, (1, 2))
        self.add_timer(154, self.cosmetics_movement.ear_outwards, (2,))
        self.add_timer(156, self.joints_movement.nod, (3, 2))
        self.add_timer(161, self.cosmetics_movement.ears_inwards, (2,))

        self.add_timer(164, self.joints_controller.move_pitch, (4, 0))
        self.add_timer(170, self.joints_movement.shake, (1, 2))

        self.add_timer(173, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(179, self.joints_controller.move_pitch, (3, 7))
        self.add_timer(182, self.joints_controller.move_yaw, (3, 0))

        self.add_timer(186, self.joints_movement.nod, (2, 2))
        self.add_timer(188, self.led_controller.fade_in_led, (4, (255, 255, 0), 250))
        self.add_timer(192, self.led_controller.fade_out_led, (2,))

        self.add_timer(195, self.cosmetics_controller.move_ears, (2, 1))
        self.add_timer(200, self.joints_controller.move_pitch, (3, 7))
        self.add_timer(200, self.joints_controller.move_yaw, (3, -25))

        self.add_timer(204, self.joints_movement.shake, (3, 2))
        self.add_timer(207, self.cosmetics_controller.move_ears, (2, 0))
        self.add_timer(213, self.joints_controller.move_yaw, (2, 20))
        self.add_timer(216, self.joints_controller.move_yaw, (3, 0))
        self.add_timer(217, self.cosmetics_movement.blink_eyes, (1, 2))

        self.add_timer(221, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(221, self.joints_controller.move_neck, (3, 30))
        self.add_timer(223, self.led_controller.fade_in_led, (2, (255, 255, 0), 250))
        self.add_timer(226, self.joints_movement.shake, (3, 2))

        self.add_timer(234, self.joints_controller.move_neck, (3, 30))
        self.add_timer(234, self.led_controller.fade_in_led, (2, (0, 0, 255), 250))
        self.add_timer(234, self.led_controller.fade_out_led, (2,))

        self.add_timer(238, self.cosmetics_controller.move_ears, (2, 1))
        self.add_timer(244, self.joints_movement.nod, (3, 2))

        self.add_timer(247, self.joints_controller.move_yaw, (3, -25))
        self.add_timer(247, self.joints_controller.move_neck, (3, 20))
        self.add_timer(251, self.joints_controller.move_pitch, (2, -15))
        self.add_timer(253, self.joints_controller.move_yaw, (4, 25))
        self.add_timer(258, self.joints_movement.nod, (3, 3))
        self.add_timer(267, self.joints_movement.shake, (2, 2))

        self.add_timer(272, self.cosmetics_controller.move_ears, (2, 0.6))
        self.add_timer(277, self.joints_movement.nod, (3, 3))
        self.add_timer(280, self.joints_movement.shake, (2, 2))
        self.add_timer(287, self.cosmetics_controller.move_ears, (2, 0))
        self.add_timer(289, self.joints_controller.move_yaw, (3, 25))

        self.add_timer(294, self.cosmetics_movement.eyes_squint, (1,))
        self.add_timer(297, self.led_controller.fade_in_led, (2, (255, 65, 0), 250))
        self.add_timer(299, self.led_controller.fade_in_led, (2, (255, 165, 0), 250))

        self.add_timer(301, self.cosmetics_movement.open_eyes, (3,))
        self.add_timer(307, self.cosmetics_controller.move_ears, (2, 1))
        self.add_timer(310, self.joints_controller.move_yaw, (4, 0))
        self.add_timer(316, self.joints_controller.move_pitch, (3, -15))
        self.add_timer(321, self.cosmetics_controller.move_ears, (3, 0))
        self.add_timer(321, self.joints_controller.move_pitch, (3, 0))
        self.add_timer(329, self.joints_movement.shake, (2, 2))
        self.add_timer(334, self.joints_movement.shake, (2, 2))

        self.add_timer(337, self.joints_controller.move_pitch, (3, -15))
        self.add_timer(340, self.joints_movement.shake, (1, 1))
        self.add_timer(342, self.joints_movement.shake, (1, 1))
        self.add_timer(345, self.led_controller.fade_in_led, (2, (255, 0, 0), 250))
        self.add_timer(351, self.cosmetics_movement.blink_eyes, (1, 1))
        self.add_timer(354, self.cosmetics_movement.blink_eyes, (1, 1))
        self.add_timer(357, self.led_controller.fade_out_led, (2,))
        self.add_timer(359, self.cosmetics_controller.move_ears, (3, 1))
        self.add_timer(362, self.led_controller.toggle_led_sections, (15, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))

        self.add_timer(366, self.led_controller.fade_out_led, (2,))
        self.add_timer(366, self.cosmetics_controller.move_ears, (2, 0))

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

        print("[AUDIOBOOK] Playing ending line")
        audiobook_end_file = 'mp3_files/audiobooks/story_end.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_end_file,))
        play_thread.start()
        self.add_timer(0, self.joints_movement.nod, (1,12))
        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive() and play_open_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
        
    def book2(self):  # The Emperor's New Clothes
        print("[AUDIOBOOK] Playing opening line emperorsNewClothes")
        audiobook_open_file = 'mp3_files/audiobooks/emperorsNewClothes_open.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_open_file,))
        play_thread.start()
        self.add_timer(6, self.joints_movement.nod, (2,1))
        play_thread.join()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The Emperor's New Clothes")
        audiobook_file = 'mp3_files/audiobooks/emperorsNewClothes.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        # Schedule all movements
        self.add_timer(6, self.joints_controller.move_neck, (3, 20))  # Many years ago, there was an emperor
        self.add_timer(6, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Gold for royalty
        self.add_timer(10, self.cosmetics_controller.move_ears, (0.8, 0.8))  # Fond of new clothes
        self.add_timer(14, self.joints_controller.move_yaw, (3, -20))  # Spent all his money on dress
        self.add_timer(18, self.cosmetics_movement.eyes_squint, (2,))  # Didn't care about soldiers
        self.add_timer(23, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Displaying new clothes
        self.add_timer(29, self.joints_movement.nod, (2, 2))  # Different suit for each hour
        self.add_timer(34, self.joints_controller.move_pitch, (2, -10))  # Sitting in council
        self.add_timer(38, self.cosmetics_controller.move_ears, (0.6, 0.6))  # Sitting in his wardrobe
        self.add_timer(41, self.led_controller.fade_in_led, (3, (255, 223, 0), 200))  # Time passed merrily
        self.add_timer(48, self.joints_controller.move_yaw, (3, 20))  # Strangers arrived
        self.add_timer(53, self.cosmetics_movement.ear_outwards, (2))  # Rogues calling themselves weavers
        self.add_timer(58, self.led_controller.toggle_led, (5, (255, 255, 0), 250))  # Beautiful colors and patterns
        self.add_timer(64, self.joints_controller.move_neck, (3, 25))  # Clothes with wonderful properties
        self.add_timer(71, self.cosmetics_movement.eyes_squint, (2,))  # Invisible to the unfit
        self.add_timer(81, self.joints_movement.nod, (2, 2))  # Splendid clothes, thought the emperor
        self.add_timer(87, self.led_controller.fade_out_led, (3,))  # Find out unfit men
        self.add_timer(92, self.cosmetics_controller.move_ears, (0.8, 0.8))  # Distinguish the wise from the foolish
        self.add_timer(98, self.joints_controller.move_pitch, (2, -5))  # Weave the stuff immediately
        self.add_timer(101, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Large sums of money
        self.add_timer(108, self.cosmetics_controller.wagging, (2, 15))  # Pretended weavers set up looms
        self.add_timer(113, self.joints_controller.move_yaw, (3, -20))  # Affected to work busily
        self.add_timer(118, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Delicate silk and gold thread
        self.add_timer(123, self.cosmetics_movement.ears_inwards, (2,))  # Put into their knapsacks
        self.add_timer(128, self.joints_controller.move_neck, (3, 30))  # Continued pretended work
        self.add_timer(133, self.cosmetics_movement.eyes_squint, (2,))  # Emperor curious about progress
        self.add_timer(136, self.led_controller.fade_in_led, (3, (255, 223, 0), 200))  # Embarrassed about the cloth
        self.add_timer(144, self.joints_movement.shake, (2, 2))  # Simpleton or unfit would see nothing
        self.add_timer(153, self.cosmetics_controller.move_ears, (0.6, 0.6))  # Prefer sending someone else
        self.add_timer(159, self.joints_controller.move_pitch, (2, -10))  # Faithful old minister sent
        self.add_timer(168, self.led_controller.toggle_led, (5, (255, 255, 0), 250))  # People heard of the cloth
        self.add_timer(171, self.cosmetics_movement.ear_outwards, (2,))  # Anxious to learn about neighbors
        self.add_timer(176, self.joints_controller.move_yaw, (3, 20))  # Minister sent to inspect
        self.add_timer(182, self.cosmetics_movement.eyes_squint, (2,))  # Minister enters the hall
        self.add_timer(189, self.led_controller.fade_out_led, (3,))  # Knaves working at empty looms
        self.add_timer(194, self.joints_controller.move_neck, (3, 25))  # Minister cannot see the cloth
        self.add_timer(200, self.cosmetics_movement.close_eyes, (1,))  # What can be the meaning of this?
        self.add_timer(208, self.joints_movement.shake, (2, 2))  # Cannot discover any thread
        self.add_timer(213, self.led_controller.turn_on_led, ((255, 0, 0), 200))  # Impostors request him to come nearer
        self.add_timer(225, self.cosmetics_movement.ear_outwards, (2,))  # Asked if the design pleased him
        self.add_timer(231, self.joints_controller.move_pitch, (2, -5))  # Pointing to empty frames
        self.add_timer(239, self.cosmetics_movement.eyes_squint, (2,))  # Poor old minister looks and looks
        self.add_timer(242, self.led_controller.fade_in_led, (3, (255, 223, 0), 200))  # Nothing on the looms
        self.add_timer(248, self.joints_movement.shake, (2, 2))  # Is it possible I am a simpleton?
        self.add_timer(255, self.cosmetics_controller.move_ears, (0.6, 0.6))  # Never thought so myself
        self.add_timer(262, self.joints_controller.move_neck, (3, 30))  # Unfit for my office?
        self.add_timer(265, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Will never confess
        self.add_timer(270, self.cosmetics_movement.ear_outwards, (2))  # Knaves ask if the stuff pleases
        self.add_timer(275, self.joints_movement.nod, (2, 2))  # Oh, it is excellent
        self.add_timer(281, self.led_controller.fade_out_led, (3,))  # Minister praises the pattern
        self.add_timer(288, self.cosmetics_movement.eyes_squint, (2,))  # Will tell the emperor
        self.add_timer(293, self.joints_controller.move_pitch, (2, -10))  # Knaves describe the pattern
        self.add_timer(301, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Asked for more silk and gold
        self.add_timer(307, self.cosmetics_controller.wagging, (2, 15))  # Continued pretended work
        self.add_timer(314, self.joints_controller.move_yaw, (3, -20))  # Emperor sends another officer
        self.add_timer(321, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Same as with the minister
        self.add_timer(327, self.cosmetics_movement.close_eyes, (1,))  # Worked with apparent diligence
        self.add_timer(332, self.joints_controller.move_neck, (3, 25))  # Emperor sends another officer
        self.add_timer(336, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Cloth would soon be ready
        self.add_timer(341, self.cosmetics_movement.eyes_squint, (2,))  # Same as with the minister
        self.add_timer(345, self.joints_controller.move_neck, (3, 25))  # Surveyed the looms
        self.add_timer(345, self.cosmetics_movement.eyes_squint, (2,))  # Could see nothing
        self.add_timer(350, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Does not the stuff appear beautiful?
        self.add_timer(355, self.joints_movement.shake, (2, 2))  # Impostors ask the ambassador
        self.add_timer(359, self.cosmetics_controller.move_ears, (0.8, 0.8))  # Making gestures
        self.add_timer(364, self.led_controller.fade_in_led, (3, (255, 223, 0), 200))  # Talking of design and colors
        self.add_timer(373, self.joints_controller.move_pitch, (2, -10))  # I am not fit for my office
        self.add_timer(376, self.cosmetics_movement.close_eyes, (1,))  # That is very odd
        self.add_timer(382, self.led_controller.turn_on_led, ((255, 0, 0), 200))  # Praised the stuff he could not see
        self.add_timer(386, self.joints_movement.nod, (2, 2))  # Delighted with colors and patterns
        self.add_timer(396, self.cosmetics_movement.ear_outwards, (2,))  # The cloth is extraordinarily magnificent
        self.add_timer(402, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Whole city talking of the cloth
        self.add_timer(408, self.joints_controller.move_yaw, (3, -20))  # Emperor wishes to see the cloth
        self.add_timer(412, self.cosmetics_movement.eyes_squint, (2,))  # Accompanied by officers
        self.add_timer(419, self.led_controller.fade_out_led, (3,))  # Honest men admired the cloth
        self.add_timer(424, self.joints_controller.move_neck, (3, 30))  # Crafty impostors working diligently
        self.add_timer(430, self.cosmetics_controller.wagging, (2, 15))  # No thread through the looms
        self.add_timer(439, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Work is magnificent
        self.add_timer(446, self.joints_movement.nod, (2, 2))  # Splendid design and colors
        self.add_timer(454, self.cosmetics_controller.move_ears, (0.6, 0.6))  # Pointed to empty frames
        self.add_timer(458, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Imagined everyone could see
        self.add_timer(463, self.joints_controller.move_pitch, (2, -5))  # Emperor sees nothing
        self.add_timer(472, self.cosmetics_movement.close_eyes, (1,))  # Am I a simpleton?
        self.add_timer(479, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Oh, the cloth is charming
        self.add_timer(486, self.joints_controller.move_yaw, (3, 20))  # Smiled graciously at empty looms
        self.add_timer(492, self.cosmetics_movement.eyes_squint, (2,))  # Would not admit he saw nothing
        self.add_timer(500, self.led_controller.fade_out_led, (3,))  # Retinue strained their eyes
        self.add_timer(506, self.joints_movement.shake, (2, 2))  # Could see no more than others
        self.add_timer(510, self.cosmetics_controller.move_ears, (0.8, 0.8))  # Exclaimed, "Oh, how beautiful"
        self.add_timer(515, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Advised new clothes for the procession
        self.add_timer(522, self.joints_movement.nod, (2, 2))  # Magnificent, charming, excellent
        self.add_timer(528, self.cosmetics_controller.wagging, (2, 15))  # Everyone uncommonly gay
        self.add_timer(531, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Emperor presented impostors with honors
        self.add_timer(544, self.joints_controller.move_neck, (3, 25))  # Rogues sat up all night
        self.add_timer(550, self.cosmetics_movement.eyes_squint, (2,))  # 16 lights burning
        self.add_timer(554, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Pretended to finish the suit
        self.add_timer(560, self.joints_controller.move_yaw, (3, -20))  # Cut the air with scissors
        self.add_timer(564, self.cosmetics_movement.ear_outwards, (2,))  # Sewed with needles without thread
        self.add_timer(569, self.led_controller.fade_out_led, (3,))  # Emperor's new clothes are ready
        self.add_timer(574, self.joints_movement.nod, (2, 2))  # Emperor and grandees arrive
        self.add_timer(579, self.cosmetics_controller.move_ears, (0.6, 0.6))  # Raised arms as if holding something
        self.add_timer(582, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Here are your trousers
        self.add_timer(589, self.joints_controller.move_pitch, (2, -10))  # The whole suit is light as a cobweb
        self.add_timer(594, self.cosmetics_movement.close_eyes, (1,))  # One might fancy nothing is on
        self.add_timer(598, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Great virtue of the cloth
        self.add_timer(603, self.joints_movement.shake, (2, 2))  # Courtiers agree, though they see nothing
        self.add_timer(612, self.cosmetics_movement.eyes_squint, (2,))  # Emperor undressed
        self.add_timer(616, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Pretended to array him in new suit
        self.add_timer(622, self.joints_controller.move_yaw, (3, 20))  # Emperor turns before the mirror
        self.add_timer(629, self.cosmetics_movement.ear_outwards, (2))  # How splendid he looks
        self.add_timer(634, self.led_controller.fade_out_led, (3,))  # Everyone admires the suit
        self.add_timer(640, self.joints_movement.nod, (2, 2))  # What a design, what colors
        self.add_timer(645, self.cosmetics_controller.wagging, (2, 15))  # Royal robes
        self.add_timer(654, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # Canopy waiting
        self.add_timer(656, self.joints_controller.move_pitch, (2, -5))  # Emperor ready for procession
        self.add_timer(662, self.cosmetics_movement.eyes_squint, (2,))  # Turning before the mirror
        self.add_timer(665, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Examining his suit
        self.add_timer(670, self.joints_controller.move_neck, (3, 25))  # Lords of the bedchamber
        self.add_timer(675, self.cosmetics_movement.ears_inwards, (2,))  # Pretended to carry the train
        self.add_timer(680, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Would not betray unfitness
        self.add_timer(690, self.joints_movement.nod, (2, 2))  # Emperor walked under the canopy
        self.add_timer(695, self.cosmetics_controller.wagging, (2, 15))  # People admired the clothes
        self.add_timer(700, self.led_controller.fade_out_led, (3,))  # Oh, how beautiful
        self.add_timer(706, self.joints_controller.move_yaw, (3, -20))  # Magnificent train
        self.add_timer(710, self.cosmetics_movement.eyes_squint, (2,))  # Gracefully the scarf hangs
        self.add_timer(714, self.led_controller.turn_on_led, ((255, 223, 0), 200))  # No one admitted they saw nothing
        self.add_timer(720, self.joints_movement.shake, (2, 2))  # Simpleton or unfit
        self.add_timer(728, self.cosmetics_movement.close_eyes, (1,))  # Emperor's suits made an impression
        self.add_timer(736, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Child says, "He has nothing on"
        self.add_timer(740, self.joints_controller.move_pitch, (2, -10))  # Voice of innocence
        self.add_timer(744, self.cosmetics_movement.ear_outwards, (2,))  # Whispered from one to another
        self.add_timer(750, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # People cried out
        self.add_timer(755, self.joints_movement.shake, (2, 2))  # Emperor vexed
        self.add_timer(758, self.cosmetics_movement.eyes_squint, (2,))  # Procession must go on
        self.add_timer(763, self.led_controller.fade_out_led, (3,))  # Lords pretended to hold the train
        self.add_timer(767, self.joints_controller.move_neck, (3, 30))  # No train to hold
        self.add_timer(774, self.cosmetics_movement.close_eyes, (2,))  # End of the story

        # Wait for all threads to finish (including the audio playback)
        play_thread.join()

        print("[AUDIOBOOK] Playing ending line")
        audiobook_end_file = 'mp3_files/audiobooks/story_end.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_end_file,))
        play_thread.start()
        self.add_timer(0, self.joints_movement.nod, (2,1))
        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book2...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

    def book3(self): # Aladdin
        
        print("[AUDIOBOOK] Playing opening line Aladdin")
        audiobook_open_file = 'mp3_files/audiobooks/aladdin_open.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_open_file,))
        play_thread.start()
        self.add_timer(4, self.joints_movement.nod, (2,1))
        play_thread.join()
        

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing Aladdin")
        audiobook_file = 'mp3_files/audiobooks/aladdin.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(4, self.joints_controller.move_neck, (3, 25))  # Aladdin was the son of a poor widow.
        self.add_timer(4, self.cosmetics_controller.move_eyes, (0.5, 0.5))  # Neutral eyes
        self.add_timer(7, self.joints_controller.move_yaw, (2, -20))  # One day a wicked magician met him and said,
        self.add_timer(7, self.cosmetics_controller.move_ears, (0.8, 0.8))  # Ears outward, listening
        self.add_timer(10, self.led_controller.fade_in_led, (3, (255, 255, 0), 200))  # Take this ring and come with me.
        self.add_timer(14, self.joints_movement.nod, (2, 2))  # If you do as I say, you will be well rewarded.
        self.add_timer(19, self.joints_controller.move_pitch, (2, -10))  # Magician stopped and pointing to a stone said
        self.add_timer(23, self.cosmetics_movement.eyes_squint, (2,))  # Lift up that stone and you will see a ladder
        self.add_timer(26, self.joints_controller.move_neck, (3, 30))  # Go down the ladder, open the door, and keep walking
        self.add_timer(30, self.led_controller.toggle_led_sections, (5, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))  # Beautiful garden
        self.add_timer(33, self.cosmetics_controller.wagging, (2, 15))  # Find a little lamp
        self.add_timer(38, self.led_controller.turn_on_led, ((255, 165, 0), 250))  # Aladdin climbed down, found jewels
        self.add_timer(42, self.joints_controller.move_yaw, (3, 20))  # Rare precious jewels
        self.add_timer(47, self.cosmetics_movement.ear_outwards, (2,))  # Picked up the lamp and jewels
        self.add_timer(51, self.led_controller.turn_off_led)  # Magician saw him and cried
        self.add_timer(53, self.joints_movement.shake, (2, 2))  # Hand me the lamp. But Aladdin refused.
        self.add_timer(59, self.cosmetics_movement.close_eyes, (2,))  # Magician angrily closed the stone
        self.add_timer(59, self.joints_controller.move_neck, (3, 40))  # Poor Aladdin wept bitterly
        self.add_timer(65, self.led_controller.fade_in_led, (3, (0, 0, 255), 200))  # Rubbed the ring
        self.add_timer(69, self.cosmetics_movement.open_eyes, (1,))  # Genie appeared
        self.add_timer(72, self.joints_movement.nod, (2, 2))  # I am the slave of the ring
        self.add_timer(77, self.led_controller.toggle_led_sections, (5, (0, 0, 255), (0, 0, 255), (0, 0, 255), 250))  # Aladdin asked for help
        self.add_timer(81, self.joints_controller.move_pitch, (2, -5))  # Stone flew open, Aladdin escaped
        self.add_timer(85, self.cosmetics_controller.wagging, (2, 15))  # Aladdin and his mother became wealthy
        self.add_timer(93, self.joints_controller.move_yaw, (3, -20))  # Emperor's daughter came to town
        self.add_timer(96, self.cosmetics_controller.move_ears, (0.6, 0.6))  # Aladdin saw the princess
        self.add_timer(100, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # Fell in love with her
        self.add_timer(105, self.joints_movement.nod, (2, 2))  # His mother promised to help
        self.add_timer(109, self.led_controller.turn_on_led, ((255, 165, 0), 250))  # Polishing the old lamp
        self.add_timer(113, self.cosmetics_movement.eyes_squint, (2,))  # Another genie appeared
        self.add_timer(118, self.joints_controller.move_neck, (3, 25))  # I am the slave of the lamp
        self.add_timer(123, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Wish to marry the princess
        self.add_timer(127, self.cosmetics_controller.wagging, (2, 15))  # Genie brought rare jewels
        self.add_timer(135, self.joints_controller.move_pitch, (2, -10))  # Emperor pleased with gifts
        self.add_timer(143, self.led_controller.turn_on_led, ((255, 255, 0), 250))  # Aladdin went to the palace
        self.add_timer(149, self.cosmetics_movement.ear_outwards, (2,))  # Married the princess
        self.add_timer(155, self.led_controller.toggle_led_sections, (5, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))  # Genie built a palace
        self.add_timer(158, self.joints_controller.move_yaw, (3, 20))  # Old man passed the palace
        self.add_timer(161, self.cosmetics_movement.close_eyes, (1,))  # New lamps for old ones
        self.add_timer(165, self.led_controller.turn_off_led)  # Princess gave the old lamp
        self.add_timer(169, self.joints_controller.move_neck, (3, 40))  # Wicked magician took the lamp
        self.add_timer(173, self.cosmetics_movement.close_eyes, (2,))  # Palace disappeared
        self.add_timer(176, self.led_controller.fade_in_led, (3, (0, 0, 255), 200))  # Aladdin in despair
        self.add_timer(181, self.joints_movement.nod, (2, 2))  # Called the slave of the ring
        self.add_timer(185, self.cosmetics_movement.open_eyes, (1,))  # Slave took him to the palace
        self.add_timer(190, self.joints_controller.move_yaw, (3, -20))  # Princess saw Aladdin
        self.add_timer(192, self.led_controller.toggle_led_sections, (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 250))  # Overjoyed to see him
        self.add_timer(196, self.cosmetics_controller.wagging, (2, 15))  # Planned to get the lamp
        self.add_timer(200, self.joints_controller.move_pitch, (2, -5))  # Princess invited magician to tea
        self.add_timer(203, self.cosmetics_movement.eyes_squint, (2,))  # Magician delighted
        self.add_timer(207, self.led_controller.turn_off_led)  # Princess dropped sleeping powder
        self.add_timer(210, self.cosmetics_movement.close_eyes, (2,))  # Magician fell asleep
        self.add_timer(213, self.joints_controller.move_yaw, (3, 20))  # Aladdin took the lamp
        self.add_timer(216, self.led_controller.turn_on_led, ((255, 165, 0), 250))  # Rubbed the lamp
        self.add_timer(219, self.cosmetics_movement.open_eyes, (1,))  # Genie appeared
        self.add_timer(221, self.joints_movement.nod, (2, 2))  # Aladdin asked for transport
        self.add_timer(225, self.led_controller.toggle_led_sections, (5, (0, 255, 0), (0, 255, 0), (0, 255, 0), 250))  # Palace returned
        self.add_timer(229, self.cosmetics_movement.ears_inwards, (2,))  # Magician still asleep
        
        self.add_timer(233, self.led_controller.fade_out_led, (5,))  # Happy ending
        self.add_timer(237, self.cosmetics_controller.wagging, (3, 15))  # Lived happily ever after


        # Wait for all threads to finish (including the audio playback)
        play_thread.join()
        
        print("[AUDIOBOOK] Playing ending line")
        audiobook_end_file = 'mp3_files/audiobooks/story_end.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_end_file,))
        play_thread.start()
        self.add_timer(0, self.joints_movement.nod, (2,1))
        play_thread.join()
        
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book3...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

    ###Exit###
    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            if self.aruco_detect.exit_behaviour :
                self.stop_flag = True
                self.audio_player.stop()
                exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files/i_will_stop.mp3',))
                exit_behaviour_thread.start()
                exit_behaviour_thread.join()
                print("[AUDIOBOOK] Exit behaviour detected, stopping audiobook.")

            time.sleep(0.1)

    def safe_execute(self, func, args):
        """Execute function only if stop flag is not set and parent thread is still alive."""
        if not self.stop_flag and (self.parent_thread and self.parent_thread.is_alive()):
            func(*args)
        else:
            print(f"[AUDIOBOOK] Skipping {func.__name__} because parent thread has exited.")

    # Function to safely start actions
    def add_timer(self, delay, func, args=()):
        timer = threading.Timer(delay, lambda: self.safe_execute(func, args))
        self.timers.append(timer)
        timer.start()