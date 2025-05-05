#!/usr/bin/python3

# ----------------------------------------------
# Title: audiobooks.py
# Description: Play audiobooks with body movement
# Author: Jasmine 
# Date created: Dec 24, 2024
# Date modified: Feb 24, 2024
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
from actuators.remote import connect_remote, receive_data, send_data, close_connection


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
        self.parent_intro_thread= None
        self.timers = []  # List to track timers
        self.remote_data=[0,0,0,0,0]


    def run(self):
        print("[AUDIOBOOK] Running audiobooks behavior")
        # Start the check_exit_flag thread
        self.parent_intro_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()
                
        #audiobook_confirm = 'mp3_files/audiobook_confirmation.mp3' # NOT slushy. Requires a different file
        #play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_confirm,))
        #play_thread.start()
        #play_thread.join()
        audiobook_select = 'mp3_files_slushy/audiobooks/story_choice.mp3'
        play_book_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_select,))
        play_book_thread.start()
        play_book_thread.join()


       
    def book1(self): # The invisible alligators
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The Invisible Alligators")
        audiobook_file = 'mp3_files_slushy/audiobooks/book1-the_invisible_alligators.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()

        # Schedule all movements
        self.add_timer(0, self.joints_controller.move_neck, (3, 20))  # A young monkey named Ivy
        self.add_timer(2.5, self.cosmetics_controller.move_eyes, (2, 0.3))  # woke up one morning
        self.add_timer(4.5, self.led_controller.fade_in_led, (2, (255, 165, 0), 200))  # knew there was trouble
        self.add_timer(6.5, self.joints_movement.nod, (1, 2))  # hopped out of bed
        self.add_timer(9.5, self.cosmetics_controller.move_ears, (2, 0.4))  # found the bridge broken
        self.add_timer(11, self.led_controller.toggle_led, (3, (255, 0, 0), 150))  # It took her forever to fix it
        self.add_timer(14.5, self.joints_controller.move_yaw, (2, -20))  # stairs were covered in toys
        self.add_timer(17, self.cosmetics_movement.eyes_squint, (1,))  # tripped and had to pick them up
        self.add_timer(19, self.led_controller.fade_out_led, (2,))  # Then her llamas were loose
        self.add_timer(22, self.joints_controller.move_pitch, (2, -10))  # took six tries to herd them
        self.add_timer(26, self.cosmetics_controller.move_eyes, (2, 0.6))  # stop eating her homework
        self.add_timer(28, self.led_controller.turn_on_led, ((0, 255, 0), 200))  # so late that she missed math class
        self.add_timer(33, self.joints_movement.shake, (2, 2))  # homework covered in bites
        self.add_timer(35.5, self.cosmetics_controller.move_ears, (2, 0))  # Tonight, she'd put a stop to this
        self.add_timer(38.3, self.led_controller.fade_in_led, (3, (0, 0, 255), 200))  # That night, Ivy stayed awake
        self.add_timer(41, self.joints_controller.move_neck, (3, 30))  # heard rustling under her bed
        self.add_timer(44, self.cosmetics_movement.eye_wink, ("right", 1, 1))  # flipped it over and found
        self.add_timer(46, self.led_controller.toggle_led, (5, (255, 255, 0), 250))  # invisible alligators!
        self.add_timer(48, self.joints_controller.move_yaw, (3, 20))  # One of the alligators said
        self.add_timer(50.2, self.cosmetics_controller.move_eyes, (2, 0.2))  # "We cause trouble like this"
        self.add_timer(53.5, self.led_controller.turn_off_led)  # "We're just trying to help"
        self.add_timer(55.5, self.joints_movement.nod, (2, 1))  # "Let me show you"
        self.add_timer(58, self.cosmetics_controller.move_ears, (2, 0.6))  # Curious, Ivy followed him
        self.add_timer(59.5, self.led_controller.fade_in_led, (3, (255, 0, 255), 200))  # deep into their catacombs
        self.add_timer(63.5, self.joints_controller.move_pitch, (2, -15))  # hiding the sheep remotes
        self.add_timer(65.8, self.cosmetics_movement.eyes_squint, (2,))  # swapping cake mix for broccoli
        self.add_timer(68.6, self.led_controller.toggle_led, (4, (0, 255, 255), 200))  # singing hippos to sleep
        self.add_timer(71.4, self.joints_controller.move_neck, (3, 20))  # "But why?" Ivy asked
        self.add_timer(73.7, self.cosmetics_controller.move_eyes, (2, 0.5))  # "Why can't everything just go right?"
        self.add_timer(75.6, self.led_controller.fade_out_led, (3,))  # The alligator sighed
        self.add_timer(79.6, self.joints_movement.shake, (2, 1))  # He took her to their headquarters
        self.add_timer(83.2, self.cosmetics_controller.move_ears, (2, 0))  # her book
        self.add_timer(85.5, self.led_controller.turn_on_led, ((255, 165, 0), 200))  # all the troubles she'd faced
        self.add_timer(87.5, self.joints_controller.move_yaw, (3, -20))  # all the things she'd learned
        self.add_timer(90, self.cosmetics_movement.close_eyes, (1,))  # "I don't get it"
        self.add_timer(92, self.led_controller.fade_in_led, (3, (255, 0, 0), 200))  # She left, disappointed
        self.add_timer(97, self.joints_controller.move_pitch, (2, -10))  # found rocks on the stairs
        self.add_timer(99, self.cosmetics_controller.move_eyes, (2, 0.3))  # a broken bridge
        self.add_timer(105, self.led_controller.toggle_led, (5, (0, 255, 0), 200))  # very dangerous blue goats
        self.add_timer(104.5, self.joints_movement.nod, (1, 3))  # she cleared the rocks
        self.add_timer(106, self.cosmetics_controller.move_ears, (2, 0.4))  # fixed the bridge
        self.add_timer(107.5, self.led_controller.turn_off_led)  # herded the goats
        self.add_timer(111, self.joints_controller.move_neck, (3, 25))  # climbed into bed
        self.add_timer(114, self.cosmetics_movement.eye_wink, ("right", 1, 2))  # dreaming of Math
        self.add_timer(115.8, self.led_controller.fade_in_led, (3, (0, 0, 255), 200))  # How did she know?
        self.add_timer(118, self.joints_controller.move_yaw, (3, 20))  # Maybe the invisible alligators
        self.add_timer(120.5, self.cosmetics_controller.move_eyes, (2, 0.2))  # did help after all
        self.add_timer(124, self.led_controller.toggle_led, (5, (255, 255, 0), 250))  # they might visit you tonight
        self.add_timer(126.3, self.joints_movement.shake, (2, 1))  # The End
        play_thread.join()
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)

        # Wait for all threads to finish (including the audio playback)
        self.add_timer(0, self.joints_movement.nod, (2,1))
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)
        play_end_thread.join()
        
        

    def book2(self): # The day the crayons quit
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The day the crayons quit")
        audiobook_file = 'mp3_files_slushy/audiobooks/book2-the_day_the_crayons_quitV2.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        
        # Opening
        self.add_timer(0, self.joints_movement.nod, (2, 2))  # "ok. I will be reading:"
        self.add_timer(0, self.led_controller.turn_off_led, ())
        self.add_timer(2, self.cosmetics_movement.blink, (0.5,))  # "The Day the Crayons Quit."
        
        self.add_timer(4, self.joints_controller.move_yaw, (1, 30))  # "One day in class, Duncan"
        self.add_timer(5.7, self.cosmetics_movement.blink, (0.5,))  # "went to take out his crayons"
        self.add_timer(8, self.joints_controller.move_all, (1, 0, 8, 30))  # "found a stack of letters"

        # RED Crayon's Letter
        self.add_timer(12, self.joints_controller.move_all, (1, 0, -10, 15))  # ""Hey Duncan, it's me,"
        self.add_timer(13.7, self.cosmetics_movement.blink, (0.5,))  # "RED Crayon."
        self.add_timer(13.7, self.led_controller.turn_on_led, ((206, 32, 41), 200))  # Fire Engine Red
        self.add_timer(15, self.cosmetics_movement.eyes_squint, (1,))  # "We need to talk."
        self.add_timer(15, self.cosmetics_movement.ear_outwards, (1,))
        self.add_timer(17.5, self.joints_controller.move_yaw, (1, 30))  # "work harder than any"
        #self.add_timer(17.5, self.led_controller.turn_on_led, ((220, 20, 60), 200))  # Crimson
        self.add_timer(20.6, self.cosmetics_movement.open_eyes, (1,))  # "All year long"
        self.add_timer(20.6, self.joints_controller.move_yaw, (1, 0))
        self.add_timer(24, self.cosmetics_movement.blink, (0.5,))  # "fire engines,"
        self.add_timer(25, self.cosmetics_movement.blink, (0.5,))  # "apples,"
        self.add_timer(25.8, self.cosmetics_movement.blink, (0.5,))  # "strawberries"
        self.add_timer(27, self.cosmetics_movement.eyes_squint, (1,))  # "EVERYTHING ELSE"
        self.add_timer(27, self.cosmetics_movement.ears_inwards, (1,))
        #self.add_timer(27, self.led_controller.turn_on_led, ((255, 36, 0), 200))  # Scarlet
        self.add_timer(28.6, self.cosmetics_movement.blink, (0.5,))  # "I even work on Holidays!"
        self.add_timer(28.6, self.joints_movement.nod, (2, 2))
        self.add_timer(30.9, self.joints_controller.move_yaw, (1, 30))  # "Santas at Christmas"
        self.add_timer(33.4, self.joints_controller.move_yaw, (1, -30))  # "hearts at Valentine's Day"
        self.add_timer(36.1, self.joints_controller.move_all, (1, 0, -10, 30))  # "I need a REST!"
        self.add_timer(37, self.cosmetics_movement.eyes_squint, (1,))  # "Your overworked friend,"
        self.add_timer(40, self.cosmetics_movement.blink, (0.5,))  # "RED Crayon""
        self.add_timer(41.2, self.led_controller.turn_off_led, ())

        # GREEN Crayon's Letter
        self.add_timer(42.4, self.joints_controller.move_neck, (1, 15))  # ""Dear Duncan,"
        self.add_timer(42.4, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(43.5, self.led_controller.turn_on_led, ((50, 201, 50), 200))  # Green
        self.add_timer(44.6, self.cosmetics_movement.ears_facing_front, (1,))  # "two reasons."
        #self.add_timer(46.8, self.cosmetics_movement.blink, (0.5,))  # "One is to say"
        self.add_timer(48, self.joints_movement.nod, (2, 2))  # "I like my work"
        self.add_timer(49, self.cosmetics_movement.blink, (0.5,))  # "crocodiles,"
        #self.add_timer(50.6, self.cosmetics_movement.blink, (0.5,))  # "trees,"
        self.add_timer(51.5, self.cosmetics_movement.blink, (0.5,))  # "dinosaurs,"
        self.add_timer(52, self.joints_movement.shake, (1.5, 2))  # "no problems"
        self.add_timer(54, self.joints_movement.nod, (1.5, 2))  # "congratulate"
        self.add_timer(54, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(55.7, self.cosmetics_movement.ear_outwards, (1,))  # "'colouring things green'"
        #self.add_timer(55.7, self.cosmetics_controller.wagging, (2, 15))
        #self.add_timer(55.7, self.led_controller.turn_on_led, ((80, 200, 120), 200))  # Emerald
        self.add_timer(58.5, self.cosmetics_movement.ears_inwards, (1,))  # "The second reason"
        #self.add_timer(58.5, self.led_controller.turn_on_led, ((128, 128, 0), 200))  # Olive
        self.add_timer(61, self.cosmetics_controller.move_one_ear, (0.5, 1, "right"))  # "Yellow Crayon"
        self.add_timer(62, self.cosmetics_controller.move_one_ear, (0.5, 1, "left"))  # "Orange Crayon"
        self.add_timer(63.4, self.joints_movement.shake, (1.5, 2))  # "no longer speaking"
        self.add_timer(65.2, self.cosmetics_movement.ears_inwards, (1,))  # "Both crayons feel"
        self.add_timer(65.2, self.cosmetics_movement.blink, (0.3,))
        self.add_timer(66.5, self.cosmetics_movement.blink, (0.3,))
        self.add_timer(68.6, self.joints_controller.move_neck, (1, 30))  # "Please settle this"
        self.add_timer(70.2, self.joints_movement.shake, (1, 1))  # "they're driving"
        self.add_timer(71.2, self.cosmetics_movement.blink, (0.5,))  # "rest of us CRAZY!"
        self.add_timer(72.5, self.joints_movement.nod, (2, 2))  # "Your happy friend,"
        self.add_timer(72.5, self.cosmetics_controller.wagging, (2, 15))
        self.add_timer(74.6, self.led_controller.turn_off_led, ())

        # YELLOW Crayon's Letter
        self.add_timer(77, self.joints_movement.nod, (2, 2))  # ""Dear Duncan,"
        self.add_timer(77, self.led_controller.turn_on_led, ((255, 255, 0), 200))  # Yellow
        self.add_timer(78.6, self.cosmetics_movement.ear_outwards, (1,))  # "Yellow Crayon here."
        self.add_timer(78.6, self.joints_controller.move_yaw, (1, 20))
        self.add_timer(80.4, self.joints_controller.move_pitch, (1, -15))  # "I am the colour"
        self.add_timer(80.4, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(81.8, self.joints_controller.move_yaw, (1, 0))  # "I would tell him"
        self.add_timer(82.8, self.cosmetics_movement.ears_inwards, (1,))  # "no longer speaking"
        self.add_timer(84.8, self.joints_controller.move_all, (1, 0, 5, 30))  # "I can PROVE"
        self.add_timer(84.8, self.cosmetics_movement.eyes_squint, (1,))
        #self.add_timer(84.8, self.led_controller.turn_on_led, ((218, 165, 32), 200))  # Goldenrod
        self.add_timer(88, self.joints_controller.move_neck, (1, 15))  
        self.add_timer(88, self.cosmetics_movement.ear_outwards, (1,))  # "Last Tuesday"
        self.add_timer(89, self.cosmetics_movement.open_eyes, (1,))  # "used me to colour"
        self.add_timer(89, self.joints_controller.move_pitch, (1, -10))
        self.add_timer(90.6, self.cosmetics_controller.wagging, (2, 15))  # "sun in your book"
        self.add_timer(93, self.joints_controller.move_yaw, (1, 30))  # "page 7"
        self.add_timer(96, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(96, self.joints_controller.move_yaw, (1, -30))  # "field of yellow corn"
        self.add_timer(96, self.cosmetics_controller.wagging, (1, 20))
        #self.add_timer(96, self.led_controller.turn_on_led, ((255, 212, 59), 200))  # Sunflower
        self.add_timer(101, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(101, self.joints_controller.move_yaw, (1, 0))  # "Your pal"
        self.add_timer(103.5, self.joints_movement.nod, (2, 2))  # "Yellow Crayon""
        self.add_timer(104.7, self.led_controller.turn_off_led, ())

        # ORANGE Crayon's Letter
        self.add_timer(105.8, self.cosmetics_movement.blink, (0.5,))  # ""Dear Duncan, I see"
        self.add_timer(105.8, self.led_controller.turn_on_led, ((255, 170, 0), 200))  # Orange
        self.add_timer(107.4, self.cosmetics_movement.ears_inwards, (1,))  # "Yellow Crayon"
        self.add_timer(107.4, self.joints_controller.move_yaw, (1, 30))
        self.add_timer(109.8, self.cosmetics_movement.eyes_squint, (1,))  # "the BIG WHINER"
        self.add_timer(111.1, self.joints_controller.move_yaw, (1, 0))  # "Anyway, could you"
        self.add_timer(111.1, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(114, self.joints_movement.shake, (1.5, 2))  # "NOT the colour"
        self.add_timer(115.6, self.cosmetics_movement.blink, (0.5,))  # "I would, but"
        self.add_timer(116.6, self.joints_movement.shake, (1, 1))  # "no longer speaking"
        self.add_timer(118.6, self.joints_controller.move_all, (1, 0, -15, 15))  # "We both know"
        self.add_timer(118.6, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(118.6, self.cosmetics_movement.ear_outwards, (1,))
        self.add_timer(121.6, self.cosmetics_movement.open_eyes, (1,))  # "on Thursday"
        #self.add_timer(121.6, self.led_controller.turn_on_led, ((242, 133, 0), 200))  # Tangerine
        self.add_timer(124.5, self.joints_movement.nod, (1.5, 2))  # "Monkey Island"
        #self.add_timer(125.5, self.joints_movement.nod, (1, 1))  # "Meet the Zookeeper"
        self.add_timer(127.1, self.cosmetics_movement.ears_inwards, (1,))  # "Day at the Zoo"
        #self.add_timer(127.1, self.cosmetics_controller.wagging, (2, 15))
        self.add_timer(129.3, self.cosmetics_movement.eye_wink, (1, 1, "left"))  # "Orange you glad"
        self.add_timer(129.3, self.joints_controller.move_yaw, (1, 20))
        #self.add_timer(129.3, self.led_controller.turn_on_led, ((237, 145, 33), 200))  # Carrot
        self.add_timer(131.6, self.joints_movement.nod, (2, 2))  # "Your Pal"
        self.add_timer(133.9, self.cosmetics_movement.blink, (0.5,))  # "Orange Crayon""
        self.add_timer(134.8, self.led_controller.turn_off_led, ())

        # BLUE Crayon's Letter
        self.add_timer(136, self.led_controller.turn_on_led, ((0, 0, 255), 200))  # Blue
        self.add_timer(137.5, self.joints_movement.nod, (2, 2))  # "Dear Duncan, It has"
        self.add_timer(137.5, self.cosmetics_movement.ear_outwards, (1,))
        self.add_timer(139.5, self.joints_controller.move_neck, (1, 25))  # "great being your FAVOURITE"
        self.add_timer(139.5, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(140.6, self.joints_controller.move_neck, (1, 30))  # "year before."
        self.add_timer(140.6, self.cosmetics_movement.eyes_squint, (1,))
        self.add_timer(141.9, self.joints_controller.move_neck, (1, 35))  # "YEAR before THAT!"
        self.add_timer(141.9, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(143.8, self.cosmetics_movement.open_eyes, (1,))  # "I have really enjoyed"
        self.add_timer(143.8, self.joints_movement.nod, (1, 1))
        self.add_timer(145.8, self.cosmetics_movement.blink, (0.5,))  # "OCEANS,"
        #self.add_timer(146.4, self.cosmetics_movement.blink, (0.3,))  # "lakes,"
        self.add_timer(147, self.cosmetics_movement.blink, (0.5,))  # "rivers,"
        #self.add_timer(147.6, self.cosmetics_movement.blink, (0.3,))  # "raindrops,"
        self.add_timer(148.5, self.cosmetics_movement.ears_inwards, (1,))  # "rain CLOUDS"
        self.add_timer(151.7, self.joints_controller.move_pitch, (1, 5))  # "I am so short"
        #self.add_timer(151.7, self.led_controller.turn_on_led, ((0, 0, 128), 200))  # Navy
        self.add_timer(152.6, self.cosmetics_movement.eyes_squint, (1,))  # "short and stubby"
        self.add_timer(154.2, self.joints_controller.move_neck, (1, 15))  # "can't even see over"
        self.add_timer(155.6, self.joints_movement.shake, (1.5, 2))  # "crayon box railing"
        self.add_timer(158, self.cosmetics_movement.open_eyes, (1,))  # "I need a BREAK!"
        #self.add_timer(158, self.led_controller.turn_on_led, ((100, 149, 237), 200))  # Cornflower
        self.add_timer(159.9, self.cosmetics_movement.blink, (0.5,))  # "Your very stubby friend"
        self.add_timer(163.5, self.led_controller.turn_off_led, ())

        # PINK Crayon's Letter
        self.add_timer(164.5, self.cosmetics_movement.blink, (0.5,))  # ""Dear Duncan,"
        self.add_timer(164.5, self.joints_controller.move_pitch, (0.5, -20))  # head up fast
        self.add_timer(164.5, self.led_controller.turn_on_led, ((240, 29, 215), 200))  # Pink
        self.add_timer(165.7, self.cosmetics_movement.eyes_squint, (1,))  # "OKAY, LISTEN HERE"
        self.add_timer(165.7, self.cosmetics_movement.ear_outwards, (1,))
        self.add_timer(167.6, self.joints_movement.shake, (1.5, 2))  # "not used me"
        self.add_timer(168.5, self.cosmetics_movement.open_eyes, (1,))  # "ONCE in the past year"
        self.add_timer(169.7, self.joints_controller.move_yaw, (1, 30))  # "Is it because"
        self.add_timer(169.7, self.cosmetics_movement.close_eyes, (1,))
        self.add_timer(169.7, self.cosmetics_movement.ears_inwards, (1,))
        self.add_timer(172.7, self.cosmetics_movement.open_eyes, (1,))  # "tell your little sister"
        self.add_timer(172.7, self.cosmetics_movement.ears_facing_front, (1,))
        self.add_timer(172.7, self.joints_controller.move_yaw, (1, 0))
        self.add_timer(175.4, self.joints_movement.nod, (1.5, 2))  # "thank you"
        self.add_timer(176.6, self.cosmetics_movement.blink, (0.5,))  # "Pretty Princess"
        self.add_timer(179.6, self.cosmetics_movement.ear_outwards, (1,))  # "fabulous job"
        self.add_timer(179.6, self.joints_movement.nod, (2, 2))
        self.add_timer(182.6, self.cosmetics_movement.ears_facing_front, (1,))  # "Now, back to us"
        self.add_timer(182.6, self.cosmetics_movement.eyes_squint, (1,))
        #self.add_timer(182.6, self.led_controller.turn_on_led, ((255, 0, 255), 200))  # Magenta
        self.add_timer(184.6, self.joints_controller.move_pitch, (1, -15))  # "Could you PLEASE"
        self.add_timer(186.6, self.cosmetics_movement.blink, (0.5,))  # "PINK dinosaur,"
        #self.add_timer(187.6, self.cosmetics_movement.blink, (0.3,))  # "monster,"
        self.add_timer(188.4, self.cosmetics_movement.blink, (0.5,))  # "or cowboy?"
        self.add_timer(189.1, self.joints_movement.nod, (2, 2))  # "Goodness knows"
        self.add_timer(190.4, self.cosmetics_movement.blink, (0.5,))  # "splash of colour"
        self.add_timer(191.7, self.cosmetics_movement.eyes_squint, (1,))  # "Your unused friend"
        self.add_timer(191.7, self.joints_movement.shake, (1.5, 2))  
        self.add_timer(194, self.led_controller.turn_off_led, ())

        # Ending
        self.add_timer(195.2, self.cosmetics_movement.ear_outwards, (1,))  # "Well,"
        self.add_timer(195.2, self.led_controller.turn_on_led, ((250, 250, 250), 150))  # Neutral
        self.add_timer(195.6, self.joints_controller.move_all, (1, 0, 5, 30))  # "poor Duncan"
        self.add_timer(197.7, self.joints_movement.nod, (1.5, 2))  # "wanted his crayons"
        self.add_timer(197.7, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(201.5, self.joints_controller.move_all, (1, 0, -10, 15))  # "gave him an idea"
        self.add_timer(201.5, self.cosmetics_movement.blink, (0.3,))
        self.add_timer(201.8, self.cosmetics_movement.blink, (0.3,))
        self.add_timer(201.5, self.cosmetics_movement.ear_outwards, (1,))
        self.add_timer(203.7, self.joints_controller.move_yaw, (1, 30))  # "showed his teacher"
        self.add_timer(206.4, self.joints_controller.move_yaw, (1, 0))  # ""Good Work" sticker"
        self.add_timer(206.4, self.cosmetics_controller.wagging, (2, 15))
        self.add_timer(208.6, self.cosmetics_movement.blink, (0.5,))  # "all the colours"
        self.add_timer(208.6, self.joints_movement.nod, (1.5, 2))
        self.add_timer(206.4, self.led_controller.toggle_led_sections, 
                    (5, (255, 0, 0), (0, 255, 0), (0, 0, 255), 200))  # rainbow
        self.add_timer(211.5, self.cosmetics_movement.ears_inwards, (1,))  # "The end."
        play_thread.join()
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
        # Wait for all threads to finish (including the audio playback)
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)

    def book3(self): # how miro was built
        self.stop_flag = False
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing how miro was built")
        audiobook_file = 'mp3_files_slushy/audiobooks/book3-how_miro_was_built.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        
        # Schedule all movements with story lines as comments
        self.add_timer(0, self.joints_movement.nod, (2, 2))  # "Ok. I will be reading:"
        self.add_timer(2.3, self.cosmetics_movement.blink, (0.5,))  # "How MiRo Was Built."
        
        # "Once upon a time, in a lab filled with"
        self.add_timer(4.1, self.cosmetics_movement.ear_outwards, (1,))  
        self.add_timer(4.1, self.joints_controller.move_yaw, (1, -30))  
        self.add_timer(4.1, self.led_controller.turn_on_led, ((176, 224, 230), 200)) # Calm readiness - Powder Blue
        
        # "blinking lights and"
        self.add_timer(6.7, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(6.7, self.led_controller.turn_on_led, ((135, 206, 250), 200)) #  Intrigue - Light Sky Blue
        
        # "buzzing gadgets,"
        self.add_timer(7.8, self.cosmetics_movement.blink, (0.5,))  
        
        # "a team of clever engineers had a big dream:"
        self.add_timer(9.2, self.joints_controller.move_yaw, (1, 0))  
        self.add_timer(9.2, self.joints_controller.move_neck, (1, 15))  
        self.add_timer(9.2, self.led_controller.turn_on_led, ((230, 190, 138), 200))  # Inspiration - Light Gold
        
        # "to build a robot friend who"
        self.add_timer(12.3, self.joints_movement.nod, (2, 2))  
        
        # "could play, learn,"
        self.add_timer(14.3, self.cosmetics_movement.blink, (0.5,))  
        
        # "and wag its tail just like a real pet!"
        self.add_timer(16.3, self.cosmetics_controller.wagging, (2, 15))  
        self.add_timer(16.3, self.led_controller.turn_on_led, ((144, 238, 144), 200))  # Excitement - Light Green
        
        # "They started with the robot's big,"
        self.add_timer(18.8, self.cosmetics_movement.eyes_squint, (1,))  
        
        # "round eyes—"
        self.add_timer(20.9, self.cosmetics_movement.open_eyes, (1,))  
        self.add_timer(20.9, self.led_controller.turn_on_led, ((135, 206, 235), 200))  # Focus - Sky Blue
        
        # "but these"
        self.add_timer(22.5, self.cosmetics_movement.ears_inwards, (1,))  
        
        # "weren't ordinary eyes."
        self.add_timer(23.2, self.joints_movement.shake, (2, 2))  
        
        # "They were special cameras that"
        self.add_timer(25.2, self.joints_controller.move_yaw, (1, 30))  
        self.add_timer(25.2, self.led_controller.turn_on_led, ((112, 128, 144), 200))  # Suspense - Slate Gray
        
        # "helped the robot"
        self.add_timer(27, self.cosmetics_movement.eye_wink, (1, 1, "left"))  
        
        # "see the world, just like we do!"
        self.add_timer(28, self.joints_controller.move_yaw, (1, -30))  
        self.add_timer(28, self.joints_controller.move_neck, (1, 15))  
        
        # "Then, they gave the robot cute little ears"
        self.add_timer(30.4, self.cosmetics_movement.ears_facing_front, (1,))  
        self.add_timer(30.4, self.joints_controller.move_yaw, (1, 0))  
        self.add_timer(30.4, self.led_controller.turn_on_led, ((127, 255, 212), 200))  # Curiosity - Aquamarine
        
        # "which were actually microphones so it could hear"
        self.add_timer(33.5, self.joints_movement.nod, (2, 2))  
        self.add_timer(33.5, self.led_controller.turn_on_led, ((119, 221, 119), 200))  # Engagement - Pastel Green
        
        # "voices and"
        self.add_timer(36.3, self.cosmetics_controller.move_one_ear, (1, 1, "left"))  
        
        # "sounds."
        self.add_timer(37.3, self.cosmetics_controller.move_one_ear, (1, 1, "right"))  
        
        # "Next came the best part—"
        self.add_timer(38.2, self.cosmetics_movement.eyes_squint, (1,))  
        self.add_timer(38.2, self.joints_movement.nod, (2, 2))  
        
        # "the robot's brain!"
        self.add_timer(40.2, self.joints_controller.move_neck, (1, 15))  
        self.add_timer(40.2, self.cosmetics_movement.open_eyes, (1,))  
        self.add_timer(40.2, self.led_controller.turn_on_led, ((255, 215, 0), 200))  # Anticipation - Golden Yellow
        
        # "The engineers filled it with tiny computers"
        #self.add_timer(42, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(42, self.cosmetics_controller.wagging, (2, 15))  
        
        # "that made the robot smart. It could"
        self.add_timer(45, self.joints_movement.nod, (2, 2))  
        self.add_timer(45, self.led_controller.turn_on_led, ((70, 130, 180), 200))  # Cleverness - Steel Blue
        
        # "learn tricks,"
        self.add_timer(47.3, self.joints_movement.nod, (2, 2))  
        
        # "recognize faces,"
        self.add_timer(48.5, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(48.5, self.led_controller.turn_on_led, ((255, 127, 80), 200)) # Surprise - Coral 
        
        # "and even dance to music!"
        self.add_timer(49.7, self.joints_controller.move_neck, (1, 30))  
        self.add_timer(49.7, self.joints_controller.move_yaw, (1, -30))  
        
        # "But a robot friend needed a body too,"
        self.add_timer(52, self.cosmetics_movement.ears_inwards, (1,))  
        self.add_timer(52, self.cosmetics_movement.close_eyes, (1,))  
        self.add_timer(52, self.led_controller.turn_on_led, ((255, 165, 0), 200))  # Energy - Orange
        
        # "so they built the robot with wheels to zoom around"
        self.add_timer(55, self.cosmetics_movement.open_eyes, (1,))  
        self.add_timer(55, self.joints_controller.move_yaw, (0.5, 20))
        self.add_timer(55, self.joints_controller.move_neck, (1, 20))  
        
        # "and a waggy tail that wiggled when it was happy."
        self.add_timer(58.4, self.joints_controller.move_yaw, (0.5, -20))  
        self.add_timer(58.4, self.cosmetics_controller.wagging, (2, 15))  
        
        # "But the coolest part?"
        self.add_timer(61, self.joints_movement.nod, (2, 2))  
        self.add_timer(61, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(61, self.led_controller.turn_on_led, ((255, 188, 217), 200))  # Playfulness - Cotton Candy
        
        # "The robot could"
        self.add_timer(63.3, self.cosmetics_controller.wagging, (2, 30))  
        
        # "feel your touch!"
        self.add_timer(63.8, self.joints_movement.nod, (2, 2))  
        
        # "When you pet its head and body,"
        self.add_timer(66, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(67, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(66, self.cosmetics_movement.ear_outwards, (1,))  
        self.add_timer(66, self.led_controller.turn_on_led, ((255, 255, 0), 200))  # Happiness - Canary
        
        # "it wiggled with joy."
        self.add_timer(68, self.joints_movement.nod, (2, 2))  
        self.add_timer(68, self.cosmetics_controller.wagging, (2, 15))  
        
        # "After months of tinkering,"
        self.add_timer(70, self.cosmetics_movement.ears_inwards, (1,))  
        self.add_timer(70, self.joints_controller.move_neck, (1, 30))  
        self.add_timer(70, self.led_controller.turn_on_led, ((175, 238, 238), 200))  # Awe - Pale Turquoise
        
        # "testing,"
        self.add_timer(71.8, self.cosmetics_movement.blink, (0.5,))  
        
        # "and tickling,"
        self.add_timer(72.4, self.cosmetics_movement.eyes_squint, (1,))  
        
        # "yes, tickling!,"
        self.add_timer(73.4, self.joints_movement.nod, (2, 2))  
        
        # "the robot was finally ready to play!"
        self.add_timer(75, self.cosmetics_movement.open_eyes, (1,))  
        self.add_timer(76, self.joints_controller.move_yaw, (1, 20))  
        
        # "The engineer exclaimed,"
        self.add_timer(77.8, self.joints_controller.move_neck, (1, 15))  
        self.add_timer(77.8, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(77.8, self.led_controller.turn_on_led, ((240, 128, 128), 200))  # Warmth - Light Coral
        
        # ""What should we name him?""
        self.add_timer(79.6, self.cosmetics_movement.open_eyes, (1,))  
        self.add_timer(79.6, self.cosmetics_movement.ear_outwards, (1,))  
        
        # "The robot giggled and said,"
        self.add_timer(81.5, self.joints_controller.move_yaw, (1, -20))  
        self.add_timer(81.5, self.cosmetics_movement.ears_facing_front, (1,))  
        self.add_timer(81.5, self.led_controller.turn_on_led, ((135, 206, 235), 200))  # Contentment - Sky Blue
        
        # ""Call me MiRo!""
        self.add_timer(83.5, self.cosmetics_movement.eye_wink, (1, 1, "right"))  
        #self.add_timer(83.5, self.cosmetics_controller.wagging, (2, 15))  
        
        # The end.
        self.add_timer(85.5, self.cosmetics_movement.blink, (0.5,))  
        self.add_timer(85.5, self.cosmetics_movement.ears_inwards, (1,))  
        self.add_timer(85.5, self.joints_controller.move_yaw, (1, 0))  
        self.add_timer(85.5, self.led_controller.turn_off_led, ())  
        play_thread.join()
        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
        # Wait for all threads to finish (including the audio playback)
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)        

    def book4(self): # The clock that ran backwards
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        # Start playing the audiobook in the background
        print("[AUDIOBOOK] Playing The The clock that ran backwards")
        audiobook_file = 'mp3_files_slushy/audiobooks/book4-the_clock_that_ran_backwards.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        
        # Schedule all movements with story text as comments
        
        self.add_timer(5, self.led_controller.turn_off_led)
        self.add_timer(0, self.joints_movement.nod, (2, 2))# ok. I will be reading:
        self.add_timer(2.3, self.cosmetics_movement.blink, (0.5,))# The Clock That Ran Backwards. Max
        self.add_timer(5, self.led_controller.turn_on_led, ((0, 128, 128), 250))# Curiosity - Teal
        self.add_timer(5.2, self.cosmetics_movement.blink, (0.5,))# found the pocket watch
        self.add_timer(5.7, self.cosmetics_movement.blink, (0.5,))
        
        self.add_timer(6.5, self.joints_controller.move_neck, (1, 30))# on the third step of the library staircase.
        self.add_timer(9.7, self.led_controller.turn_on_led, ((75, 0, 130), 250))# Fascination – Indigo
        self.add_timer(9.7, self.cosmetics_movement.eyes_squint, (1,))# It was old, really old, with swirly
        self.add_timer(13, self.cosmetics_movement.ear_outwards, (1,))# engravings and a tiny dragon carved into the back.
        self.add_timer(14.5, self.joints_movement.nod, (1, 2))
        self.add_timer(16.7, self.joints_controller.move_neck, (1, 20))# He turned it over in his hand,
        self.add_timer(18.6, self.cosmetics_movement.open_eyes, (1,))# curious. The time was
        self.add_timer(19, self.led_controller.turn_on_led, ((128, 128, 128), 250))# Confusion – Gray
        self.add_timer(20.5, self.cosmetics_movement.ears_inwards, (3,))# wrong. It always said 3:14. He
        self.add_timer(24, self.led_controller.turn_on_led, ((255, 191, 0), 250))# Anticipation – Amber 
        self.add_timer(24.2, self.cosmetics_movement.blink, (0.5,))# clicked the top button. The watch made a
        
        self.add_timer(27.2, self.joints_movement.nod, (2, 2))# soft whirrr... and
        self.add_timer(30, self.joints_controller.move_yaw, (1, 30))# then the strangest thing happened.
        self.add_timer(28.8, self.cosmetics_movement.eyes_squint, (1,))
        self.add_timer(32.3, self.led_controller.turn_on_led, ((44, 117, 255), 250))# Shock – Electric Blue
        self.add_timer(32.3, self.cosmetics_controller.move_eyes, (1, 0))# His best friend Noah,
        self.add_timer(33.1, self.cosmetics_movement.ear_outwards, (1,))# who had just walked past him seconds ago, was
        self.add_timer(35, self.joints_controller.move_yaw, (1, -30))# back at the end of the hallway again.
        self.add_timer(37.4, self.led_controller.turn_on_led, ((173, 216, 230), 250))# Amazement – Light Blue
        self.add_timer(37.4, self.cosmetics_movement.blink, (0.5,))# Max blinked.
        
        self.add_timer(38.5, self.joints_controller.move_yaw, (1, 0))# Everything had... rewound?
        self.add_timer(38.5, self.joints_controller.move_neck, (1, 25))
        self.add_timer(41, self.led_controller.turn_on_led, ((0, 128, 128), 250))# Curiosity – Teal
        self.add_timer(41, self.joints_controller.move_pitch, (1, -15))# He tested it over the next few days.
        self.add_timer(41, self.cosmetics_movement.ears_inwards, (1,))
        #self.add_timer(41, self.cosmetics_movement.eyes_squint, (1,))
        
        self.add_timer(44.8, self.joints_movement.nod, (2, 2))# Every time he clicked the button, the watch
        self.add_timer(46, self.led_controller.turn_on_led, ((255, 165, 0), 250))# Excitement – Orange
        self.add_timer(46.5, self.cosmetics_movement.close_eyes, (1,))# sent him exactly five minutes into the past
        self.add_timer(49.7, self.cosmetics_movement.open_eyes, (1,))# but only once per day.
        self.add_timer(53.1, self.joints_movement.shake, (2, 3))# After that, it wouldn't work until the next morning.
        self.add_timer(55.5, self.cosmetics_movement.ear_outwards, (1,))# At first, he used it for silly stuff:
        self.add_timer(56.8, self.cosmetics_controller.wagging, (2, 10))
        
        self.add_timer(58.6, self.led_controller.turn_on_led, ((144, 238, 144), 250))# Playfulness – Light Green
        self.add_timer(58.6, self.joints_movement.nod, (2,2))# • Re-doing a test question
        self.add_timer(60.6, self.cosmetics_movement.blink, (0.5,))# he knew he got wrong.
        self.add_timer(62.5, self.led_controller.turn_on_led, ((255, 255, 0), 250))# Joy – Yellow
        self.add_timer(62.5, self.joints_movement.nod, (2,2))# • Catching a ball in gym class that
        self.add_timer(64.5, self.cosmetics_movement.blink, (0.5,))# he'd missed the first time.
        self.add_timer(62.5, self.led_controller.turn_on_led, ((255, 105, 180), 250))# Mischief – Hot Pink
        self.add_timer(66.3, self.joints_movement.nod, (2,2))# • Avoiding detention after
        self.add_timer(70, self.cosmetics_movement.eye_wink, (0.5, 1, "right"))# accidentally launching a spitball into Mrs. Dunley's hair (oops).
        self.add_timer(68, self.joints_controller.move_yaw, (1, -10))
        
        self.add_timer(72.8, self.led_controller.turn_on_led, ((47, 79, 79), 250))# Suspense – Dark Slate Gray
        self.add_timer(72.8, self.cosmetics_movement.ear_outwards, (1,)) # It was awesome.
        self.add_timer(74.4, self.cosmetics_movement.ears_inwards, (1,))# Until it wasn't.
        self.add_timer(76.4, self.led_controller.turn_on_led, ((255, 99, 71), 250))# Embarrassment – Tomato Red
        self.add_timer(76.4, self.joints_controller.move_pitch, (1, -15)) # One day, Noah tripped in the cafeteria
        self.add_timer(76.4, self.joints_controller.move_yaw, (1, 20))
        self.add_timer(79.3, self.joints_controller.move_neck, (1, 30))# and dropped his lunch tray.
        self.add_timer(79.3, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(81.2, self.joints_controller.move_pitch, (1, 5))# Spaghetti everywhere.
        self.add_timer(83.3, self.cosmetics_movement.close_eyes, (1,))# Everyone laughed.
        
        self.add_timer(84.8, self.led_controller.turn_on_led, ((128, 0, 0), 250))# Conflict – Maroon
        self.add_timer(84.8, self.cosmetics_controller.move_eyes, (1, 0.4))# Max hesitated.
        self.add_timer(86.4, self.joints_controller.move_neck, (1, 20))# He could rewind time and warn Noah.
        self.add_timer(86.4, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(90.1, self.led_controller.turn_on_led, ((176, 224, 230), 250))# Relief – Powder Blue 
        self.add_timer(90.1, self.cosmetics_movement.ear_outwards, (1,))# But then he saw Noah get up,
        self.add_timer(91.6, self.cosmetics_movement.blink, (0.5,))# smile, and make a
        self.add_timer(92.2, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(92.2, self.led_controller.turn_on_led, ((135, 206, 235), 250))# Admiration – Sky Blue
        #self.add_timer(92.8, self.cosmetics_movement.blink, (0.3,))
        self.add_timer(93, self.cosmetics_controller.wagging, (2, 10))# joke so funny the whole cafeteria roared.
        
        self.add_timer(96.8, self.led_controller.turn_on_led, ((188, 184, 138), 250))# Acceptance – Sage Green
        self.add_timer(96.8, self.joints_movement.shake, (2, 2))# Max didn't press the button.
        self.add_timer(96.8, self.cosmetics_movement.ears_inwards, (1,))
        self.add_timer(99.7, self.led_controller.turn_on_led, ((25, 25, 112), 250))# Maturity – Midnight Blue
        self.add_timer(99.7, self.joints_controller.move_pitch, (1, -15))# That's when he realized: sometimes mistakes
        self.add_timer(101.5, self.joints_movement.nod, (0.5, 1))# were okay.
        self.add_timer(103.3, self.led_controller.turn_on_led, ((32, 178, 170), 250))# Peace – Light Sea Green
        self.add_timer(103.3, self.cosmetics_movement.ear_outwards, (1,))# Sometimes they led to better moments.
        self.add_timer(104.7, self.cosmetics_movement.blink, (0.5,))
        self.add_timer(107.5, self.joints_movement.shake, (1, 1))# And maybe—just maybe he didn't need to fix everything.
        
        self.add_timer(109.8, self.led_controller.turn_on_led, ((70, 130, 180), 250))# Closure – Steel Blue
        self.add_timer(109.8, self.cosmetics_movement.ears_inwards, (1,))# The next morning, he took the
        self.add_timer(111.5, self.joints_controller.move_yaw, (1, 30))# watch back to the library staircase
        self.add_timer(114, self.joints_controller.move_neck, (1, 35))# and left it on the third step.
        self.add_timer(116.2, self.led_controller.turn_on_led, ((255, 255, 153), 250))# Hope – Pale Yellow
        self.add_timer(116.2, self.joints_controller.move_yaw, (1, 0))# Someone else would find it.
        self.add_timer(116.2, self.joints_controller.move_neck, (1, 20))
        self.add_timer(116.2, self.led_controller.turn_on_led, ((230, 230, 250), 250))# Reflection – Lavender
        self.add_timer(118.3, self.cosmetics_movement.blink, (0.5,))# Maybe they'd learn the same lesson.
        self.add_timer(120.6, self.led_controller.turn_on_led, ((255, 218, 185), 250))# Humor – Peach
        self.add_timer(120.6, self.cosmetics_movement.eyes_squint, (1,))# Or maybe... they'd just
        
        self.add_timer(122.2, self.joints_controller.move_neck, (1, 30))# use it to redo a math quiz.
        self.add_timer(124.6, self.led_controller.toggle_led_sections, (15, (255,0,0),(255,0,0),(255,0,0), 250))# Finality 
        self.add_timer(124.6, self.joints_controller.move_all, (1, 15, -15, 20)) # Either way, the clock was 
        self.add_timer(124.6, self.cosmetics_movement.open_eyes, (1,))
        self.add_timer(125.6, self.led_controller.toggle_led_sections, (15, (255,0,0),(0,255,0),(0,255,0), 250))# Finality 
        self.add_timer(126.1, self.cosmetics_movement.eye_wink, (0.5, 1, "right"))# ticking.
        self.add_timer(126.1, self.joints_controller.move_yaw, (1, -10))
        self.add_timer(126.6, self.led_controller.toggle_led_sections, (15, (255,0,0),(0,255,0),(0,0,255), 250))# Finality 
        # The end.

        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
        # Wait for all threads to finish (including the audio playback)
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)     

    def ending_line(self): 
        # Start the check_exit_flag thread
        self.parent_thread = threading.current_thread()
        exit_thread = threading.Thread(target=self.check_exit_flag)
        exit_thread.start()

        
        print("[AUDIOBOOK] Ending line")
        audiobook_file = 'mp3_files_slushy/audiobooks/story_end.mp3'
        play_thread = threading.Thread(target=self.audio_player.play_audio, args=(audiobook_file,))
        play_thread.start()
        play_thread.join()

        # Monitor play_audio thread
        while play_thread.is_alive():
            if self.stop_flag:
                print("[AUDIOBOOK] Stopping book1...")
                for timer in self.timers:
                    timer.cancel()  # Cancel scheduled movements
                break
            time.sleep(0.1)
        # Wait for all threads to finish (including the audio playback)
        # Turn LEDs orange to indicate a transition period
        self.current_color = (255, 165, 0)  # Orange
        self.led_controller.turn_on_led(self.current_color, 250)  


    ##Exit###


    def check_exit_flag(self):
        while not self.stop_flag:
            # Detect aruco markers
            self.aruco_detect.tick_camera()
            
            try:
                self.remote_data = receive_data()
            except Exception as e:
                print(f"Failed to send data: {e}")

            if self.aruco_detect.exit_behaviour or self.remote_data[4]==2:
                self.stop_flag = True
                self.audio_player.stop()
                exit_behaviour_thread = threading.Thread(target=self.audio_player.play_audio, args=('mp3_files_slushy/i_will_stop.mp3',))
                exit_behaviour_thread.start()
                exit_behaviour_thread.join()
                print("[AUDIOBOOK] Exit behaviour detected, stopping relaxation exercise.")
                
                # Turn LEDs orange to indicate a transition period
                self.current_color = (255, 165, 0)  # Orange
                self.led_controller.turn_on_led(self.current_color, 250)
            
            elif self.aruco_detect.rupelstiltskin or self.remote_data[4]==3:
                self.stop_flag = True
                self.audio_player.stop()
                self.book3()
            elif self.aruco_detect.emperor or self.remote_data[4]==4:
                self.stop_flag = True
                self.audio_player.stop()
                self.book4()
            elif self.aruco_detect.frog or self.remote_data[4]==5:
                self.stop_flag = True
                self.audio_player.stop()
                self.book4()

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

        

