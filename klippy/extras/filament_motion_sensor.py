# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor

CHECK_RUNOUT_TIMEOUT = .250

class EncoderSensor:
    def __init__(self, config):
        # Read config
        self.printer = config.get_printer()
        switch_pin = config.get('switch_pin')
        self.extruder_name = config.get('extruder')
        self.detection_length = config.getfloat(
                'detection_length', 7., above=0.)
        self.use_irq = config.getboolean('use_irq', False)
        logging.info(f"motion use_irq?:{self.use_irq}")
        self.debounce_us = 0
        # Configure pins
        if self.use_irq:
            logging.info(f"motion use_irq1?:{self.use_irq}")
            self.debounce_us = config.getint('debounce_us', 0)          
            irq_button = config.printer.load_object(config, "buttons_irq")
            #irq_button.register_buttons([switch_pin], callback=self.encoder_event, self.debounce_us)
            irq_button.register_buttons([switch_pin], callback=self.encoder_event, debounce_us = self.debounce_us)
        else:
            logging.info(f"motion use_irq2?:{self.use_irq}")
            buttons = self.printer.load_object(config, 'buttons')
            buttons.register_buttons([switch_pin], self.encoder_event)

        # Get printer objects
        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.get_status = self.runout_helper.get_status
        self.extruder = None
        self.estimated_print_time = None
        # Initialise internal state
        self.filament_runout_pos = None
        self.button_state = 0
        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready',
                self._handle_ready)
        self.printer.register_event_handler('idle_timeout:printing',
                self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                self._handle_not_printing)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'CLEAR_MOTION_DATA',
            self.cmd_CLEAR_MOTION_DATA
        )
        self.motion_name = config.get_name().split()[1]
        self.gcode.register_mux_command('SET_MOTION_DETECTION', "NAME", self.motion_name, self.cmd_SET_MOTION_DETECTION)
        self.gcode.register_mux_command('GET_MOTION_DETECTION', "NAME", self.motion_name, self.cmd_GET_MOTION_DETECTION)
    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = (
                self._get_extruder_pos(eventtime) +
                self.detection_length)
    def cmd_SET_MOTION_DETECTION(self, gcmd):
        d_l = gcmd.get_float('LENGTH', 0.)
        if d_l <= 0:
            self.gcode.respond_info("The LENGTH value is error")
        else:
            self.detection_length = d_l
            self._update_filament_runout_pos()
            self.gcode.respond_info("the detection_length has changed successsful. The will restart after next restart.")
    
    def cmd_GET_MOTION_DETECTION(self, gcmd):
        self.gcode.respond_info("the detection_length is: " + str(self.detection_length))

    def cmd_CLEAR_MOTION_DATA(self, gcmd):
        self._update_filament_runout_pos()
    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = (
                self.printer.lookup_object('mcu').estimated_print_time)
        self._update_filament_runout_pos()
        self._extruder_pos_update_timer = self.reactor.register_timer(
                self._extruder_pos_update_event)
    def _handle_printing(self, print_time):
        self.reactor.update_timer(self._extruder_pos_update_timer,
                self.reactor.NOW)
    def _handle_not_printing(self, print_time):
        self.reactor.update_timer(self._extruder_pos_update_timer,
                self.reactor.NEVER)
    def _get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)
    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self._get_extruder_pos(eventtime)
        # Check for filament runout
        self.runout_helper.note_filament_present(
                extruder_pos < self.filament_runout_pos)
        return eventtime + CHECK_RUNOUT_TIMEOUT
    def encoder_event(self, eventtime, state):
        #self.gcode.respond_info(f"motion state: {state} ")
        if self.extruder is not None:
            self._update_filament_runout_pos(eventtime)
            self.button_state = state
            # Check for filament insertion
            # Filament is always assumed to be present on an encoder event
            self.runout_helper.note_filament_present(True)
            

def load_config_prefix(config):
    return EncoderSensor(config)
