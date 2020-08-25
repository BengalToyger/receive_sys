#!/usr/bin/python3

"""
Wiring Check, Pi Radio w/RFM69

Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the RFM69 radio module.
import adafruit_rfm69
import struct
from guizero import App, ButtonGroup, Text, TextBox, PushButton, Box
import threading
import numpy as np
import beam_calcs
'''
Data Packet Structure
struct Data_Packet {
       float latitude_degrees;
       float longitude_degrees;
       int16_t mag_x_u_T;
       int16_t mag_y_u_T;
       int16_t mag_z_u_T;
};
'''
# Data format string used with python struct library to unpack/pack data
data_format = 'ffhhh'

'''
Command packet structure
struct Cmd_Packet {
    uint8_t opcode;
    uint8_t ps_1;
    uint8_t ps_2;
    uint8_t ps_3;
    uint8_t ps_4;
};
'''	

cmd_format = 'BBBBB'

# RFM69 Configuration
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize the radio
try:
    rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0)
    print('RFM69: Online')
except RuntimeError as error:
    # Thrown on version mismatch
    print('RFM69 Error: ', error)

# Global Vars
rfm69.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'

# Node value for ground/pi side radio
rfm69.node = 1
# Node value for the avionics side radio
rfm69.destination = 2
unpacked = None
receive_counter = 0
send_flag = 0

# Max value in degrees that the phase shifter can be shifted
MAX_PS_DEG_VAL = 353
# Phase shifter takes a 6 bit input, thus max value is 63
MAX_PS_BIT_VAL = 63
# With a max phase shift of 353 and 6 bits available, the precision is about
# 5.6 degrees

# Based on the schematic sent to me, the shift register pins do not line up
# LSB to MSB/Lowest degree shift to highest degree shift. This bit map 
# translates the numerical representation of the degree value to the way
# the pins are actually connected. Eg we have a six bit number, bit 0 gets 
# mapped to bit 2 on the shift register, bit 1 to bit 6, etc
# Bit 0 of the shifted value corresponds to Q_H or pin 7 on the shift register
# Bit 6 of the shifted value corresponds to Q_B or pin 1 on the shift register
PS_BIT_MAP = [2,6,3,4,0,1]
# The ref bit is bit 5 and is always set. It is Q_C or pin 2 on the shift reg
PS_REF_BIT = 5

# Antenna and frequency data
freq = 10*10**9
v_p = 3*10**8
wave_l = v_p/freq
k = 1/wave_l
d_e = wave_l/2 # Distance between elements
d_div_wave_l = d_e/wave_l


# Print data to the terminal
def print_data():
    print('RSSI:',rfm69.rssi)
    print('Latitude:',unpacked[0])
    print('Longitue:',unpacked[1])
    print('Mag_x:',unpacked[2])
    print('Mag_y:',unpacked[3])
    print('Mag_z:',unpacked[4])

# See if there is data to receive, and if there is, unpack it
# Sends command packet if the send_flag is set
# Prints error if struct is unpackable
def send_receive_data():
    global receive_counter, unpacked, rfm69, send_flag
    while True:    
        packet = None
        packet = rfm69.receive()
        if packet is not None:
            receive_counter = receive_counter + 1
            try:
                unpacked = struct.unpack(data_format,packet)
            except:
                print("Error: Unable to unpack struct\n")
            latitude.value = str(unpacked[0])
            longitude.value = str(unpacked[1])
            rssi.value = str(rfm69.rssi)
            number_received.value = str(receive_counter)
        if send_flag:
            send_flag = 0
            send_com()
            
# Check and see if a value is a float. Fail without stopping program
def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

# Checks to make sure the opcode is valid
def is_valid_opcode(opcode):
    if opcode.isdigit():
        opcode = int(opcode)
        if not (opcode <= 2 and opcode >= 0):
            error_label.value = "Please enter a valid opcode between 0 and 2"
            return 0
        else:
            return 1
    else: 
        error_label.value = "Please enter a valid opcode between 0 and 2"
        return 0

# Checks input for phase shifter values. If the phase shifter value isn't a float,
# it sets the value to zero. If the float is out of range, it sets an error message
# and returns zero
def is_valid_ps_vals(ps_1, ps_2, ps_3, ps_4):
    if isfloat(ps_1) and isfloat(ps_2) and isfloat(ps_3)\
            and isfloat(ps_4):
        ps_1 = float(ps_1)
        ps_2 = float(ps_2)
        ps_3 = float(ps_3)
        ps_4 = float(ps_4)
    else:
        error_label.value = "Phase Shifter Input Value Error \
Please Enter a Value Between 0 and 353" 
        return 0
    if ps_1 > MAX_PS_DEG_VAL or ps_1 < 0 \
        or ps_2 > MAX_PS_DEG_VAL or ps_2 < 0 \
        or ps_3 > MAX_PS_DEG_VAL or ps_3 < 0 \
        or ps_4 > MAX_PS_DEG_VAL or ps_4 < 0:
        error_label.value = "Phase Shifter Input Value Error \
Please Enter a Value Between 0 and 353"
        return 0
    else:
        return 1

# Rounds the phase shifter input values to the nearest 5.6 degrees
def round_ps_vals(ps_1, ps_2, ps_3, ps_4):
    ps_1 = int(round(ps_1/MAX_PS_DEG_VAL*MAX_PS_BIT_VAL))
    ps_2 = int(round(ps_2/MAX_PS_DEG_VAL*MAX_PS_BIT_VAL))
    ps_3 = int(round(ps_3/MAX_PS_DEG_VAL*MAX_PS_BIT_VAL))
    ps_4 = int(round(ps_4/MAX_PS_DEG_VAL*MAX_PS_BIT_VAL))
    
    return [ps_1, ps_2, ps_3, ps_4]


# Maps the phase shifter bit values to the correct locations for the shift registers
def map_ps_bits(ps_1, ps_2, ps_3, ps_4):
    ps_1_fix = 0
    ps_2_fix = 0
    ps_3_fix = 0
    ps_4_fix = 0

    # Iterating through the bit map, set the correct bits for each phase shifter
    for i in range(6):
        ps_1_fix = ps_1_fix|((((1 << i) & (ps_1) and 1)\
                <<PS_BIT_MAP[i])&(1<<PS_BIT_MAP[i]))
        ps_2_fix = ps_2_fix|((((1 << i) & (ps_2) and 1)\
                <<PS_BIT_MAP[i])&(1<<PS_BIT_MAP[i]))
        ps_3_fix = ps_3_fix|((((1 << i) & (ps_3) and 1)\
                <<PS_BIT_MAP[i])&(1<<PS_BIT_MAP[i]))
        ps_4_fix = ps_4_fix|((((1 << i) & (ps_4) and 1)\
                <<PS_BIT_MAP[i])&(1<<PS_BIT_MAP[i]))

    ps_1_fix = ps_1_fix|(1<<PS_REF_BIT)
    ps_2_fix = ps_2_fix|(1<<PS_REF_BIT)
    ps_3_fix = ps_3_fix|(1<<PS_REF_BIT)
    ps_4_fix = ps_4_fix|(1<<PS_REF_BIT)

    return [ps_1_fix, ps_2_fix, ps_3_fix, ps_4_fix]

# Formats and sends the command packet
def send_com():
    opcode = opcode_entr.value
    if mode_entr.value == "0":   
        ps_1 = ps_1_entr.value
        ps_2 = ps_2_entr.value
        ps_3 = ps_3_entr.value
        ps_4 = ps_4_entr.value
        ps_1_calc_val.value = "N/A"
        ps_3_calc_val.value = "N/A"
    elif mode_entr.value == "1":
        ps_1 = beam_calcs.phase_shift_calc(d_div_wave_l,
            int(beam_angle_yz_entr.value))
        ps_2 = 0
        ps_3 = beam_calcs.phase_shift_calc(d_div_wave_l,
            int(beam_angle_yz_entr.value))
        ps_4 = 0
        ps_1 = beam_calcs.wrap_phase_deg(ps_1)
        ps_3 = beam_calcs.wrap_phase_deg(ps_3)
        ps_1_calc_val.value = str(ps_1)
        ps_3_calc_val.value = str(ps_3)
    else:
        return
    if is_valid_opcode(opcode):
        opcode = int(opcode)
    else:
        return
    if is_valid_ps_vals(ps_1, ps_2, ps_3, ps_4):
        ps_1 = float(ps_1)
        ps_2 = float(ps_2)
        ps_3 = float(ps_3)
        ps_4 = float(ps_4)
    else:
        return
    [ps_1, ps_2, ps_3, ps_4] = round_ps_vals(ps_1, ps_2, ps_3, ps_4) 
    [ps_1_fix, ps_2_fix, ps_3_fix, ps_4_fix] = map_ps_bits(ps_1, ps_2, ps_3, ps_4)
    reply = struct.pack(cmd_format,opcode,ps_1_fix,ps_2_fix,ps_3_fix,ps_4_fix)
    rfm69.send(reply,destination=2)
    error_label.value = ""

# Sets the send flag so that send_com() is called in the send_receive_data thread
def set_send_flag():
    global send_flag
    send_flag = 1

print(round_ps_vals(353,356,359,360))


# Set up GUI components
# Main app
app = App(title="Receive System", height=300, width=650, layout="grid")

# Status Box and components
status_box = Box(app, grid=[0, 0])
status_label = Text(status_box, text="Status", size=20)
rssi_label = Text(status_box, text="Radio Received Signal Strength")
rssi = Text(status_box, text="No data")
receive_counter_label = Text(status_box, text="Number of Received Packets")
number_received = Text(status_box, text="No data")
latitude_label = Text(status_box, text="Latitude (Decimal Degrees)")
latitude = Text(status_box, text="No data")
longitude_label = Text(status_box, text="Longitude (Decimal Degrees)")
longitude = Text(status_box, text="No data")
ps_1_calc_label = Text(status_box, text="Calculated Phase Shifter 1 Val")
ps_1_calc_val = Text(status_box, text="N/A")
ps_3_calc_label = Text(status_box, text="Calculated Phase Shifter 3 Val")
ps_3_calc_val = Text(status_box, text="N/A")

# Control Box
control_box = Box(app, grid=[1, 0])
control_label = Text(control_box, text="Control", size=20)
sub_control_box = Box(control_box, layout="grid")
# Modes Box
modes_box = Box(sub_control_box, grid=[0, 0])
mode_label = Text(modes_box, text="Mode Select")
mode_entr = ButtonGroup(modes_box, options=[["Set Individually", "0"], 
                                        ["Main Beam Angle in YZ Plane from -Z", "1"]]
                                        )
opcode_label = Text(modes_box, text="Command to Send (Value from 0 to 2)")
opcode_entr = ButtonGroup(modes_box, options=[["No Op", "0"],
                                        ["Blink", "1"],
                                        ["Set Phase Shifters", "2"]
                                        ], selected="0")
error_label = Text(modes_box, text="", color="red")

send_button = PushButton(modes_box, command=set_send_flag, text="Send Command")
# Values Box
values_box = Box(sub_control_box, grid=[1, 0])
ps_1_label = Text(values_box, text="Phase Shifter 1 Value (Degrees)")
ps_1_entr = TextBox(values_box, text="0")
ps_2_label = Text(values_box, text="Phase Shifter 2 Value (Degrees)")
ps_2_entr = TextBox(values_box, text="0")
ps_3_label = Text(values_box, text="Phase Shifter 3 Value (Degrees)")
ps_3_entr = TextBox(values_box, text="0")
ps_4_label = Text(values_box, text="Phase Shifter 4 Value (Degrees)")
ps_4_entr = TextBox(values_box, text="0")
beam_angle_yz_label = Text(values_box, text="Main Beam Angle in YZ Plane from -Z")
beam_angle_yz_entr = TextBox(values_box, text="0")

# Set up and start a thread to receive data
thread = threading.Thread(name="send_receive_data",target=send_receive_data,daemon=True)
thread.start()

#Display GUI
app.display()
