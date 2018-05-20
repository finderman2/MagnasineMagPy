#!/usr/bin/env python

'''
MagPy - an application to interface with Magnum inverters
The application decodes the MagNet data protocol to display
data for all devices connected and active.

This program was initially inspired by Chris (aka user cpfl) Midnightsolar forum.

Created     13 Jun 2015
Modified    26 Feb 2018

@author: Paul Alting van Geusau
@author: Liam O'Brien

---------------------------------------------------------------------------------------------------------------------------
    Data capture from actual MagNet with MS4448PAE, RTR, AGS and BMK devices:
                                                                                                                    Bytes  Total   Packet Marker
    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FF                            22
    Remote_B+A0+A1      00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 17 26 14 00 73 00 A0 A1 02 35 FC 00 7D             27    = 49    pos[41] = 0xA0

    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FE                            22
    Remote_B+A1+A2      00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 20 20 90 3C 3C 78 A1 A2 01 00 00 00 00             27    = 49    pos[41] = 0xA1

    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FF                            22
    Remote_B+A2+RTR     00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 00 64 00 3C 0A 3C A2 91 01                         23    = 45    pos[41] = 0xA2

    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FF                            22
    Remote_B+A3         00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 50 28 00 20 0A 00 A3                               21    = 43    pos[41] = 0xA3

    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FE                            22
    Remote_B+A4         00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 3C 3C 00 00 00 00 A4                               21    = 43    pos[41] = 0xA4

    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FF                            22
    Remote_B+Z0         00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 17 26 00 00 00 00 00                               21    = 43    pos[41] = 0x00

    Inverter            40 00 01 F8 00 04 77 00 01 00 33 17 2E 22 73 01 00 01 02 58 00 FF                                                       22
    Remote_B+80+81 00 05 50 94 64 1E 16 08 00 D5 9B 84 07 19 17 26 00 00 28 00 80 81 61 13 AF FF D2 12 4C 18 BB FF E9 FF FF 00 5E 0A 01 39 = 61 pos[42] = 0x80 
---------------------------------------------------------------------------------------------------------------------------------------------
    Byte position       01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39
    Buffer position     00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38

'''
import serial
import time
import array
import argparse

import paho.mqtt.client as mqtt #import the client1

broker_address="localhost"
client = mqtt.Client("magnum") #create new instance
client.username_pw_set("emonpi", password="emonpimqtt2016")

try:
    client.connect(broker_address, port=1883) #connect to broker
except:
    print("Broker Connection Failed")

def safeDiv(x,y):
    try:
        return x/y
    except ZeroDivisionError:
        return 0

class inverter_proto():
    """definition of inverter packet """
#    def __init__(self):
    status_descript = "NA"      #       status descriptive word:
    fault_descript = "NA"       #       fault descriptive word:
    model_descript = "NA"
    status_code = '0x00'        # 0     packet_buffer[0]
    fault_code = '0x00'         # 1     packet_buffer[1]
    volts_dc = 0.0              # 2     packet_buffer[2] HIGH BYTE
                                # 3     packet_buffer[3] LOW BYTE
    amps_dc = 0.0               # 4     packet_buffer[4] HIGH BYTE
                                # 5     packet_buffer[5] LOW BYTE
    volts_ac_out = 0            # 6     packet_buffer[6]
    volts_ac_in = 0             # 7     packet_buffer[7]
    LED_inverter = 0            # 8     packet_buffer[8]
    LED_charger = 0             # 9     packet_buffer[9]
    revision = 0.0              # 10    packet_buffer[10]
    temp_battery = 0            # 11    packet_buffer[11]
    temp_transformer = 0        # 12    packet_buffer[12]
    temp_FET = 0                # 13    packet_buffer[13]
    model_id = 0                # 14    packet_buffer[14]
    stack_mode = "NA"           # 15    packet_buffer[15]
    amps_ac_in = 0              # 16    packet_buffer[16]
    amps_ac_out = 0             # 17    packet_buffer[17]
    frequency_ac_out = 0.0      # 18    packet_buffer[18] HIGH BYTE
                                # 19    packet_buffer[19] LOW BYTE
#    ALWAYS 0                   # 20    packet_buffer[20]
#    ALWAYS 0xFF                # 21    packet_buffer[21]
#    END
#
#    inverter decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.status_code = hex(ord(packet_buffer[0]))
        if (self.status_code == '0x0') :
            self.status_descript = "Charger Standby"

        if (self.status_code == '0x1') :
            self.status_descript = "Equalizing"

        if (self.status_code == '0x2') :
            self.status_descript = "Float Charging"

        if (self.status_code == '0x4') :
            self.status_descript = "Absorb Charging"

        if (self.status_code == '0x8') :
            self.status_descript = "Bulk Charging"

        if (self.status_code == '0x9') :
            self.status_descript = "Battery Saver"

        if (self.status_code == '0x10') :
            self.status_descript = "Charge"

        if (self.status_code == '0x20') :
            self.status_descript = "Off"

        if (self.status_code == '0x40') :
            self.status_descript = "Inverting"

        if (self.status_code == '0x50') :
            self.status_descript = "Standby"

        if (self.status_code == '0x60') :
            self.status_descript = "Searching"

        self.fault_code = hex(ord(packet_buffer[1]))
        if (self.fault_code == '0x0') :
            self.fault_descript = "No Faults"

        if (self.fault_code == '0x1') :
            self.fault_descript = "Stuck Relay"
    
        if (self.fault_code == '0x2') :
            self.fault_descript = "DC Overload"
    
        if (self.fault_code == '0x3') :
            self.fault_descript = "AC Overload"
    
        if (self.fault_code == '0x4') :
            self.fault_descript = "Dead Battery"
    
        if (self.fault_code == '0x5') :
            self.fault_descript = "AC Backfeed"
    
        if (self.fault_code == '0x8') :
            self.fault_descript = "Low Battery Cutout"
    
        if (self.fault_code == '0x9') :
            self.fault_descript = "High Battery Cutout"
    
        if (self.fault_code == '0xA') :
            self.fault_descript = "High AC Output Volts"
    
        if (self.fault_code == '0x10') :
            self.fault_descript = "Bad FET Bridge"
    
        if (self.fault_code == '0x12') :
            self.fault_descript = "FETs Over Temperature"
    
        if (self.fault_code == '0x13') :
            self.fault_descript = "FETs Over Temperature Quick"
    
        if (self.fault_code == '0x14') :
            self.fault_descript = "Internal Fault #4"
    
        if (self.fault_code == '0x16') :
            self.fault_descript = "Stacker Mode Fault"
    
        if (self.fault_code == '0x18') :
            self.fault_descript = "Stacker Sync Clock Out of Phase"
    
        if (self.fault_code == '0x17') :
            self.fault_descript = "Stacker Sync Clock Lost"
    
        if (self.fault_code == '0x19') :
            self.fault_descript = "Stacker AC Phase Fault"
    
        if (self.fault_code == '0x20') :
            self.fault_descript = "Over temperature Shutdown"
    
        if (self.fault_code == '0x21') :
            self.fault_descript = "Transfer Relay Fault"
    
        if (self.fault_code == '0x80') :
            self.fault_descript = "Charger Fault"
    
        if (self.fault_code == '0x81') :
            self.fault_descript = "Battery Temperature High"
    
        if (self.fault_code == '0x90') :
            self.fault_descript = "Transformer Temperature Cutout Open"
    
        if (self.fault_code == '0x91') :
            self.fault_descript = "AC Breaker CB3 Tripped"
    
        self.volts_dc = ((ord(packet_buffer[2]) * 256) + ord(packet_buffer[3])) / 10.0
        self.amps_dc = ((ord(packet_buffer[4]) * 256) + ord(packet_buffer[5]))
        self.volts_ac_out = ord(packet_buffer[6])
        self.volts_ac_in = ord(packet_buffer[7])
        self.revision = ord(packet_buffer[10]) / 10.0
        self.temp_battery = ord(packet_buffer[11])
        self.temp_transformer = ord(packet_buffer[12])
        self.temp_FET = ord(packet_buffer[13])
        self.model_id = ord(packet_buffer[14])
        if (self.model_id < 53):
            system_bus_volts = 12
            
        if (self.model_id > 47):
            system_bus_volts = 24
            
        if (self.model_id > 107):
            system_bus_volts = 48

        if (self.model_id == 6) :
            self.model_descript = "MM612"
        elif (self.model_id == 7) :
            self.model_descript = "MM612-AE"
        elif (self.model_id == 8) :
            self.model_descript = "MM1212" 
        elif (self.model_id == 9) :
            self.model_descript = "MMS1012" 
        elif (self.model_id == 10) :
            self.model_descript = "MM1012E" 
        elif (self.model_id == 11) :
            self.model_descript = "MM1512"
        elif (self.model_id == 15) :
            self.model_descript = "ME1512"
        elif (self.model_id == 20) :
            self.model_descript = "ME2012"
        elif (self.model_id == 25) :
            self.model_descript = "ME2512"
        elif (self.model_id == 30) :
            self.model_descript = "ME3112"
        elif (self.model_id == 35) :
            self.model_descript = "MS2012"
        elif (self.model_id == 40) :
            self.model_descript = "MS2012E"
        elif (self.model_id == 45) :
            self.model_descript = "MS2812"
        elif (self.model_id == 47) :
            self.model_descript = "MS2712E"
        elif (self.model_id == 53) :
            self.model_descript = "MM1324E"
        elif (self.model_id == 54) :
            self.model_descript = "MM1524"
        elif (self.model_id == 55) :
            self.model_descript = "RD1824"
        elif (self.model_id == 59) :
            self.model_descript = "RD2624E"
        elif (self.model_id == 63) :
            self.model_descript = "RD2824"
        elif (self.model_id == 69) :
            self.model_descript = "RD4024E"
        elif (self.model_id == 74) :
            self.model_descript = "RD3924"
        elif (self.model_id == 90) :
            self.model_descript = "MS4124E"
        elif (self.model_id == 91) :
            self.model_descript = "MS2024"
        elif (self.model_id == 105) :
            self.model_descript = "MS4024"
        elif (self.model_id == 106) :
            self.model_descript = "MS4024AE"
        elif (self.model_id == 107) :
            self.model_descript = "MS4024PAE"
        elif (self.model_id == 111) :
            self.model_descript = "MS4448AE"
        elif (self.model_id == 112) :
            self.model_descript = "MS3748AEJ"
        elif (self.model_id == 115) :
            self.model_descript = "MS4448PAE"
        elif (self.model_id == 116) :
            self.model_descript = "MS3748PAEJ"
        else :
            self.model_descript = "UNKNOWN"
        
        self.stack_mode_code = hex(ord(packet_buffer[15]))
        if (self.stack_mode_code == '0x0') :
            self.stack_mode = "Standalone Unit"

        if (self.stack_mode_code == '0x1') :
            self.stack_mode = "Master in Parallel Stack"

        if (self.stack_mode_code == '0x2') :
            self.stack_mode = "Slave in Parallel Stack"

        if (self.stack_mode_code == '0x4') :
            self.stack_mode = "Master in Series Stack"

        if (self.stack_mode_code == '0x8') :
            self.stack_mode = "Slave in Series Stack"
            
        self.amps_ac_in = ord(packet_buffer[16])
        self.amps_ac_out = ord(packet_buffer[17])
        self.frequency_ac_out = ((ord(packet_buffer[18]) * 256) + ord(packet_buffer[19])) / 10.0

        
class remote_base_proto():
    """definition of remote packet """
#    def __init__(self):
    status_code = '0x00'        # 00    ord(packet_buffer[22])
    search_watts = 0.0          # 01    ord(packet_buffer[23])
    battery_size = 0.0          # 02    ord(packet_buffer[24])
    battery_type = 0.0          # 03    ord(packet_buffer[25])
    charger_amps = 0.0          # 04    ord(packet_buffer[26])
    shore_ac_amps = 0.0         # 05    ord(packet_buffer[27])
    revision = 0.0              # 06    ord(packet_buffer[28])
    parallel_threshold = 0      # 07    ord(packet_buffer[29]) LOW NIBBLE
    force_charge = '0x00'       # 07    ord(packet_buffer[29]) HIGH NIBBLE
    genstart_auto = '0x00'      # 08    ord(packet_buffer[30])
    battery_low_trip = 0.0      # 09    ord(packet_buffer[31])
    volts_ac_trip = 0           # 10    ord(packet_buffer[32])
    float_volts = 0.0           # 11    ord(packet_buffer[33])
    equalise_volts = 0.0        # 12    ord(packet_buffer[34])
    absorb_time = 0.0           # 13    ord(packet_buffer[35]
    absorb_volts = 14.4
    status_descript = "NA"
    battery_type_descript = "NA"
    force_charge_descript = "NA"
    genstart_auto_descript = "NA"
#    END
#
#    remote_base decode function:
    def decode(self, packet_buffer):
        global system_bus_volts
        if (system_bus_volts == 24):
            self.absorb_volts *= 2

        if (system_bus_volts == 48):
            self.absorb_volts *= 4

        self.status_code = ord(packet_buffer[22])
        if (self.status_code == '0x0') :
            self.status_descript = "Remote Command Clear"

        self.search_watts = ord(packet_buffer[23])
        self.battery_size = ord(packet_buffer[24]) * 10
        self.battery_type = ord(packet_buffer[25])
        if (self.battery_type == 2):
            self.battery_type_descript = "Gel"

        if (self.battery_type == 4):
            self.battery_type_descript = "Flooded"

        if (self.battery_type == 8):
            self.battery_type_descript = "AGM"

        if (self.battery_type == 10):
            self.battery_type_descript = "AGM2"

        if (self.battery_type > 100):
            self.battery_type_descript = "Custom"
            if (system_bus_volts == 12):
                self.absorb_volts = self.battery_type / 10.0
                
            if (system_bus_volts == 24):
                self.absorb_volts = self.battery_type * 2 / 10.0

            if (system_bus_volts == 48):
                self.absorb_volts = self.battery_type * 4 / 10.0

        self.charger_amps = ord(packet_buffer[26])  # % of charger capacity ?
        self.shore_ac_amps = ord(packet_buffer[27])
        self.revision = ord(packet_buffer[28]) / 10.0
        self.parallel_threshold = (ord(packet_buffer[29]) & 0x0f) * 10
        self.force_charge = hex(ord(packet_buffer[29]) & 0xf0)
        if (self.force_charge == '0x10'):
            self.force_charge_descript = "Disable Refloat"

        if (self.force_charge == '0x20'):
            self.force_charge_descript = "Force Silent"

        if (self.force_charge == '0x40'):
            self.force_charge_descript = "Force Float"

        if (self.force_charge == '0x80'):
            self.force_charge_descript = "Force Bulk"

        self.genstart_auto = ord(packet_buffer[30])
        if (self.genstart_auto == 0):
            self.genstart_auto_descript = "Off"

        if (self.genstart_auto == 1):
            self.genstart_auto_descript = "Enable"

        if (self.genstart_auto == 2):
            self.genstart_auto_descript = "Test"

        if (self.genstart_auto == 4):
            self.genstart_auto_descript = "Enable with Quiet Time"

        if (self.genstart_auto == 5):
            self.genstart_auto_descript = "On"

        if (system_bus_volts == 12):
            self.battery_low_trip = ord(packet_buffer[31]) / 10.0

        if (system_bus_volts == 24):
            self.battery_low_trip = ord(packet_buffer[31]) / 10.0

        if (system_bus_volts == 48):
            self.battery_low_trip = ord(packet_buffer[31]) * 2 / 10.0

        self.volts_ac_trip = ord(packet_buffer[32])
        if (self.volts_ac_trip == 110):
            self.volts_ac_trip = 60

        if (self.volts_ac_trip == 122):
            self.volts_ac_trip = 65

        if (self.volts_ac_trip == 135):
            self.volts_ac_trip = 70

        if (self.volts_ac_trip == 145):
            self.volts_ac_trip = 75

        if (self.volts_ac_trip == 155):
            self.volts_ac_trip = 80

        if (self.volts_ac_trip == 165):
            self.volts_ac_trip = 85

        if (self.volts_ac_trip == 175):
            self.volts_ac_trip = 90

        if (self.volts_ac_trip == 182):
            self.volts_ac_trip = 95

        if (self.volts_ac_trip == 190):
            self.volts_ac_trip = 90

        if (system_bus_volts == 12):
            self.float_volts = ord(packet_buffer[33]) / 10.0

        if (system_bus_volts == 24):
            self.float_volts = ord(packet_buffer[33]) * 2 / 10.0

        if (system_bus_volts == 48):
            self.float_volts = ord(packet_buffer[33]) * 4 / 10.0

        if (system_bus_volts == 12):    # value added to absorbtion volts to give equalise volts range 0 - 2 volts
            self.equalise_volts = self.absorb_volts + (ord(packet_buffer[34]) / 10.0)

        if (system_bus_volts == 24):    # value added to absorbtion volts to give equalise volts range 0 - 2 volts
            self.equalise_volts = self.absorb_volts + (ord(packet_buffer[34]) * 2 / 10.0)

        if (system_bus_volts == 48):    # value added to absorbtion volts to give equalise volts range 0 - 2 volts
            self.equalise_volts = self.absorb_volts + (ord(packet_buffer[34]) * 4 / 10.0)
            
        self.absorb_time = ord(packet_buffer[35]) / 10.0                 # as decimal hours, 2.5 is 2 hours 30 minutes
        
    # remote_base send function:
    def encode(self, packet_buffer):
        global system_bus_volts

        self.status_code = 0x00
        self.search_watts = 0x05
        self.battery_size = 0x22 #220Ah
        self.battery_type = 0xF6 #246 = 24.6V * 2 =
        self.charger_amps = 0x1e  # 1e = 30%  % of charger capacity
        self.shore_ac_amps = 0x0a
        self.revision = 0x16 # V2.2
        self.parallel_threshold = 0x08
        self.force_charge = 0x10 #disable refloat
        self.genstart_auto = 0x00
        self.battery_low_trip = 0xF7 #24.7V * 2 = 49.4V
        self.volts_ac_trip = 0x9B # 80VAC
        self.float_volts = ord(packet_buffer[33]) * 4 / 10.0
        # value added to absorption volts to give equalize volts range 0 - 2 volts
        self.equalise_volts = self.absorb_volts + (ord(packet_buffer[34]) * 4 / 10.0)
        self.absorb_time = 0x0F # as decimal hours, 1.5hr = 15


class remote_A0_proto():
    """definition of remote A0 packet """
#    def __init__(self):
    remote_hours = 0.0          # 14    ord(packet_buffer[36])
    remote_min = 0.0            # 15    ord(packet_buffer[37])
    ags_run_time = 0.0          # 16    ord(packet_buffer[38]
    ags_start_temp = 0.0        # 17    ord(packet_buffer[39]
    ags_start_volts_dc = 0.0    # 18    ord(packet_buffer[40]
    ags_quite_hours = 0         # 19    ord(packet_buffer[41]
#    FOOTER 0xA0                # 20    ord(packet_buffer[42]
#    END
#
#    remoteA0_agsA1 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.remote_hours = ord(packet_buffer[36])       # duplicate from BMK class
        self.remote_min = ord(packet_buffer[37])         # duplicate from BMK class
        self.ags_run_time = ord(packet_buffer[38]) / 10.0
        self.ags_start_temp = ord(packet_buffer[39])
        if (system_bus_volts == 12):
            self.ags_start_volts_dc = ord(packet_buffer[40]) / 10.0

        if (system_bus_volts == 24):
            self.ags_start_volts_dc = ord(packet_buffer[40]) * 2 / 10.0

        if (system_bus_volts == 48):
            self.ags_start_volts_dc = ord(packet_buffer[40]) * 4 / 10.0

        self.ags_quite_hours = ord(packet_buffer[41])
        if (self.ags_quite_hours == 0):
            self.ags_quiet_hours_descrip = "Off"

        if (self.ags_quite_hours == 1):
            self.ags_quiet_hours_descrip = "21h - 07h"

        if (self.ags_quite_hours == 2):
            self.ags_quiet_hours_descrip = "21h - 09h"

        if (self.ags_quite_hours == 3):
            self.ags_quiet_hours_descrip = "21h - 09h"

        if (self.ags_quite_hours == 4):
            self.ags_quiet_hours_descrip = "22h - 08h"

        if (self.ags_quite_hours == 5):
            self.ags_quiet_hours_descrip = "23h - 08h"


class AGS1_proto():
    """definition of remote A0 packet """
#    HEADER 0xA1                # 21    ord(packet_buffer[43]
    ags_status = 0              # 22    ord(packet_buffer[44]
    ags_revision = 0.0          # 23    ord(packet_buffer[45]
    ags_temperature = 0.0       # 24    ord(packet_buffer[46]
    ags_gen_runtime = 0.0       # 25    ord(packet_buffer[47]
    ags_volts_dc = 0.0          # 26    ord(packet_buffer[48]
    ags_quiet_hours_descrip = "NA"
    ags_status_descript = "NA"
#    END
#
#    remoteA0_agsA1 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.ags_status = ord(packet_buffer[44])
        if (self.ags_status == 0):
            self.ags_status_descript = "Non Valid"

        if (self.ags_status == 1):
            self.ags_status_descript = "Off"

        if (self.ags_status == 2):
            self.ags_status_descript = "Ready"

        if (self.ags_status == 3):
            self.ags_status_descript = "Manual Run"

        if (self.ags_status == 4):
            self.ags_status_descript = "Inverter in Charge Mode"

        if (self.ags_status == 5):
            self.ags_status_descript = "In Quiet Time"

        if (self.ags_status == 6):
            self.ags_status_descript = "Start in Test Mode"

        if (self.ags_status == 7):
            self.ags_status_descript = "Start on Temperature"

        if (self.ags_status == 8):
            self.ags_status_descript = "Start on Voltage"

        if (self.ags_status == 9):
            self.ags_status_descript = "Fault Start on Test"

        if (self.ags_status == 10):
            self.ags_status_descript = "Fault Start on Temperature"

        if (self.ags_status == 11):
            self.ags_status_descript = "Fault Start on Voltage"

        if (self.ags_status == 12):
            self.ags_status_descript = "Start Time of Day"

        if (self.ags_status == 13):
            self.ags_status_descript = "Start State of Charge"

        if (self.ags_status == 14):
            self.ags_status_descript = "Start Exercise"

        if (self.ags_status == 15):
            self.ags_status_descript = "Fault Start Time of Day"

        if (self.ags_status == 16):
            self.ags_status_descript = "Fault Start State of Charge"

        if (self.ags_status == 17):
            self.ags_status_descript = "Fault Start Exercise"

        if (self.ags_status == 18):
            self.ags_status_descript = "Start on Amps"

        if (self.ags_status == 19):
            self.ags_status_descript = "Start on Topoff"

        if (self.ags_status == 20):
            self.ags_status_descript = "Non Valid"

        if (self.ags_status == 21):
            self.ags_status_descript = "Fault Start on Amps"

        if (self.ags_status == 22):
            self.ags_status_descript = "Fault Start on Topoff"

        if (self.ags_status == 23):
            self.ags_status_descript = "Non Valid"

        if (self.ags_status == 24):
            self.ags_status_descript = "Fault Maximum Run"

        if (self.ags_status == 25):
            self.ags_status_descript = "Gen Run Fault"

        if (self.ags_status == 26):
            self.ags_status_descript = "Generator in Warm Up"

        if (self.ags_status == 27):
            self.ags_status_descript = "Generator in Cool Down"

        self.ags_revision = ord(packet_buffer[45]) / 10.0
        self.ags_temperature = ord(packet_buffer[46])
        self.ags_run_time = ord(packet_buffer[47]) / 10.0

        if (system_bus_volts == 12):
            self.ags_volts_dc = ord(packet_buffer[48]) / 10.0

        if (system_bus_volts == 24):
            self.ags_volts_dc = ord(packet_buffer[48]) * 2 / 10.0

        if (system_bus_volts == 48):
            self.ags_volts_dc = ord(packet_buffer[48]) * 4 / 10.0



class remote_A1_proto():
    """definition of remote A1 packet """
#    def __init__(self):
    ags_start_time = 0.0        # 14    ord(packet_buffer[36]
    ags_stop_time = 0.0         # 15    ord(packet_buffer[37]
    ags_volts_dc_stop = 0.0     # 16    ord(packet_buffer[38]
    ags_start_delay = 0.0       # 17    ord(packet_buffer[39]
    ags_stop_delay = 0.0        # 18    ord(packet_buffer[40]
    ags_max_run_time = 0.0      # 19    ord(packet_buffer[41]
#    FOOTER 0xA1                # 20    ord(packet_buffer[42]#
#    remoteA1_agsA2 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.ags_start_time = ord(packet_buffer[36]) * 0.25
        self.ags_stop_time = ord(packet_buffer[37]) * 0.25

        if (ord(packet_buffer[36]) == ord(packet_buffer[37])):
            self.ags_start_stop_enable = "Disabled"
        else:
            self.ags_start_stop_enable = "Enabled"

        if (system_bus_volts == 12):
            self.ags_volts_dc_stop = ord(packet_buffer[38]) / 10.0

        if (system_bus_volts == 24):
            self.ags_volts_dc_stop = ord(packet_buffer[38]) * 2 / 10.0

        if (system_bus_volts == 48):
            self.ags_volts_dc_stop = ord(packet_buffer[38]) * 4 / 10.0

        if (ord(packet_buffer[39]) & 0x80):                                  # check for minutes selection as MSB:
            self.ags_start_delay = (ord(packet_buffer[39]) & 0x7f) * 60      # strip MSB and store as seconds
        else:
            self.ags_start_delay = ord(packet_buffer[39])                    # store seconds

        if (ord(packet_buffer[40]) & 0x80):                                  # check for minutes selection as MSB:
            self.ags_stop_delay = (ord(packet_buffer[40]) & 0x7f) * 60       # strip MSB and store as seconds
        else:
            self.ags_stop_delay = ord(packet_buffer[40])                     # store seconds

        self.ags_max_run_time = ord(packet_buffer[41]) / 10.0



class AGS2_proto():
    """definition of remote A1 packet """
#    def __init__(self):
#    HEADER 0xA2                # 21    ord(packet_buffer[43]
    ags_days_last_gen_run = 0   # 22    ord(packet_buffer[44]
#    ALWAYS 0                   # 23    ord(packet_buffer[45]
#    ALWAYS 0                   # 24    ord(packet_buffer[46]
#    ALWAYS 0                   # 25    ord(packet_buffer[47]
#    ALWAYS 0                   # 26    ord(packet_buffer[48]
    ags_start_stop_enable = "NA"
#    END
#
#    remoteA1_agsA2 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts
        self.ags_days_last_gen_run = ord(packet_buffer[44])


class remote_A2_proto():
    """definition of remote A2 packet """
#    def __init__(self):
    ags_soc_start = 0           # 14    ord(packet_buffer[36]
    ags_soc_stop = 0            # 15    ord(packet_buffer[37]
    ags_amps_start = 0          # 16    ord(packet_buffer[38]
    ags_amps_start_delay = 0    # 17    ord(packet_buffer[39]
    ags_amps_stop = 0           # 18    ord(packet_buffer[40]
    ags_amps_stop_delay = 0     # 19    ord(packet_buffer[41]
#    FOOTER 0xA2                # 20    ord(packet_buffer[42]
#    END
#
#    remote_A2 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.ags_soc_start = ord(packet_buffer[36])
        self.ags_soc_stop = ord(packet_buffer[37])
        self.ags_amps_start = ord(packet_buffer[38])

        if (ord(packet_buffer[39]) & 0x80):                                          # check for minutes selection as MSB:
            self.ags_amps_start_delay = (ord(packet_buffer[39]) & 0x7f) * 60         # strip MSB and store as seconds
        else:
            self.ags_amps_start_delay = ord(packet_buffer[39])                       # store seconds

        self.ags_amps_stop = ord(packet_buffer[40])

        if (ord(packet_buffer[41]) & 0x80):                                          # check for minutes selection as MSB:
            self.ags_amps_stop_delay = (ord(packet_buffer[41]) & 0x7f) * 60          # strip MSB and store as seconds
        else:
            self.ags_amps_stop_delay = ord(packet_buffer[41])                        # store seconds

class RTR_proto():
    """definition of remote A2 packet """
#    def __init__(self):
#    HEADER 0x91                # 21    ord(packet_buffer[43]
    rtr_revision = 0            # 22    ord(packet_buffer[44]
#    END
#
#    remoteA2_RTR decode function:
    def decode(self, packet_buffer):

        self.rtr_revision = ord(packet_buffer[44]) / 10.0

class remote_A3_proto():
    """definition of remote A3 packet """
#    def __init__(self):
    ags_quite_time_start = 0.0  # 14    ord(packet_buffer[36]
    ags_quite_time_stop = 0.0   # 15    ord(packet_buffer[37]
    ags_exercise_days = 0       # 16    ord(packet_buffer[38]
    ags_exercise_start_time = 0.0 # 17    ord(packet_buffer[39]
    ags_exercise_run_time = 0   # 18    ord(packet_buffer[40]
    ags_top_off = 0             # 19    ord(packet_buffer[41]
#    FOOTER 0xA3                # 20    ord(packet_buffer[42]
#    END
#
#    remoteA3 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts
        self.ags_quite_time_start = ord(packet_buffer[36]) * 0.25
        self.ags_quite_time_stop = ord(packet_buffer[37]) * 0.25
        self.ags_exercise_days = ord(packet_buffer[38])
        self.ags_exercise_start_time = ord(packet_buffer[39]) * 0.25
        self.ags_exercise_run_time = ord(packet_buffer[40]) / 10
        self.ags_top_off = ord(packet_buffer[41])


class remote_A4_proto():
    """definition of remote A4 packet """
#    def __init__(self):
    ags_warm_up_time = 0        # 14    ord(packet_buffer[36])
    ags_cool_down_time = 0      # 15    ord(packet_buffer[37]
#     ALWAYS 0                  # 16    ord(packet_buffer[38]
#     ALWAYS 0                  # 17    ord(packet_buffer[39]
#     ALWAYS 0                  # 18    ord(packet_buffer[40]
#     ALWAYS 0                  # 19    ord(packet_buffer[41]
#    FOOTER 0xA4                # 20    ord(packet_buffer[42]
#    END
#
#    remoteA4 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts
        
        if (ord(packet_buffer[36]) & 0x80):                                          # check for minutes selection as MSB:
            self.ags_warm_up_time = (ord(packet_buffer[36]) & 0x7f) * 60             # strip MSB and store as seconds
        else:
            self.ags_warm_up_time = ord(packet_buffer[36])                           # store seconds

        if (ord(packet_buffer[37]) & 0x80):                                          # check for minutes selection as MSB:
            self.ags_cool_down_time = (ord(packet_buffer[37]) & 0x7f) * 60           # strip MSB and store as seconds
        else:
            self.ags_cool_down_time = ord(packet_buffer[37])                         # store seconds


class remote_Z0_proto():
    """definition of remote Z0 packet """
#    def __init__(self):
    remote_hours = 0            # 14    ord(packet_buffer[36])
    remote_min = 0              # 15    ord(packet_buffer[37])
#     ALWAYS 0                  # 16    ord(packet_buffer[38])
#     ALWAYS 0                  # 17    ord(packet_buffer[39])
#     ALWAYS 0                  # 18    ord(packet_buffer[40])
#     ALWAYS 0                  # 19    ord(packet_buffer[41])
#     ALWAYS 0                  # 20    ord(packet_buffer[42])
#    END
#
#    remote_Z0 decode function:
    def decode(self, packet_buffer):
        global system_bus_volts
#        nothing to do here, move along:


class remote_BMK_proto():
    """definition of remote BMK packet """
#    def __init__(self):
    remote_hours = ""           # 14    ord(packet_buffer[36]
    remote_min = ""             # 15    ord(packet_buffer[37]
    bmk_battery_efficiency = 0  # 16    ord(packet_buffer[38]
    bmk_resets = 0              # 17    ord(packet_buffer[39]
    bmk_battery_size = 0        # 18    ord(packet_buffer[40]
#    ALWAYS 0                   # 19    ord(packet_buffer[41]
#    FOOTER 0x80                # 20    ord(packet_buffer[42]
#    END
#
#    remote_BMK decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.remote_hours = packet_buffer[36]
        self.remote_min = packet_buffer[37]
        self.bmk_battery_efficiency = ord(packet_buffer[38])
        self.bmk_resets = ord(packet_buffer[39])                     # XXX needs decoding XXX
        self.bmk_battery_size = ord(packet_buffer[40]) * 10


class BMK_proto():
    """definition of remote BMK packet """
#    def __init__(self):
#    HEADER 0x81                # 21    ord(packet_buffer[43]
    bmk_soc = 0                 # 22    ord(packet_buffer[44]
    bmk_volts_dc = 0.0          # 23    ord(packet_buffer[45] HIGH BYTE
                                # 24    ord(packet_buffer[46] LOW  BYTE
    bmk_amps_dc = 0             # 25    ord(packet_buffer[47] HIGH BYTE
                                # 26    ord(packet_buffer[48] LOW  BYTE 
    bmk_volts_min = 0.0         # 27    ord(packet_buffer[49] HIGH BYTE
                                # 28    ord(packet_buffer[50] LOW  BYTE
    bmk_volts_max = 0.0         # 29    ord(packet_buffer[51] HIGH BYTE
                                # 30    ord(packet_buffer[52] LOW  BYTE
    bmk_amp_hour_net = 0.0      # 31    ord(packet_buffer[53] HIGH BYTE
                                # 32    ord(packet_buffer[54] LOW  BYTE
    bmk_amp_hour_trip = 0       # 33    ord(packet_buffer[55] HIGH BYTE
                                # 34    ord(packet_buffer[56] LOW  BYTE
    bmk_amp_hour_total = 0      # 35    ord(packet_buffer[57] HIGH BYTE
                                # 36    ord(packet_buffer[58] LOW  BYTE
    bmk_revision = 0.0          # 37    ord(packet_buffer[59]
    bmk_fault = 0               # 38    ord(packet_buffer[60]
    bmk_fault_descrip = "NA"
#    END
#
#    remote_BMK decode function:
    def decode(self, packet_buffer):
        global system_bus_volts

        self.bmk_battery_soc = ord(packet_buffer[44])
        self.bmk_volts_dc = ((ord(packet_buffer[45]) * 256) + ord(packet_buffer[46])) / 100.0
#        self.bmk_amps_dc = (65535 - ((ord(packet_buffer[47]) * 256) + ord(packet_buffer[48]))) / 10.0
        self.bmk_amps_dc = (ord(packet_buffer[47]) * 256) + ord(packet_buffer[48])
        if (self.bmk_amps_dc > 32768):
            self.bmk_amps_dc = (65535 - self.bmk_amps_dc)
        self.bmk_amps_dc = self.bmk_amps_dc / 10

        self.bmk_volts_min = ((ord(packet_buffer[49]) * 256) + ord(packet_buffer[50])) / 100.0
        self.bmk_volts_max = ((ord(packet_buffer[51]) * 256) + ord(packet_buffer[52])) / 100.0

        self.bmk_amp_hour_net = (ord(packet_buffer[53]) * 256) + ord(packet_buffer[54])
        if (self.bmk_amp_hour_net > 32768):
            self.bmk_amp_hour_net = 65535 - self.bmk_amp_hour_net
            
        self.bmk_amp_hour_trip = ((ord(packet_buffer[55]) * 256) + ord(packet_buffer[56])) / 10
        self.bmk_amp_hour_total = ((ord(packet_buffer[57]) * 256) + ord(packet_buffer[58])) * 100

        self.bmk_revision = ord(packet_buffer[59]) / 10.0
        self.bmk_fault = ord(packet_buffer[60])
        if (self.bmk_fault == 0):
            self.bmk_fault_descrip = "Reserved"

        if (self.bmk_fault == 1):
            self.bmk_fault_descrip = "No Faults"    # documentation says 'Normal', what ever that is:

        if (self.bmk_fault == 2):
            self.bmk_fault_descrip = "Fault Start"



# main application function:
def main():
    inverter = inverter_proto()
    remote_base = remote_base_proto()
    remote_A0 = remote_A0_proto()
    remote_A1 = remote_A1_proto()
    remote_A2 = remote_A2_proto()
    remote_A3 = remote_A3_proto()
    remote_A4 = remote_A4_proto()
    remote_Z0 = remote_Z0_proto()
    remote_BMK = remote_BMK_proto()
    BMK = BMK_proto()    
    RTR = RTR_proto()
    AGS1 = AGS1_proto()
    AGS2 = AGS2_proto()
    global serial_port          # access to the global serial port argument:

#    open commuication port:
    def openPort(serial_port):
        try:
            mag_port = serial.Serial(port = serial_port,
#            mag_port = serial.Serial(port = '/dev/ttyUSB0',
#            mag_port = serial.Serial(port = '/dev/tty.usbmodem621',
                            baudrate = 19200,
                            bytesize = 8,
                            timeout = None)
            
            return(mag_port)
        except:
            print('Error: Failed to open commuications port, exiting')
            exit()

#    main application loop:
    def mainLoop():
        serial_byte     = 0
        datacount       = 0
        sync_locked     = 0
        serial_buffer   = array.array('i')

#        call to open the communications port and assign handle on success:
        mag_port = openPort(serial_port)
        
        mag_port.read(100)
        mag_port.flush()
      
        sync_locked = 0
        bytes_waiting = 0

        while (sync_locked < 0.03):
          sync_check_start = time.time()

          try:
            serial_byte = ord(mag_port.read(1))
            #print(format(serial_byte, '02x')), #debug

          except:
            print
            print("Error: Failed to read communications Port, exiting")
            mag_port.close()
            exit()
        
          sync_check_stop = time.time()
          sync_locked = sync_check_stop - sync_check_start
            
        if (sync_locked > 0.03):

          if debug_level > 0:
            print
            print("sync")
          
            for i in range(0,10):
              print(str(i) + " "),

            for i in range(10,60):#bytes_waiting -1):
              print(i),
            print

          for p in range(0,8):

            time.sleep(0.07)
         
            bytes_waiting = mag_port.inWaiting()
       
            serial_buffer = chr(serial_byte) + mag_port.read(bytes_waiting)

            if debug_level > 0:
              print("(" + str(bytes_waiting+1) + ")"),

              for i in range(0,bytes_waiting+1):
                print(format(ord(serial_buffer[i]), '02x')),

              print

            #Check if we have a remoted connected and decode it, otherwise program fails
            if (len(serial_buffer) > 22):
                if (ord(serial_buffer[42]) == 0xa0):
                  if debug_level > 0:
                    print("Remote A0 Packet ")
                  remote_A0.decode(serial_buffer)

                elif (ord(serial_buffer[42]) == 0xa1):
                  if debug_level > 0:
                    print("Remote A1 Packet")
                  remote_A1.decode(serial_buffer)

                elif (ord(serial_buffer[42]) == 0xa2):
                  if debug_level > 0:
                    print("Remote A2 Packet")
                  remote_A2.decode(serial_buffer)

                elif (ord(serial_buffer[42]) == 0xa3):
                  if debug_level > 0:
                    print("Remote A3 Packet")
                  remote_A3.decode(serial_buffer)

                elif (ord(serial_buffer[42]) == 0xa4):
                  if debug_level > 0:
                    print("Remote A4 Packet")
                  remote_A4.decode(serial_buffer)
            

                #elif (ord(serial_buffer[42]) == 0x11):
                
                elif (ord(serial_buffer[42]) == 0x00):
                  if debug_level > 0:
                    print("Remote Z0 Packet")
                  remote_Z0.decode(serial_buffer)

                elif (ord(serial_buffer[42]) == 0x80):
                  if debug_level > 0:
                    print("Remote BMK Packet")
                  remote_BMK.decode(serial_buffer)
              
            #decode if an accesory responded 
            if(bytes_waiting+1 > 43):
              if debug_level > 0:
                print ("Decoding Accessory"),

              if (ord(serial_buffer[43]) == 0x81):
                if debug_level > 0:
                  print("BMK Packet")
                BMK.decode(serial_buffer)

              elif (ord(serial_buffer[43]) == 0x91):
                if debug_level > 0:
                  print("RTR Packet")
                RTR.decode(serial_buffer)
              elif (ord(serial_buffer[43]) == 0xA1):
                if debug_level > 0:
                  print("AGS1 Packet")
                AGS1.decode(serial_buffer)              
              elif (ord(serial_buffer[43]) == 0xA2):
                if debug_level > 0:
                  print("AGS2 Packet")
                AGS1.decode(serial_buffer)

            if p < 7:
              serial_byte = ord(mag_port.read(1))
        #remote_base.decode(serial_buffer)
        inverter.decode(serial_buffer)

        if debug_level > 0:
            report_results()        # print results to terminal:
        else:
            pubMqtt()               # publish to MQTT broker
            
        #mag_port.close()        # close the com port and finish:

    #application report via MQTT
    def pubMqtt():
        #Publish device versions
        client.publish("inverter/desc", inverter.model_descript)
        client.publish("inverter/rev", inverter.revision)
    
        if (RTR.rtr_revision != 0):
          client.publish("inverter/me-rtr-rev",RTR.rtr_revision)
        else:
          client.publish("inverter/me-arc-rev",remote_base.revision)
        if (BMK.bmk_revision != 0):
          client.publish("inverter/bmk-rev", BMK.bmk_revision)
        if (AGS1.ags_revision != 0):
          client.publish("inverter/ags-rev", remote_A0_agsA1.ags_revision)
  
        acwatts = int((inverter.volts_ac_out)*(inverter.amps_ac_in-inverter.amps_ac_out)) #assumes symmetric load across phases
        dcwatts = inverter.volts_dc*inverter.amps_dc
        eff = round(abs(safeDiv(acwatts, dcwatts)*100), 2) #extra function prevents divide by zero error if power is 0
        
        client.publish("inverter/mode",inverter.status_descript)
        client.publish("inverter/faults",inverter.fault_descript)
    
        #AC readings
        client.publish("inverter/ac/vout", inverter.volts_ac_out)
        client.publish("inverter/ac/iin", inverter.amps_ac_in)
        client.publish("inverter/ac/iout", inverter.amps_ac_out)
        client.publish("inverter/ac/freq", inverter.frequency_ac_out)
	client.publish("inverter/ac/power", acwatts)        

        #DC readings
        client.publish("inverter/dc/vin", inverter.volts_dc)
        client.publish("inverter/dc/iout", inverter.amps_dc)
        client.publish("inverter/dc/power", inverter.volts_dc*inverter.amps_dc)
    
        #Inverter thermal readings
        client.publish("inverter/thermal/battemp", int(inverter.temp_battery))
        client.publish("inverter/thermal/xfmrtemp", int(inverter.temp_transformer))
        client.publish("inverter/thermal/fettemp", int(inverter.temp_FET))

#    application report function:
    def report_results():
        print("")
        print("Equip List")
        print("  " + inverter.model_descript + "({0})".format(inverter.revision))
        if (RTR.rtr_revision != 0):
          print("  ME-RTR({0})".format(RTR.rtr_revision))
        else:
          print("  ME-ARC({0})".format(remote_base.revision))
        if (BMK.bmk_revision != 0):
          print("  BMK({0})".format(BMK.bmk_revision))
        if (AGS1.ags_revision != 0):
          print("  AGS({0})".format(remote_A0_agsA1.ags_revision))
      
        acwatts = int(inverter.volts_ac_out*(inverter.amps_ac_in-inverter.amps_ac_out))
        dcwatts = inverter.volts_dc*inverter.amps_dc
        eff = round(abs(safeDiv(acwatts, dcwatts)*100), 2) #extra function prevents divide by zero error if power is 0
            
        print("\nLive Data")
        print("\tClock                     {:02}:{:02}".format(remote_A0.remote_hours, remote_A0.remote_min))
        print("")
        print("\tInverter Mode:            " + inverter.status_descript)
        print("\tFault:                    " + inverter.fault_descript)
        print("")
        print("\tOutput AC Volts:          {0} Vac".format(inverter.volts_ac_out))
        print("\tInput AC Amps:            {0} Amps".format(inverter.amps_ac_in))
        print("\tOutput AC Amps:           {0} Amps".format(inverter.amps_ac_out))
        print("\tL1 + L2 AC Output Watts:  {0} Watts".format(int(inverter.volts_ac_out*(inverter.amps_ac_in-inverter.amps_ac_out))))
        #print("")
        #print("\tInput AC Volts:  {0} Vac".format(inverter.volts_ac_in))
        print("\tInput AC Amps:            {0} Amps".format(inverter.amps_ac_in))
        #print("\tInput AC Power:   {0} kW".format(inverter.volts_ac_in*inverter.amps_ac_in/1000.0))
        print("\tLine Frequency            {0} Hz".format(inverter.frequency_ac_out))
        print("\tAC/DC Efficiency          {0} %".format(eff))
        print("")
        print("\tTemperature Battery       {0} F".format(int(1.8 * inverter.temp_battery + 32), 0))
        print("\tTemperature Transformer   {0} F".format(int(1.8 * inverter.temp_transformer + 32), 0))
        print("\tTemperature FETs          {0} F".format(int(1.8 * inverter.temp_FET + 32), 0))
        print("")
        if (BMK.bmk_revision != 0):
          print("\tFault Status:             " + BMK.bmk_fault_descrip)
          print("\tBattery State of Charge:  {0} %".format(BMK.bmk_battery_soc))
          print("\tBattery Volts DC:         {0} Vdc".format(BMK.bmk_volts_dc))
          print("\tAmps DC:                  {0} Amps".format(BMK.bmk_amps_dc))
          print("\tPower DC:                 {0} Watts".format(BMK.bmk_volts_dc*BMK.bmk_amps_dc))
          print("\tBattery Max:              {0} Vdc".format(BMK.bmk_volts_max))
          print("\tBattery Min:              {0} Vdc".format(BMK.bmk_volts_min))
          print("\tBattery Amp Hour Net      {0} AmpHr".format(BMK.bmk_amp_hour_net))
          print("\tBattery Amp Hour Trip     {0} AmpHr".format(BMK.bmk_amp_hour_trip))
          print("\tBattery Amp Hour Total    {0} AmpHr".format(BMK.bmk_amp_hour_total))
        else:
          print("\tInverter Input Volts:     {0} Vdc".format(inverter.volts_dc))
          print("\tInverter Input Amps:      {0} Amps".format(inverter.amps_dc))
          print("\tInverter Input Power:     {0} Watts".format(inverter.volts_dc*inverter.amps_dc))
        print("")
        
        #print("")
        #print("\tGenerator Run Time        {0} Hrs".format(remote_A0_agsA1.ags_run_time))
        #print("\tGenerator Last Run        {0} Days".format(remote_A1_agsA2.ags_days_last_gen_run))

        #print("")
        #print("AGS - ver({0})".format(remote_A0_agsA1.ags_revision))
        #print("\tStatus:                   " + remote_A0_agsA1.ags_status_descript)
        #print("\tGen Start Mode:           " + remote_base.genstart_auto_descript)
        #print("\tQuiet Hours:              " + remote_A0_agsA1.ags_quiet_hours_descrip)
        #print("\t\tQuiet Time Start  {0} Hrs".format(remote_A3.ags_quite_time_start))
        #print("\t\tQuiet Time Stop   {0} Hrs".format(remote_A3.ags_quite_time_stop))
        #print("")
        #print("\tAuto Start                " + remote_A1_agsA2.ags_start_stop_enable)
        #print("\t\tStart Time        {0} Hrs".format(remote_A1_agsA2.ags_start_time))
        #print("\t\tStop Time         {0} Hrs".format(remote_A1_agsA2.ags_stop_time))
        #print("")
        #print("\tStart Delay               {0} Sec".format(remote_A1_agsA2.ags_start_delay))
        #print("\tStop Delay                {0} Sec".format(remote_A1_agsA2.ags_stop_delay))
        #print("\tWarm Up Time              {0} Sec".format(remote_A4.ags_warm_up_time))
        #print("\tCool Down Time            {0} Sec".format(remote_A4.ags_cool_down_time))
        #print("\tSOC Start                 {0} %".format(remote_A2_RTR.ags_soc_start))
        #print("\tSOC Stop                  {0} %".format(remote_A2_RTR.ags_soc_stop))
        #print("\tAmps Start                {0} Amps".format(remote_A2_RTR.ags_amps_start))
        #print("\tAmps Stop                 {0} Amps".format(remote_A2_RTR.ags_amps_stop))
        #print("\t\tAmps Start Delay  {0} Sec".format(remote_A2_RTR.ags_amps_start_delay))
        #print("\t\tAmps Stop Delay   {0} Sec".format(remote_A2_RTR.ags_amps_stop_delay))
        #print("")
        #print("\tMaximum Run Time          {0} Hrs".format(remote_A1_agsA2.ags_max_run_time))
        #print("\tTop Off Time              {0} Min".format(remote_A3.ags_top_off))
        #print("\tExercise Day Period       {0} Days".format(remote_A3.ags_exercise_days))
        #print("\tExercise Start Time       {0} Hrs".format(remote_A3.ags_exercise_start_time))
        #print("\tExercise Run Time         {0} Hrs".format(remote_A3.ags_exercise_run_time))
        
        #print("\tStart Temperature         {0} F".format(remote_A0_agsA1.ags_start_temp))
        #print("\tStart Volts               {0} Vdc".format(remote_A0_agsA1.ags_start_volts_dc))
        #print("\tStop Volts                {0} Vdc".format(remote_A1_agsA2.ags_volts_dc_stop))

        #print("")
        #print(inverter.model_descript + " - ver({0})".format(inverter.revision))
        #print("\tSystem Bus Voltage:      {0} Vdc".format(system_bus_volts))
        #print("\tInverter Stack Mode:      " + inverter.stack_mode)
        #print("\tSearch Watts:             {0} Watts".format(remote_base.search_watts))
        #print("\tCharger Amps:             {0} %".format(remote_base.charger_amps))
        #print("\tShore AC Amps:            {0} Amps".format(remote_base.shore_ac_amps))
        #print("\tParallel Threshold:       {0} %".format(remote_base.parallel_threshold))
#        print("\tForce Charge:             {0}".format(remote_base.force_charge))
#        print("\tForce Charge:             " + remote_base.force_charge_descript)
        #print("\tAC Volts Trip:            {0} Vac".format(remote_base.volts_ac_trip))

        #print("")
        #print("Battery Settings")
        #print("\tBattery Size, remote:     {0} AmpHr".format(remote_base.battery_size))
        #print("\tBattery Size, BMK:        {0} AmpHr".format(BMK.bmk_battery_size))
        #print("\tBattery Type:             " + remote_base.battery_type_descript)
        #print("\tBattery Efficiency:       {0} %".format(BMK.bmk_battery_efficiency))
        #print("\tFloat Volts:              {0} Vdc".format(remote_base.float_volts))
        #print("\tAbsorb Volts:             {0} Vdc".format(remote_base.absorb_volts))
        #print("\tEqualise Volts:           {0} Vdc".format(remote_base.equalise_volts))
        #print("\tBattery Low Trip:         {0} Vdc".format(remote_base.battery_low_trip))

    mainLoop()

t=time.time()
#application entry point:
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", default = "/dev/ttyUSB0", help="commuications port descriptor, e.g /dev/ttyUSB0 or COM1")
    parser.add_argument("-d", "--debug", default = 0, type=int, choices=[0, 1, 2], help="debug data")
    args = parser.parse_args()
    
    serial_port = args.port
    debug_level = args.debug 

    print("MagPy Magnum Energy MagnaSine Data Protocol Decoder\n")
    print("Debug level : {0}".format(debug_level))
    print("serial port : " + serial_port + "\n")
    
    system_bus_volts = 0    # set as global variable:
    while True:
	if time.time()-t > 5:
            main()
            t=time.time()