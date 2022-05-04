#!/usr/bin/env python

# Move to Absolute position
#G0 {X:X%.4f} {Y:Y%.4f} {Z:Z%.4f} {Rotation:E%.4f} F{FeedRate:%.0f}

# PLACE_COMMAND
# M800 ; Turn on nozzle 1 vacuum solenoid



from twisted.python import log, usage
import sys
import time
import json
from pylibftdi import BitBangDevice, INTERFACE_A, INTERFACE_B, INTERFACE_C, INTERFACE_D

if sys.platform == 'win32':
    from twisted.internet import win32eventreactor
    win32eventreactor.install()


from twisted.internet.protocol import ServerFactory
from twisted.protocols.basic import LineReceiver


class QuadGpio():
#    union {
#        struct {
#            uint8_t x_limit_m : 1;
#            uint8_t x_limit_p : 1;
#            uint8_t z_home_up : 1;
#            uint8_t p_home : 1;
#            uint8_t z_home_safety : 1;
#            uint8_t z_limit_down : 1;
#            uint8_t t_home_inverted : 1;
#            uint8_t vac_solenoid : 1;
#        };
#        uint8_t data
#        struct {
#            uint8_t unlock_solenoid : 1;
#            uint8_t nozzle_1_in : 1;
#            uint8_t nozzle_2_in : 1;
#            uint8_t nozzle_3_in : 1;
#            uint8_t solenoid_open : 1;
#            uint8_t solenoid_closed : 1;
#            uint8_t : 1;
#            uint8_t illumination : 1;
#        };
   def __init__(self):
      self._adc = BitBangDevice(interface_select = INTERFACE_A)
      self._adc.direction = 0x0B  #0bxxxx1011
      self.SetAdcCs(True)
      self.SetAdcClk(True)
      self._head = BitBangDevice(interface_select = INTERFACE_B)
      self._head.direction = 0x80
      self._chgr1 = BitBangDevice(interface_select = INTERFACE_C)
      self._chgr1.direction = 0x81
      self._chgr2 = BitBangDevice(interface_select = INTERFACE_D)
      self._chgr2.direction = 0x81

   def SetAdcCs(self, state):
      if (state):
         self._adc.port |= 0x08
      else:
         self._adc.port &= (~0x08)

   def SetAdcClk(self, state):
      if (state):
         self._adc.port |= 0x01
      else:
         self._adc.port &= (~0x01)

   def SetAdcOut(self, state):
      if (state):
         self._adc.port |= 0x02
      else:
         self._adc.port &= (~0x02)

   def ClockAdc(self):
      self.SetAdcClk(False)
      time.sleep(0.001)
      self.SetAdcClk(True)
      time.sleep(0.001)


   def ReadAdcAverage(self, channel):
      total = 0
      for i in range(0,9):
         total += self.ReadAdc(channel)
         time.sleep(0.05)
      return total / 10

   def ReadAdc(self, channel):
      self._adc_reading = 0
      self.SetAdcCs(False)
      self.SetAdcOut(True)  # Start Bit = 1
      self.ClockAdc()
      self.SetAdcOut(True)  # Single Ended Mode = 1
      self.ClockAdc()
      self.SetAdcOut(False) # Channel Select = 0
      self.ClockAdc()
      self.SetAdcOut(True)  # MSBF MSB First = 1
      self.ClockAdc()

      for i in range(0,11): # tConv time
         self.ClockAdc()

      for i in range(0,11): # tData time
         self.ClockAdc()
         if self._adc.port & 0x4:
            self._adc_reading = (self._adc_reading << 1) | 0x1
         else:
            self._adc_reading = (self._adc_reading << 1)

      self.SetAdcCs(True)
      return self._adc_reading

   def UnlockChanger(self, changer):
      if (changer == 1):
         self._chgr1.port |= 0x01
      elif changer == 2:
         self._chgr2.port |= 0x01

   def LockChanger(self, changer):
      if (changer == 1):
         self._chgr1.port |= ~0x01
      elif changer == 2:
         self._chgr2.port |= ~0x01

   def UpperCameraRing(self, state):
      if (state):
         self._chgr1.port |= 0x80
      else:
         self._chgr1.port &= (~0x80)

   def FiducialIlluminator(self, state):
      if (state):
         self._chgr2.port |= 0x80
      else:
         self._chgr2.port &= (~0x80)

   def Pick(self):
         print "N1 Pick"
         self._head.port |= 0x80
   def Place(self):
         print "N1 Place"
         self._head.port &= (~0x80)

   def IsZSafe(self):
         if ( self._head.port & 0x10 ):
            print "Z is safe"
            return True
         else: 
            print "Z is NOT safe"
            return False
   def IsZHomeUp(self):
         if ( self._head.port & 0x4 ):
            print "Z is Home Up"
            return True
         else: 
            print "Z is NOT Home Up"
            return False


class SrxLineReceiver(LineReceiver):
   def __init__(self):
      self.setRawMode()
      self._parent = None

   def SetParent(self, parent):
      self._parent = parent

   def rawDataReceived(self, line):
      self._parent.submitSrxResponse(line)



class GcodeLineReceiver(LineReceiver):
   def __init__(self):
      self.delimiter = "\n"
      self._parent = None

   def SetParent(self, s):
      self._parent = s

   def lineReceived(self, line):
      self._parent.submitGcodeLine(line)

class Gcode2Srx:
   def __init__(self):
      self._glr = GcodeLineReceiver()
      self._slr = SrxLineReceiver()
      self._g = SerialPort(self._glr, "/dev/ttyUSB4", reactor, baudrate=19200)
      self._s = SerialPort(self._slr, "/dev/ttyUSB5", reactor, baudrate=19200)
      self._glr.SetParent(self)
      self._slr.SetParent(self)
      self._quadgpio = QuadGpio()
      self.HomeMachine()

   def GcodeReadVacuum(self):
      vacuum = self._quadgpio.ReadAdcAverage(0)
      self.sendGcodeResponse("vacuum:" + str(vacuum) +"\r\n")
      self.sendGcodeResponse("ok")

   def submitGcodeLine(self, line):
      print "Gcode: " + line
      if (line[0] == 'G'):
         self.GcodeParseMove(line[1:])
      elif (line[0] == 'M'):
         self.GcodeParseMisc(line[1:])
      elif (line == 'VACUUM'):
         self.GcodeReadVacuum()
      else:
         self.sendLine("error: unrecognized command prefix")
         return

   def submitSrxResponse(self, line):
      print "SRX_r: " + line
      if ( line[0] == "!" ):
         self.sendGcodeResponse("ok")
      elif ( line[0] == "#" ):
         self.sendGcodeResponse("error: limit reached")
      else:
         self.sendGcodeResponse("error: invalid response from SRX: " + line)

   def submitSrxCmd(self, line):
      print "SRX: " + line
      self._slr.sendLine(line)

   def sendGcodeResponse(self, line):
      print "GCode_r: " + line 
      self._glr.sendLine( line + "\n")

   def GcodeParseMove(self, line):
      params = line.split()
      if params[0] == "0":
         self.GcodeParseG0(params[1:])
      elif params[0][0] == "4":
         self.GcodeParseG4(params[1:])
      elif params[0] == "21":
         self.sendGcodeResponse("ok")
      elif params[0] == "90":  #G90 is set Absolute positioning mode, we use it for initialization
#         srx_cmd =  "AZ VL3000  " 
         srx_cmd =  "AZ VL3000 AT VL10000 " 
         self.submitSrxCmd(srx_cmd)
         self.sendGcodeResponse("ok")
      else:
         self.sendGcodeResponse("error: unrecognized command: G" + params[0])

   def HomeMachine(self):
	if (self._quadgpio.IsZSafe() or self._quadgpio.IsZHomeUp()):
	   #Move Z down a little
           self.submitSrxCmd("AZ VL4000 MR250 GD")
        #Home Z
        self.submitSrxCmd("AZ LR GD LP0 IP WQ RP")

   def convertGcode2Srx(self, millimeters):
      srx_mv = int(1000/25.4 * millimeters)
      if (srx_mv > 0):
         return srx_mv
      else:
         return 0

   def GcodeParseG4(self, params):
      # TODO, Actually parse the sleep command
      time.sleep(0.250)
      self.sendGcodeResponse("ok")

   def GcodeParseG0(self, params):
      srx_cmd = ""
      feedrate = None
      axes = [ "", "", "", "" ]
      count = 0 

      self._quadgpio.IsZHomeUp()

      for param in params:
         if "X" in param:
            axes[1] = str(self.convertGcode2Srx(-float(param[1:])))
            count += 1
            if (axes[1]<0):
               self.sendGcodeResponse("X Axis cannot be moved to negative location")
               return
            if ( not self._quadgpio.IsZSafe() and not self._quadgpio.IsZHomeUp() ):
               self.sendGcodeResponse("X Axis cannot be moved when Z is not safe")
               return

         elif "Y" in param:
            axes[0] = str(self.convertGcode2Srx(-float(param[1:])))
            count += 1
            if (axes[0]<0):
               self.sendGcodeResponse("Y Axis cannot be moved to negative location")
               return
            if ( not self._quadgpio.IsZSafe() and not self._quadgpio.IsZHomeUp() ):
               self.sendGcodeResponse("Y Axis cannot be moved when Z is not safe")
               return
         elif "Z" in param:
            axes[2] = str(self.convertGcode2Srx(-float(param[1:])))
            count += 1
            if (axes[2]<10):
                  axes[2] = 10

         elif "E" in param:
            count += 1
            axes[3] = str(int(float(param[len("E"):]) * 200 / 3) + 180)
#            axes[3] = str(self.convertGcode2Srx(float(param[len("E"):]) ))
         elif "F" in param:
            feedrate = param[len("F"):]
         else:
	    print "error: unrecognized param"
            self.sendGcodeResponse("error: unrecognized parameter")
            return
      if (count == 0):
         print "error: unrecognized param"
         self.sendGcodeResponse("error: no parameters in G0 move")
      else:
         srx_cmd =  "AA MA" + ','.join(axes) + "; GD ID" 
         self.submitSrxCmd(srx_cmd)
      return

   def GcodeParseMisc(self, line):
      params = line.split()
      self._quadgpio.IsZSafe()

      if params[0] == "84": # Stop Steppers
         self.submitSrxCmd("KL") # translate to Kill Motion/Flush Queue
         self.sendGcodeResponse("ok")
      elif params[0] == "400":
         self.submitSrxCmd("IP")
      elif params[0] == "800":
         self._quadgpio.Pick()
         self.sendGcodeResponse("ok")
      elif params[0] == "801":
         self._quadgpio.Place()
         self.sendGcodeResponse("ok")
      elif params[0] == "810":
         self._quadgpio.UpperCameraRing(True)
         self.sendGcodeResponse("ok")

      elif params[0] == "811":
         self._quadgpio.UpperCameraRing(False)
         self.sendGcodeResponse("ok")
      elif params[0] == "999":
         self._quadgpio.IsZSafe()
      else:
         print "Misc: " + json.dumps(params)


if __name__ == '__main__':
    from twisted.internet import reactor
    from twisted.internet.serialport import SerialPort
    gcode2srx = Gcode2Srx()
    print "Run"
    reactor.run()
