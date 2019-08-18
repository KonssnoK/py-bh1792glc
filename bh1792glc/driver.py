"""
    Driver for BH1792GLC Pulse Sensor
"""
import struct
from bh1792glc import bh1792glc_registers
from bh1792glc.i2c_interface import *
import logging
import time
import RPi.GPIO as GPIO

r = bh1792glc_registers.registers()
b = bh1792glc_registers.bits()
m = bh1792glc_registers.masks()
e = bh1792glc_registers.enums()

get_time_ms = lambda: int(round(time.time() * 1000))

class BH1792GLCDriver(SensorDriver):

    def __init__(self, int_gpio = 17):
        """ GPIO is the pin 
        """
    
        SensorDriver.__init__(self, 0x5B, 1)
        self.name = 'BH1792GLC'
        self._registers = dict(r.__dict__)
        self._dump_range = (r.BH1792GLC_REGISTER_DUMP_START,
                            r.BH1792GLC_REGISTER_DUMP_END)
        logging.basicConfig(level=logging.DEBUG)

        self.last_sync_ms = 0
        self.sync_counter = 0
        
        #Set up interrupt
        self.int_gpio = int_gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(int_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def probe(self):
        self.connected = True
        resp = self.read_register(r.BH1792GLC_PART_ID)
        if resp == b.BH1792GLC_PART_ID_PART_ID_ID:
            logging.info('detected BH1792GLC')
            return 1
        self.connected = False
        return 0

    def reset(self):
        self.write_register(r.BH1792GLC_RESET, b.BH1792GLC_RESET_SWRESET)
        time.sleep(0.002)#wait tPSC 2ms

    def measure_sync_start( self, 
                            mode=b.BH1792GLC_MEAS_CONTROL1_MSR_64HZ,
                            current= 10,
                            threshold=0xFFFC):
        # Set operation mode
        self.set_meas_mode(mode, b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_GREEN)
        # Set 0x42
        self.set_led1_current(current)
        # Set 0x43
        self.set_led2_current(current)
        # Set 0x44, 0x45
        self.set_ir_threshold(threshold)
        # Set interrupt condition
        self.set_interrupt_mode(b.BH1792GLC_MEAS_CONTROL5_INT_SEL_FIFO_WATERMARK)
        # Start measurement
        self.write_register(r.BH1792GLC_MEAS_START, b.BH1792GLC_MEAS_START_MEAS_ST)
        # Synch
        self.sync_counter = 0
        self.measure_sync_ontick()
        
    def measure_sync_ontick( self ):
        #High priority
        if (get_time_ms() - self.last_sync_ms) >= 1000:
            #Synch each second
            self.write_register(r.BH1792GLC_MEAS_SYNC, b.BH1792GLC_MEAS_SYNC_MEAS_SYNC)
            self.last_sync_ms = get_time_ms()
            
            self.sync_counter+=1
            
        #Ignore the interrupt before third cycle
        if self.sync_counter < 2:
            return []
            
        #Low priority
        #if interrupt raised or we need to clear the FIFO
        if (not GPIO.input(self.int_gpio) or (self.sync_counter == 2)):
            # Read from the FIFO
            reads = 0
            d0 = []
            d1 = []
            while reads < 32:
                #read FIFO 0x4C to 0x4F
                d0.append(self.read_short(r.BH1792GLC_FIFODATA0_LSB))
                d1.append(self.read_short(r.BH1792GLC_FIFODATA1_LSB))
                reads+=1
            # Read FIFO_LEV
            lev = self.read_register(r.BH1792GLC_FIFO_LEV)
            # No changes in parameters
            
            if (self.sync_counter == 2):
                #We cleared the FIFO, no data
                return []
            
            return [d0,d1]
        
        return []
            
    def measure_sync_stop( self ):
        self.reset()

    def measure_single_get( self,
                            adc= b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_GREEN,
                            current= 10,
                            threshold=0xFFFC):
        # Set operation mode
        self.set_meas_mode(
            b.BH1792GLC_MEAS_CONTROL1_MSR_SINGLE_MEAS_MODE, 
            adc )
        # Set 0x42
        self.set_led1_current(current)
        # Set 0x43
        self.set_led2_current(current)
        # Set 0x44, 0x45
        self.set_ir_threshold(threshold)
        # Set interrupt condition
        self.set_interrupt_mode(b.BH1792GLC_MEAS_CONTROL5_INT_SEL_ON_COMPLETE)
        # Start measurement
        self.write_register(r.BH1792GLC_MEAS_START, b.BH1792GLC_MEAS_START_MEAS_ST)
        
        while GPIO.input(self.int_gpio):
            time.sleep(0.001)
        
        val_off = 0
        val_on = 0
        
        if adc == b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_GREEN:
            # Read 0x54 to 0x57
            val_off = self.read_short(r.BH1792GLC_GDATA_LEDOFF_LSB)
            val_on = self.read_short(r.BH1792GLC_GDATA_LEDON_LSB)
        else:
            # Read 0x50 to 0x53
            val_off = self.read_short(r.BH1792GLC_IRDATA_LEDOFF_LSB)
            val_on = self.read_short(r.BH1792GLC_IRDATA_LEDON_LSB)
        
        # Clear interrupt 0x58
        self.read_register(r.BH1792GLC_INT_CLEAR)
        
        # Stop measurement
        self.reset()
        
        return [val_off, val_on]
        

    def set_nonsync_measurement(self, current=40, threshold=0xffff):
        self.reset()
        self.set_meas_mode(b.BH1792GLC_MEAS_CONTROL1_MSR_NON_SYNCH_MODE,
                           b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_IR)
        self.set_led2_current(current)
        self.set_interrupt_mode(b.BH1792GLC_MEAS_CONTROL5_INT_SEL_IR_THRESHOLD)
        self.set_ir_threshold(threshold)


    # Set registers 

    def set_led1_current(self, current):
        assert current <= m.BH1792GLC_MEAS_CONTROL2_LED_CURRENT1_MASK, 'Current too high'
        
        self.write_register(r.BH1792GLC_MEAS_CONTROL2, current)

    def set_led2_current(self, current):
        assert current <= m.BH1792GLC_MEAS_CONTROL3_LED_CURRENT2_MASK, 'Current too high'
        
        self.write_register(r.BH1792GLC_MEAS_CONTROL3, current)

    def set_meas_mode(self, frequency, adc):
        assert frequency in [b.BH1792GLC_MEAS_CONTROL1_MSR_32HZ,
                             b.BH1792GLC_MEAS_CONTROL1_MSR_64HZ,
                             b.BH1792GLC_MEAS_CONTROL1_MSR_128HZ,
                             b.BH1792GLC_MEAS_CONTROL1_MSR_256HZ,
                             b.BH1792GLC_MEAS_CONTROL1_MSR_1024HZ,
                             b.BH1792GLC_MEAS_CONTROL1_MSR_NON_SYNCH_MODE,
                             b.BH1792GLC_MEAS_CONTROL1_MSR_SINGLE_MEAS_MODE]
        assert adc in [b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_GREEN,
                       b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_IR]
                       
        self.write_register(r.BH1792GLC_MEAS_CONTROL1,
                            frequency | adc | b.BH1792GLC_MEAS_CONTROL1_RDY)

    def set_interrupt_mode(self, mode):
        assert mode in [b.BH1792GLC_MEAS_CONTROL5_INT_SEL_DISABLE,
                        b.BH1792GLC_MEAS_CONTROL5_INT_SEL_FIFO_WATERMARK,
                        b.BH1792GLC_MEAS_CONTROL5_INT_SEL_IR_THRESHOLD,
                        b.BH1792GLC_MEAS_CONTROL5_INT_SEL_ON_COMPLETE]
        self.write_register(r.BH1792GLC_MEAS_CONTROL5, mode)

    def set_ir_threshold(self, threshold):
        assert threshold <= 0xFFFF, 'Threshold too high.'
        
        self.write_short(r.BH1792GLC_MEAS_CONTROL4_LSB, threshold)
        #msb = threshold >> 8
        #lsb = threshold & 0xff
        #self.write_register(r.BH1792GLC_MEAS_CONTROL4_LSB, lsb)
        #self.write_register(r.BH1792GLC_MEAS_CONTROL4_MSB, msb)
