"""
    Driver for BH1792GLC Pulse Sensor
"""
import struct
from bh1792glc import bh1792glc_registers
from bh1792glc.i2c_interface import *
import logging

r = bh1792glc_registers.registers()
b = bh1792glc_registers.bits()
m = bh1792glc_registers.masks()
e = bh1792glc_registers.enums()


class BH1792GLCDriver(SensorDriver):

    def __init__(self):
        SensorDriver.__init__(self, 0x5B, 1)
        self.name = 'BH1792GLC'
        self._registers = dict(r.__dict__)
        self._dump_range = (r.BH1792GLC_REGISTER_DUMP_START,
                            r.BH1792GLC_REGISTER_DUMP_END)
        logging.basicConfig(level=logging.DEBUG)

    def probe(self):
        self.connected = True
        resp = self.read_register(r.BH1792GLC_PART_ID)
        if resp == b.BH1792GLC_PART_ID_PART_ID_ID:
            logging.info('detected BH1792GLC')
            return 1
        self.connected = False
        return 0

    def por(self):
        self.write_register(r.BH1792GLC_RESET, b.BH1792GLC_RESET_SWRESET)

    def ic_test(self):
        pass

    def set_sync_measurement(self,
                             mode=b.BH1792GLC_MEAS_CONTROL1_MSR_64HZ,
                             current=10):
        self.set_power_off()
        self.set_meas_mode(mode, b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_GREEN)
        self.set_led1_current(current)
        self.set_interrupt_mode(b.BH1792GLC_MEAS_CONTROL5_INT_SEL_FIFO_WATERMARK)
        self.start_measurement()
        self.sync_measurement()

    def set_single_measurement(self,
                               adc=b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_GREEN,
                               current=10):
        self.set_power_off()
        self.set_meas_mode(b.BH1792GLC_MEAS_CONTROL1_MSR_SINGLE_MEAS_MODE, adc)
        self.set_led1_current(current)
        self.set_led2_current(current)
        self.set_interrupt_mode(b.BH1792GLC_MEAS_CONTROL5_INT_SEL_ON_COMPLETE)

    def set_nonsync_measurement(self, current=40, threshold=0xffff):
        self.set_power_off()
        self.set_meas_mode(b.BH1792GLC_MEAS_CONTROL1_MSR_NON_SYNCH_MODE,
                           b.BH1792GLC_MEAS_CONTROL1_SEL_ADC_IR)
        self.set_led2_current(current)
        self.set_interrupt_mode(b.BH1792GLC_MEAS_CONTROL5_INT_SEL_IR_THRESHOLD)
        self.set_ir_threshold(threshold)

    def get_fifo_level(self):
        return self.read_register(r.BH1792GLC_FIFO_LEV, 1)[0]

    def _read_data(self, channel=None):
        data = self.read_register(r.BH1792GLC_IRDATA_LEDOFF_LSB, 9)
        return struct.unpack('HHHHB', data)

    def read_gdata(self):
        data = self.read_register(r.BH1792GLC_GDATA_LEDOFF_LSB, 4)
        return struct.unpack('HH', data)

    def read_fifo(self):
        data = self.read_register(r.BH1792GLC_FIFODATA0_LSB, 4)
        return struct.unpack('HH', data)

    def read_irdata(self):
        data = self.read_register(r.BH1792GLC_IRDATA_LEDOFF_LSB, 4)
        return struct.unpack('HH', data)

    def reset_drdy_pin(self):
        self.read_register(r.BH1792GLC_INT_CLEAR)

    def read_drdy(self):
        raise NotImplementedError("bh1792glc doesn't support reading DRDY")

    def start_measurement(self):
        self.write_register(r.BH1792GLC_MEAS_START, b.BH1792GLC_MEAS_START_MEAS_ST)

    def sync_measurement(self):
        self.write_register(r.BH1792GLC_MEAS_SYNC, b.BH1792GLC_MEAS_SYNC_MEAS_SYNC)

    def stop_measurement(self):
        self.set_power_off()

    def set_led1_current(self, current):
        assert current <= 0x63, 'Current too high'
        self.set_bit_pattern(r.BH1792GLC_MEAS_CONTROL2,
                             current, m.BH1792GLC_MEAS_CONTROL2_LED_CURRENT1_MASK)

    def set_led2_current(self, current):
        assert current <= 0x63, 'Current too high'
        self.set_bit_pattern(r.BH1792GLC_MEAS_CONTROL3,
                             current, m.BH1792GLC_MEAS_CONTROL3_LED_CURRENT2_MASK)

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
        msb = threshold >> 8
        lsb = threshold & 0xff
        self.write_register(r.BH1792GLC_MEAS_CONTROL4_LSB, lsb)
        self.write_register(r.BH1792GLC_MEAS_CONTROL4_MSB, msb)
