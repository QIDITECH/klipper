# Support for reading acceleration data from an LIS2DW chip
#
# Copyright (C) 2023  Zhou.XianMing <zhouxm@biqu3d.com>
# Copyright (C) 2020-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus, adxl345, bulk_sensor

# LIS2DW registers
REG_LIS2DW_WHO_AM_I_ADDR = 0x0F
REG_LIS2DW_CTRL_REG1_ADDR = 0x20
REG_LIS2DW_CTRL_REG2_ADDR = 0x21
REG_LIS2DW_CTRL_REG3_ADDR = 0x22
REG_LIS2DW_CTRL_REG6_ADDR = 0x25
REG_LIS2DW_STATUS_REG_ADDR = 0x27
REG_LIS2DW_OUT_XL_ADDR = 0x28
REG_LIS2DW_OUT_XH_ADDR = 0x29
REG_LIS2DW_OUT_YL_ADDR = 0x2A
REG_LIS2DW_OUT_YH_ADDR = 0x2B
REG_LIS2DW_OUT_ZL_ADDR = 0x2C
REG_LIS2DW_OUT_ZH_ADDR = 0x2D
REG_LIS2DW_FIFO_CTRL   = 0x2E
REG_LIS2DW_FIFO_SAMPLES = 0x2F
REG_MOD_READ = 0x80
# REG_MOD_MULTI = 0x40

LIS2DW_DEV_ID = 0x44

FREEFALL_ACCEL = 9.80665
SCALE = FREEFALL_ACCEL * 1.952 / 4

MIN_MSG_TIME = 0.100

BYTES_PER_SAMPLE = 6
SAMPLES_PER_BLOCK = 8

BATCH_UPDATES = 0.100

# Printer class that controls LIS2DW chip
class LIS2DW:
    def __init__(self, config):
        self.printer = config.get_printer()
        adxl345.AccelCommandHelper(config, self)
        self.axes_map = adxl345.read_axes_map(config)
        self.data_rate = 1600
        # Setup mcu sensor_lis2dw bulk query code
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=5000000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_lis2dw_cmd = self.query_lis2dw_end_cmd = None
        self.query_lis2dw_status_cmd = None
        mcu.add_config_cmd("config_lis2dw oid=%d spi_oid=%d"
                           % (oid, self.spi.get_oid()))
        mcu.add_config_cmd("query_lis2dw oid=%d clock=0 rest_ticks=0"
                           % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, "lis2dw_data", oid)
        # Clock tracking
        chip_smooth = self.data_rate * BATCH_UPDATES * 2
        self.clock_sync = bulk_sensor.ClockSyncRegression(mcu, chip_smooth)
        self.clock_updater = bulk_sensor.ChipClockUpdater(self.clock_sync,
                                                          BYTES_PER_SAMPLE)
        self.last_error_count = 0
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._finish_measurements, BATCH_UPDATES)
        self.name = config.get_name().split()[-1]
        hdr = ('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')
        self.batch_bulk.add_mux_endpoint("lis2dw/dump_lis2dw", "sensor",
                                         self.name, {'header': hdr})

    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.query_lis2dw_cmd = self.mcu.lookup_command(
            "query_lis2dw oid=%c clock=%u rest_ticks=%u", cq=cmdqueue)
        self.query_lis2dw_end_cmd = self.mcu.lookup_query_command(
            "query_lis2dw oid=%c clock=%u rest_ticks=%u",
            "lis2dw_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%c limit_count=%hu", oid=self.oid, cq=cmdqueue)
        self.query_lis2dw_status_cmd = self.mcu.lookup_query_command(
            "query_lis2dw_status oid=%c",
            "lis2dw_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%c limit_count=%hu", oid=self.oid, cq=cmdqueue)
    def read_reg(self, reg):
        params = self.spi.spi_transfer([reg | REG_MOD_READ, 0x00])
        response = bytearray(params['response'])
        return response[1]
    def set_reg(self, reg, val, minclock=0):
        self.spi.spi_send([reg, val & 0xFF], minclock=minclock)
        stored_val = self.read_reg(reg)
        if stored_val != val:
            raise self.printer.command_error(
                    "Failed to set LIS2DW register [0x%x] to 0x%x: got 0x%x. "
                    "This is generally indicative of connection problems "
                    "(e.g. faulty wiring) or a faulty lis2dw chip." % (
                        reg, val, stored_val))
    def start_internal_client(self):
        aqh = adxl345.AccelQueryHelper(self.printer)
        self.batch_bulk.add_client(aqh.handle_batch)
        return aqh
    # Measurement decoding
    def _extract_samples(self, raw_samples):
        # Load variables to optimize inner loop below
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self.axes_map
        last_sequence = self.clock_updater.get_last_sequence()
        time_base, chip_base, inv_freq = self.clock_sync.get_time_translation()
        # Process every message in raw_samples
        count = seq = 0
        samples = [None] * (len(raw_samples) * SAMPLES_PER_BLOCK)
        for params in raw_samples:
            seq_diff = (params['sequence'] - last_sequence) & 0xffff
            seq_diff -= (seq_diff & 0x8000) << 1
            seq = last_sequence + seq_diff
            d = bytearray(params['data'])
            msg_cdiff = seq * SAMPLES_PER_BLOCK - chip_base

            for i in range(len(d) // BYTES_PER_SAMPLE):
                d_xyz = d[i*BYTES_PER_SAMPLE:(i+1)*BYTES_PER_SAMPLE]
                xlow, xhigh, ylow, yhigh, zlow, zhigh = d_xyz
                # Merge and perform twos-complement

                rx = (((xhigh << 8) | xlow)) - ((xhigh & 0x80) << 9)
                ry = (((yhigh << 8) | ylow)) - ((yhigh & 0x80) << 9)
                rz = (((zhigh << 8) | zlow)) - ((zhigh & 0x80) << 9)

                raw_xyz = (rx, ry, rz)

                x = round(raw_xyz[x_pos] * x_scale, 6)
                y = round(raw_xyz[y_pos] * y_scale, 6)
                z = round(raw_xyz[z_pos] * z_scale, 6)

                ptime = round(time_base + (msg_cdiff + i) * inv_freq, 6)
                samples[count] = (ptime, x, y, z)
                count += 1
        self.clock_sync.set_last_chip_clock(seq * SAMPLES_PER_BLOCK + i)
        del samples[count:]
        return samples
    def _update_clock(self, minclock=0):
        params = self.query_lis2dw_status_cmd.send([self.oid],
                                                   minclock=minclock)
        self.clock_updater.update_clock(params)
    # Start, stop, and process message batches
    def _start_measurements(self):
        # In case of miswiring, testing LIS2DW device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        dev_id = self.read_reg(REG_LIS2DW_WHO_AM_I_ADDR)
        logging.info("lis2dw_dev_id: %x", dev_id)
        if dev_id != LIS2DW_DEV_ID:
            raise self.printer.command_error(
                "Invalid lis2dw id (got %x vs %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty lis2dw chip."
                % (dev_id, LIS2DW_DEV_ID))
        # Setup chip in requested query rate
        # ODR/2, +-16g, low-pass filter, Low-noise abled
        self.set_reg(REG_LIS2DW_CTRL_REG6_ADDR, 0x34)
        # Continuous mode: If the FIFO is full
        # the new sample overwrites the older sample.
        self.set_reg(REG_LIS2DW_FIFO_CTRL, 0xC0)
        # High-Performance / Low-Power mode 1600/200 Hz
        # High-Performance Mode (14-bit resolution)
        self.set_reg(REG_LIS2DW_CTRL_REG1_ADDR, 0x94)

        # Start bulk reading
        self.bulk_queue.clear_samples()
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        reqclock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.seconds_to_clock(4. / self.data_rate)
        self.query_lis2dw_cmd.send([self.oid, reqclock, rest_ticks],
                                    reqclock=reqclock)
        logging.info("LIS2DW starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.clock_updater.note_start(reqclock)
        self._update_clock(minclock=reqclock)
        self.clock_updater.clear_duration_filter()
        self.last_error_count = 0
    def _finish_measurements(self):
        # Halt bulk reading
        params = self.query_lis2dw_end_cmd.send([self.oid, 0, 0])
        self.bulk_queue.clear_samples()
        logging.info("LIS2DW finished '%s' measurements", self.name)
        self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x00)
    def _process_batch(self, eventtime):
        self._update_clock()
        raw_samples = self.bulk_queue.pull_samples()
        if not raw_samples:
            return {}
        samples = self._extract_samples(raw_samples)
        if not samples:
            return {}
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.clock_updater.get_last_limit_count()}

def load_config(config):
    return LIS2DW(config)

def load_config_prefix(config):
    return LIS2DW(config)
