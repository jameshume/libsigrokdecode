##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2013 Matt Ranostay <mranostay@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##
try:
    import sigrokdecode as srd
except:
    # Use for debug
    if __name__ == '__main__':
        class srd:
            OUTPUT_ANN = 1
            class Decoder:
                pass
    else:
        raise

PCA6408A_I2C_ADDRESSES_SHIFTED = [0x20, 0x21]

regs = (
    'id', 'contr', 'stat', 'rsrv_time', 'intr_stat', 'intr_msk', 'mb_lo', 'mb_hi')


class States:
    IDLE, GET_SLAVE_ADDR, GET_WR_REG_ADDR, REG_READS, WHICH_OP, REG_WRITES, NUM = range(7)

class Decoder(srd.Decoder):
    api_version = 3
    id = 'pca6408a'
    name = 'PCA6408A (IOExpander)'
    longname = 'NXP PCA6408A'
    desc = 'NXP PCA6408A I2C IO Expander'
    license = 'gplv2+'
    inputs = ['i2c']
    outputs = ['i2c'] # Want  the component to NOT terminate stack!
    tags = ['Embedded/industrial', 'IC']
    io_expander_regs = (
        ('reg-ip', 'Input Port Register'),
        ('reg-op', 'Output Port Register'),
        ('reg-pol', 'Polarity Inversion Register'),
        ('reg-conf', 'Configuration Register'),
        ('reg-drvs1', 'Output Drive Stength 1 Register'),
        ('reg-drvs2', 'Output Drive Strength 2 Register'),
        ('reg-ipl', 'Input Latch Register'),
        ('reg-pullena', 'Pull-up/down Enable Register'),
        ('reg-pullsel', 'Pull-up/down Selection Register'),
        ('reg-intrm', 'Intperrupt Mask Register'),
        ('reg-intrs', 'Intperrupt Status Register'),
        ('reg-outconf', 'Output Port Configuration Register'),
    )
    num_io_expander_regs = len(io_expander_regs)
    annotations = (('info-addr', 'Tag IO Expander address'), ) + io_expander_regs
    annotation_rows = (
        ('Inf', 'Inf', tuple(range(1 + num_io_expander_regs))),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.fh = open(r'c:\Users\James\Desktop\debug_PCAL6408A_IOExpander.txt', r'w')
        self.fh.write("------- RESETTING -------\n")
        self.fh.write("{}\n".format(Decoder.num_io_expander_regs))
        self.fh.write("{}\n".format(Decoder.annotations))
        self.fh.write("{}\n".format(Decoder.annotation_rows))
        self.state = States.IDLE
        self.reg = 0

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_python = self.register(srd.OUTPUT_PYTHON)

    def putx(self, data):
        self.fh.write("ANNOT: {}\n".format(data))
        self.put(self.ss, self.es, self.out_ann, data)

    def putp(self, data):
        self.put(self.ss, self.es, self.out_python, data)

    def handle_reg_0x00(self, b):
        self.putx([1, ['I/P Port', 'I/P']])

    def handle_reg_0x01(self, b):
        self.putx([2, ['O/P Port', 'O/P']])

    def handle_reg_0x02(self, b):
        self.putx([3, ['Polarity', 'Pol']])

    def handle_reg_0x03(self, b):
        desc = ""
        for idx in range(8):
            msk = 1 << idx
            if idx == 4: desc = "_" + desc
            desc = "{}".format('I' if b & msk else 'O') + desc
        self.putx([4, ['Config {}'.format(desc), 'Conf']])

    def handle_reg_0x04(self, b):
        self.putx([5, ['O/P Drive Strngth 1', 'O/P Drv 1']])

    def handle_reg_0x05(self, b):
        self.putx([6, ['O/P Drive Strngth 2', 'O/P Drv 2']])

    def handle_reg_0x06(self, b):
        self.putx([7, ['I/P Latch', 'I/P Lt']])

    def handle_reg_0x07(self, b):
        self.putx([8, ['Pull-Up/Dwn Ena', 'PUE']])

    def handle_reg_0x08(self, b):
        self.putx([9, ['Pull-Up/Dwn Sel', 'PUS']])

    def handle_reg_0x09(self, b):
        self.putx([10, ['Intr Mask', 'IM']])

    def handle_reg_0x0A(self, b):
        self.putx([11, ['Intr Status', 'IS']])

    def handle_reg_0x0B(self, b):
        self.putx([12, ['O/P Port Cfh', 'O/P Prt Cfg']])



    def handle_reg(self, b):
        self.fh.write("Handle Reg 0x{:X}\n".format(self.reg))
        fn = getattr(self, 'handle_reg_0x%02x' % self.reg)
        fn(b)
        self.reg = (self.reg + 1) % self.num_io_expander_regs


    def is_correct_chip(self, addr):
        if (
            (addr in PCA6408A_I2C_ADDRESSES_SHIFTED) or
            ((addr >> 1) in PCA6408A_I2C_ADDRESSES_SHIFTED)
        ):
            self.putx([0, ['PCA6408A IO Expander', 'IOExp']])
            return True
        self.put(self.ss_block, self.es, self.out_ann, [28, ['Ignoring non-PCA6408A data (slave 0x%02X)' % addr]])
        return False

    def decode(self, ss, es, data):
        cmd, databyte = data

        self.fh.write("DATA = {}, CMD = {}, DB = {:X}, SS = {}, ES = {}, STATE = {}\n".format(data, cmd, databyte, ss, es, self.state))

        # Store the start/end samples of this I²C packet.
        self.ss, self.es = ss, es

        # Output the data so others can stack on top of us!
        self.putp(data)

        if cmd == 'STOP':
            self.state = States.IDLE
            return

        # State machine.
        if self.state == States.IDLE:
            self.fh.write("IDLE\n")
            # Wait for an I²C START condition.
            if cmd != 'START':
                return
            self.state = States.GET_SLAVE_ADDR
            self.ss_block = ss
        
        elif self.state == States.GET_SLAVE_ADDR:
            self.fh.write("GET_SLAVE_ADDR\n")
            # Wait for an address read/write operation.
            if cmd != 'ADDRESS WRITE' and cmd != 'ADDRESS READ':
                self.fh.write("NOT WRITE or READ\n")
                self.state = States.IDLE
                return
            if not self.is_correct_chip(databyte):
                self.fh.write("NOT CORRECT CHIP\n")
                self.state = 'IDLE'
                return
            # A write is necessarily followed by a byte that sets of the address of the register
            # next be accessed. A read, however, reads the currently selected registers.
            if cmd == 'ADDRESS WRITE':
                self.state = States.GET_WR_REG_ADDR
            else:
                self.state = States.REG_READS

        elif self.state == States.GET_WR_REG_ADDR:
            self.fh.write("GET_WR_REG_ADDR\n")
            # The next command must be a data write (master selects the slave register via the
            # command code register).
            if cmd == 'ACK':
                return
            elif cmd != 'DATA WRITE':
                self.fh.write("\tUNEXPECTED CMD!\n")
                self.state = States.IDLE
                return

            self.reg = databyte

            # Now there will either be a repeated start and then a set of reads or there will be
            # more data written
            # Buuuut... there might also be a stop-then-start, into an address read and then a set
            # of reads. Sigh.
            self.state = States.WHICH_OP

        elif self.state == States.WHICH_OP:
            self.fh.write("WHICH_OP\n")
            if cmd == 'START REPEAT':
                cmd = States.GET_SLAVE_ADDR
            elif cmd == 'DATA WRITE':
                self.handle_reg(databyte)
            elif cmd == 'STOP':
                self.state = States.IDLE
            elif cmd == 'ACK':
                return
            else:
                self.state = States.ERROR

        elif self.state == States.REG_READS:
            self.fh.write("REG_READS\n")
            if cmd == 'ACK':
                return
            elif cmd == 'NACK':
                self.state = States.IDLE # NACK should mean next cmd is STOP so just IDLE
            elif cmd == 'DATA READ':
                self.handle_reg(databyte)
            else:
                self.fh.write("UNEXPECTED CMD\n")
                self.state = States.IDLE #
                return

        elif self.state == States.REG_WRITES:
            self.fh.write("REG_WRITES\n")
            if cmd == 'DATA WRITE':
                self.handle_reg(databyte)
            elif cmd == 'STOP':
                self.state = States.IDLE
            else:
                self.state = States.ERROR

if __name__ == '__main__':
    print(Decoder.annotations)
    print(Decoder.annotation_rows)
