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

PCA9641_I2C_ADDRESS = 0x73

regs = (
    'id', 'contr', 'stat', 'rsrv_time', 'intr_stat', 'intr_msk', 'mb_lo', 'mb_hi')


class States:
    IDLE, GET_SLAVE_ADDR, GET_WR_REG_ADDR, REG_READS, WHICH_OP, REG_WRITES, NUM = range(7)

class Decoder(srd.Decoder):
    api_version = 3
    id = 'pca9641'
    name = 'PCA9641 (Arbiter)'
    longname = 'NXP PCA9641'
    desc = 'NXP PCA9641 I2C arbiter protocol.'
    license = 'gplv2+'
    inputs = ['i2c']
    outputs = ['i2c'] # Want  the component to NOT terminate stack!
    tags = ['Embedded/industrial', 'IC']
    arbiter_regs = (
        ('reg-id', 'ID Register'),
        ('reg-contr', 'Control Register'),
        ('reg-stat', 'Status Register'),
        ('reg-rsrv-time', 'Reserve Time Register'),
        ('reg-intr-stat', 'Interrupt Status Register'),
        ('reg-intr-mask', 'Interrupt Mask Register'),
        ('reg-mb-lo', 'Mailbox Low Register'),
        ('reg-mb-hi', 'Mailbox High Register'),
    )
    num_arbiter_regs = len(arbiter_regs)
    annotations = (('info-arb-addr', 'Tag arbiter address'), ) + arbiter_regs
    annotation_rows = (
        ('Inf', 'Inf', tuple(range(1 + num_arbiter_regs))),
    )

    def __init__(self):

        self.reset()

    def reset(self):
        self.fh = open(r'c:\Users\James\Desktop\debug_PCA9641_Arbiter.txt', r'w')
        self.fh.write("------- RESETTING -------\n")
        self.fh.write("{}\n".format(Decoder.num_arbiter_regs))
        self.fh.write("{}\n".format(Decoder.annotations))
        self.fh.write("{}\n".format(Decoder.annotation_rows))
        self.state = States.IDLE
        self.reg = 0
        self.is_auto_incr = False

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_python = self.register(srd.OUTPUT_PYTHON)

    def putx(self, data):
        self.fh.write("ANNOT: {}\n".format(data))
        self.put(self.ss, self.es, self.out_ann, data)

    def putp(self, data):
        self.put(self.ss, self.es, self.out_python, data)

    def handle_reg_0x00(self, b):
        self.putx([1, ['ID', 'ID']])

    def handle_reg_0x01(self, b):
        desc = ""
        desc += "INI|" if b & (1 << 3) else ""
        desc += "CON|" if b & (1 << 2) else ""
        desc += "GRNT|" if b & (1 << 1) else ""
        desc += "REQ|" if b & (1 << 0) else ""
        self.putx([2, ['CONTR {}'.format(desc), 'CONTR']])

    def handle_reg_0x02(self, b):
        desc = ""
        desc += "MF|" if b & (1 << 4) else ""
        desc += "ME|" if b & (1 << 3) else ""
        desc += "HUNG|" if b & (1 << 2) else ""
        desc += "INIFAIL|" if b & (1 << 1) else ""
        desc += "OLCK|" if b & (1 << 0) else ""
        self.putx([3, ['STAT {}'.format(desc), 'STAT']])

    def handle_reg_0x03(self, b):
        desc = "No time lim" if b == 0 else "Time lim {} ms".format(b)
        self.putx([4, ['RSRV {}'.format(desc), 'RSRV']])

    def handle_reg_0x04(self, b):
        desc = ""
        desc += "HUNG|" if b & (1 << 6) else ""
        desc += "MF|" if b & (1 << 5) else ""
        desc += "ME|" if b & (1 << 4) else ""
        desc += "TST|" if b & (1 << 3) else ""
        desc += "GNT|" if b & (1 << 2) else ""
        desc += "LOST|" if b & (1 << 1) else ""
        desc += "IN|" if b & (1 << 0) else ""
        self.putx([5, ['INTRST: {}'.format(desc), 'INTRS']])

    def handle_reg_0x05(self, b):
        self.putx([6, ['INTRMSK', 'INTRM']])

    def handle_reg_0x06(self, b):
        self.putx([7, ['MB LO', 'MB LO']])

    def handle_reg_0x07(self, b):
        self.putx([8, ['MB LO', 'MB HI']])

    def handle_reg(self, b):
        self.fh.write("Handle Reg 0x{:X}\n".format(self.reg))

        fn = getattr(self, 'handle_reg_0x%02x' % self.reg)
        fn(b)

        if self.is_auto_incr:
            self.fh.write("Auto inc reg from {} to {}\n".format(self.reg, self.reg + 1))
            self.reg = (self.reg + 1) % self.num_arbiter_regs


    def is_correct_chip(self, addr):
        if addr == PCA9641_I2C_ADDRESS or (addr >> 1) == PCA9641_I2C_ADDRESS:
            self.putx([0, ['PCA9641 Arbiter', 'Arb']])
            return True
        self.put(self.ss_block, self.es, self.out_ann, [28, ['Ignoring non-PCA8641 data (slave 0x%02X)' % addr]])
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

            self.reg = databyte & 0x7
            self.is_auto_incr = databyte & 0x80
            if (self.is_auto_incr): self.putx([0, ['Auto Inc:R={:X}'.format(self.reg), 'AI:{}'.format(format(self.reg))]])
            else: self.putx([0, ['No Inc:{:X}'.format(self.reg), 'NI:{}'.format(format(self.reg))]])
            
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
                if self.is_auto_incr:
                    self.state = States.REG_WRITES
                else:
                    self.state = States.IDLE
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
    print(Decoder.num_arbiter_regs)
    print(Decoder.annotations)
    print(Decoder.annotation_rows)
