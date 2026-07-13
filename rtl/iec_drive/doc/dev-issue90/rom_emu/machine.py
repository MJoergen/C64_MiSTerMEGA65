# 1581 machine model mirroring the C64MEGA65 phys-mode RTL semantics.
import sys
from cpu6502 import CPU

ROM = open(sys.argv[1] if len(sys.argv)>1 else 'c1581.rom','rb').read()

class Machine:
    def __init__(self, log_io=False):
        self.ram = bytearray(0x2000)
        self.log_io = log_io
        self.trace = []            # (what, detail) tuples
        self.cyc = 0
        # ---- physical drive model (mirrors controller + mechanism) ----
        self.head = 3              # real head cylinder at power-on
        self.disk_in = True
        self.chg_latch = True      # mechanism DSKCHG latch: set until step w/ disk
        self.motor_on_at = None
        self.ids_per_rev = 11      # 10 sectors + MEGA65 TIB (R=11)
        self.rot = 0               # rotation position (ID index)
        # ---- WD registers (mirror fdc1772 incl. our fixes) ----
        self.wd_track = 0; self.wd_sector = 0; self.wd_data_in = 0; self.wd_data_out = 0
        self.wd_status = 0; self.busy_until = 0
        self.drq_bytes = []        # bytes pending delivery via data reg
        self.cmd = 0; self.rnf=0; self.crc=0; self.deleted=0
        self.step_to = 0
        self.pending = None        # deferred completion action
        # ---- CIA ----
        self.cia = bytearray(16)
        self.cia_pra_out = 0xFF; self.cia_prb_out = 0xFF
        self.cia_ddra = 0; self.cia_ddrb = 0
        self.cia_tod = 0
        self.cia_ta = 0xFFFF; self.cia_ta_latch=0xFFFF; self.cia_ta_run=0
        self.cia_tb = 0xFFFF; self.cia_tb_latch=0xFFFF; self.cia_tb_run=0
        self.cia_icr_mask = 0; self.cia_icr = 0
        # ---- VIA ----
        self.via = bytearray(16)
        self.via_t1 = 0xFFFF; self.via_t1_latch = 0xFFFF
        self.via_ifr = 0; self.via_ier = 0
        self.cpu = CPU(self.read, self.write)
        # disk image: minimal valid 1581 filesystem
        self.disk = {}
        self.build_disk()
    # ------------- disk content -------------
    def logical(self, t, s):  # logical T/S -> 256 bytes
        b = bytearray(256)
        if t==40 and s==0:
            b[0]=40; b[1]=3; b[2]=0x44; b[3]=0
            name=b'EMU DISK'
            for i in range(16): b[4+i]=0xA0
            b[4:4+len(name)]=name
            b[0x14]=0xA0; b[0x15]=0xA0
            b[0x16]=ord('4'); b[0x17]=ord('2'); b[0x18]=0xA0
            b[0x19]=ord('3'); b[0x1A]=ord('D'); b[0x1B]=0xA0; b[0x1C]=0xA0
        elif t==40 and s==1:
            b[0]=40; b[1]=2; b[2]=0x44; b[3]=0xBB; b[4]=ord('4'); b[5]=ord('2'); b[6]=0xC0
            for i in range(7,256): b[i]=0xFF   # all free-ish
        elif t==40 and s==2:
            b[0]=0; b[1]=0xFF; b[2]=0x44; b[3]=0xBB; b[4]=ord('4'); b[5]=ord('2'); b[6]=0xC0
            for i in range(7,256): b[i]=0xFF
        elif t==40 and s==3:
            b[0]=0; b[1]=0xFF                  # empty directory block
        return b
    def build_disk(self):
        for t in range(1,81):
            for s in range(40):
                cyl=t-1; side=0 if s<20 else 1
                pr=1+(s%20)//2; half=s&1
                key=(cyl,side,pr)
                if key not in self.disk: self.disk[key]=bytearray(512)
                self.disk[key][half*256:(half+1)*256]=self.logical(t,s)
    # ------------- helpers -------------
    def motor(self):  # CIA PA2 low = on
        return (self.cia_pra_eff()>>2)&1 == 0
    def side(self):   # CIA PA0
        return (self.cia_pra_eff())&1
    def cia_pra_eff(self):
        # output pins: driven where ddra=1, else pulled 1
        return (self.cia_pra_out | ~self.cia_ddra) & 0xFF
    def ready(self):
        # our media_ready: motor + 2 index edges (~2/5 rev incl. ramp): model 60k cyc
        return self.motor_on_time() > 60000
    def motor_on_time(self):
        if not self.motor(): self.motor_on_at=None; return 0
        if self.motor_on_at is None: self.motor_on_at=self.cyc
        return self.cyc - self.motor_on_at
    def do_step(self, outward):
        if outward: self.head=max(0,self.head-1)
        else: self.head=min(84,self.head+1)
        if self.disk_in: self.chg_latch=False
        self.trace.append(('STEP','out' if outward else 'in', self.head))
    # ------------- WD model -------------
    def wd_busy(self): return 1 if (self.cyc < self.busy_until or self.pending or self.drq_bytes) else 0
    def wd_finish(self):
        if self.pending and self.cyc >= self.busy_until:
            act = self.pending; self.pending=None
            act()
    def wd_read(self, r):
        self.wd_finish()
        if r==0:
            t1 = (self.cmd & 0x80)==0
            b7 = 0x80 if self.motor() else 0
            b6 = 0
            b5 = 0x20 if t1 and self.motor_on_time()>200000 else (0x20 if self.deleted and (self.cmd&0xE0)==0x80 else 0)
            b4 = 0x10 if self.rnf else 0
            b3 = 0x08 if self.crc else 0
            b2 = (0x04 if self.head==0 else 0) if t1 else 0
            b1 = (0x02 if True else 0) if t1 else (0x02 if self.drq_bytes else 0)
            b0 = self.wd_busy()
            return b7|b6|b5|b4|b3|b2|b1|b0
        if r==1: return self.wd_track
        if r==2: return self.wd_sector
        if r==3:
            if self.drq_bytes:
                self.wd_data_out = self.drq_bytes.pop(0)
                if not self.drq_bytes: self.busy_until = self.cyc  # done after last byte
            return self.wd_data_out
        return 0xFF
    def wd_write(self, r, v):
        self.wd_finish()
        if r==0: self.wd_command(v)
        elif r==1: self.wd_track=v
        elif r==2: self.wd_sector=v
        elif r==3: self.wd_data_in=v; self.wd_data_out=v   # our readback fix
    def wd_command(self, v):
        self.cmd=v
        self.trace.append(('CMD',v,self.wd_track,self.wd_sector,self.wd_data_in,self.head,'pc=%04X'%self.cpu.pc))
        self.rnf=0; self.crc=0; self.deleted=0
        top=v>>4
        if top==0x0:      # RESTORE
            self.step_to=0; self.wd_track=0xFF
            n=self.head
            for _ in range(n): self.do_step(True)
            self.wd_track=0
            self.busy_until=self.cyc+ max(1,n)*3000
        elif top==0x1:    # SEEK
            self.step_to=self.wd_data_in
            n=abs(self.wd_track-self.step_to)
            outward = self.step_to < self.wd_track
            for _ in range(n): self.do_step(outward)
            self.wd_track=self.step_to
            self.busy_until=self.cyc+max(1,n)*3000
        elif top in (0x2,0x3): # STEP (repeat last dir): model as inward
            self.do_step(False); self.busy_until=self.cyc+3000
            if v&0x10: self.wd_track=(self.wd_track+1)&0xFF
        elif top in (0x4,0x5): # STEP-IN
            self.do_step(False); self.busy_until=self.cyc+3000
            if v&0x10: self.wd_track=(self.wd_track+1)&0xFF
        elif top in (0x6,0x7): # STEP-OUT
            self.do_step(True); self.busy_until=self.cyc+3000
            if v&0x10: self.wd_track=(self.wd_track-1)&0xFF
        elif top in (0x8,0x9): # READ SECTOR
            if not self.ready():
                self.rnf=1; self.busy_until=self.cyc+2000; return
            key=(self.head, 1-self.side() if False else (0 if self.side()==0 else 1), self.wd_sector)
            # our chain: PA0=0 -> logical side0 -> head0. side() = PA0. sidesel = 0 when PA0=0.
            key=(self.head, self.side()^0, self.wd_sector)
            # match like our controller: decoded C == wd_track, R == wd_sector
            if self.wd_track==self.head and (self.head, self.pa0_to_physside(), self.wd_sector) in self.disk:
                data=self.disk[(self.head,self.pa0_to_physside(),self.wd_sector)]
                self.drq_bytes=list(data)
                self.busy_until=self.cyc+999999999   # until drained
            else:
                self.rnf=1; self.busy_until=self.cyc+40000
        elif top==0xC:    # READ ADDRESS
            if not self.ready():
                self.rnf=1; self.busy_until=self.cyc+2000; return
            self.rot=(self.rot+1)%self.ids_per_rev
            rr=self.rot+1
            c=self.head; h=self.pa0_to_physside(); n=2
            self.drq_bytes=[c,h,rr,n,0xAB,0xCD]
            def fin(): self.wd_sector=c
            self.wd_sector=c   # our phys_set_sector at completion; simplify: now
            self.trace.append(('RA',c,h,rr))
            self.busy_until=self.cyc+3000
        elif top==0xD:    # FORCE INTERRUPT
            self.drq_bytes=[]; self.pending=None; self.busy_until=self.cyc
        elif top in (0xA,0xB): # WRITE SECTOR (blocked read-only)
            self.busy_until=self.cyc+2000
        elif top in (0xE,0xF): # READ/WRITE TRACK: finish
            self.busy_until=self.cyc+2000
    def pa0_to_physside(self):
        # PA0=0 (DOS logical side 0) -> f_side1 high -> head 0 (our FIXED mapping)
        return 0 if self.side()==0 else 1
    # ------------- CIA -------------
    def cia_read(self, r):
        if r==0:
            pa_in = 0xFF
            if self.chg_latch: pa_in &= ~0x80
            if self.ready():   pa_in &= ~0x02
            pa_in &= ~0x18     # drive number 00 -> bits 3,4 low
            return pa_in & self.cia_pra_eff()
        if r==1:
            prb_in = 0xFF & ~0x80  # ~ATN: ATN idle -> bit7=0
            prb_in &= ~0x04        # ~CLK idle
            prb_in &= ~0x01        # ~DATA idle
            # bit6 wps_n: not protected -> 1
            return prb_in & ((self.cia_prb_out | ~self.cia_ddrb)&0xFF)
        if r==2: return self.cia_ddra
        if r==3: return self.cia_ddrb
        if r==4: return self.cia_ta & 0xFF
        if r==5: return (self.cia_ta>>8)&0xFF
        if r==6: return self.cia_tb & 0xFF
        if r==7: return (self.cia_tb>>8)&0xFF
        if r==8: return self.cia_tod & 0xFF
        if r==9: return (self.cia_tod>>8)&0xFF
        if r==10: return (self.cia_tod>>16)&0xFF
        if r==13:
            v=self.cia_icr; self.cia_icr=0; return v
        return self.cia[r]
    def cia_write(self, r, v):
        if r==0: self.cia_pra_out=v
        elif r==1: self.cia_prb_out=v
        elif r==2: self.cia_ddra=v
        elif r==3: self.cia_ddrb=v
        elif r==4: self.cia_ta_latch=(self.cia_ta_latch&0xFF00)|v
        elif r==5: self.cia_ta_latch=(self.cia_ta_latch&0xFF)|(v<<8)
        elif r==6: self.cia_tb_latch=(self.cia_tb_latch&0xFF00)|v
        elif r==7: self.cia_tb_latch=(self.cia_tb_latch&0xFF)|(v<<8)
        elif r==8: self.cia_tod=(self.cia_tod&0xFFFF00)|v
        elif r==13:
            if v&0x80: self.cia_icr_mask|= v&0x7F
            else: self.cia_icr_mask &= ~(v&0x7F)
        elif r==14:
            self.cia[14]=v
            if v&0x10: self.cia_ta=self.cia_ta_latch
            self.cia_ta_run=v&1
        elif r==15:
            self.cia[15]=v
            if v&0x10: self.cia_tb=self.cia_tb_latch
            self.cia_tb_run=v&1
        else: self.cia[r]=v
    # ------------- VIA -------------
    def via_read(self, r):
        if r==4:
            self.via_ifr &= ~0x40
            return self.via_t1 & 0xFF
        if r==5: return (self.via_t1>>8)&0xFF
        if r==13: return self.via_ifr | (0x80 if (self.via_ifr & self.via_ier & 0x7F) else 0)
        if r==14: return self.via_ier|0x80
        if r in (0,15): return 0xFF
        if r==1: return 0xFF
        return self.via[r]
    def via_write(self, r, v):
        if r==4 or r==6: self.via_t1_latch=(self.via_t1_latch&0xFF00)|v
        elif r==5:
            self.via_t1_latch=(self.via_t1_latch&0xFF)|(v<<8)
            self.via_t1=self.via_t1_latch; self.via_ifr&=~0x40
        elif r==7: self.via_t1_latch=(self.via_t1_latch&0xFF)|(v<<8)
        elif r==13: self.via_ifr &= ~(v&0x7F)
        elif r==14:
            if v&0x80: self.via_ier |= v&0x7F
            else: self.via_ier &= ~(v&0x7F)
        else: self.via[r]=v
    # ------------- bus -------------
    def read(self, a):
        a&=0xFFFF
        if a<0x2000: return self.ram[a]
        if a<0x4000: return self.via_read(a&15)
        if a<0x6000: return self.cia_read(a&15)
        if a<0x8000: return self.wd_read(a&3)
        return ROM[a-0x8000]
    def write(self, a, v):
        a&=0xFFFF; v&=0xFF
        if a<0x2000: self.ram[a]=v; return
        if a<0x4000: self.via_write(a&15, v); return
        if a<0x6000: self.cia_write(a&15, v); return
        if a<0x8000: self.wd_write(a&3, v); return
    # ------------- run -------------
    def tick_devices(self, n):
        self.cyc += n
        self.cia_tod += n
        if self.cia_ta_run:
            self.cia_ta -= n
            if self.cia_ta <= 0:
                self.cia_ta += self.cia_ta_latch or 0x10000
                self.cia_icr |= 1
        if self.cia_tb_run:
            self.cia_tb -= n
            if self.cia_tb <= 0:
                self.cia_tb += self.cia_tb_latch or 0x10000
                self.cia_icr |= 2
        self.via_t1 -= n
        if self.via_t1 <= 0:
            self.via_t1 += (self.via_t1_latch or 0x10000)
            self.via_ifr |= 0x40
        irq = False
        if (self.cia_icr & self.cia_icr_mask): irq=True
        if (self.via_ifr & self.via_ier & 0x7F): irq=True
        self.cpu.irq_line = irq
    def run(self, steps):
        for _ in range(steps):
            self.cpu.step()
            self.tick_devices(3)
