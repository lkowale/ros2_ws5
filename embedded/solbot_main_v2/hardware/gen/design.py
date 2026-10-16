"""solbot_main v2 (compact) — single source of truth for the board netlist.

Run `python3 build.py` to regenerate the KiCad files.

Compared with solbot_main (v1, full power path) this board has no on-board battery
switching. Rev B uses JLCPCB *basic* parts wherever possible (LCSC field on every part):
  * 12 V input -> 2x AMS1117-5.0 (logic / ESP32) -> XC6206 3.3 V (no Extended buck+inductor),
  * main battery bus voltage + 12 V monitored on the ESP32-S3 ADC1,
  * two external DC solid-state relays (DELIXI CDG1-1DD/25A) for PV on/off,
  * 4 IBT-2 ports on 2x5 IDC (v3 pinout, pins 5/6 = R_IS/L_IS used), 74HC14 level shift,
    E-stop gates the IBT-2 R_EN/L_EN pins,
  * 4 position pots read directly by ADC1 (no ADS1115), BNO085 on I2C.
Through-hole parts (connectors, DevKit sockets) are hand-soldered: not in the JLC BOM.
"""

R0603 = 'Resistor_SMD:R_0603_1608Metric'
R0805 = 'Resistor_SMD:R_0805_2012Metric'
LED0805 = 'LED_SMD:LED_0805_2012Metric'
R1206 = 'Resistor_SMD:R_1206_3216Metric'
C0603 = 'Capacitor_SMD:C_0603_1608Metric'
C0805 = 'Capacitor_SMD:C_0805_2012Metric'
C1206 = 'Capacitor_SMD:C_1206_3216Metric'
LED0603 = 'LED_SMD:LED_0603_1608Metric'
SOT23 = 'Package_TO_SOT_SMD:SOT-23'
PT3 = 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-3-3.5-H_1x03_P3.50mm_Horizontal'
PT2 = 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-2-3.5-H_1x02_P3.50mm_Horizontal'
MKDS2 = 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal'
IDC10 = 'Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical'

SHEETS = [
    ('power', '12 V input, 5 V / 3.3 V supplies, bus voltage monitor, SSR drivers'),
    ('mcu', 'ESP32-S3, IMU, E-stop'),
    ('actuators', 'Linear actuators: 4x IBT-2 (IDC 2x5), position & current sense'),
]


class Part:
    def __init__(self, prefix, lib_id, value, footprint, conns, sheet, block,
                 nc=(), dnp=False, fields=None):
        self.prefix = prefix
        self.lib_id = lib_id
        self.value = value
        self.footprint = footprint
        self.conns = {str(k): v for k, v in conns.items()}
        self.nc = [str(x) for x in nc]
        self.sheet = sheet
        self.block = block
        self.dnp = dnp
        self.fields = fields or {}
        self.ref = None


class Design:
    def __init__(self):
        self.parts = []
        self.sheet = None
        self.block = None
        self.block_notes = {}

    def blk(self, sheet, block, note=''):
        self.sheet, self.block = sheet, block
        if note:
            self.block_notes[(sheet, block)] = note

    def add(self, prefix, lib_id, value, fp, conns, **kw):
        p = Part(prefix, lib_id, value, fp, conns, self.sheet, self.block, **kw)
        self.parts.append(p)
        return p

    def R(self, val, a, b, fp=R0603, **kw):
        kw.setdefault('fields', {'LCSC': LCSC_R[(val, fp)]})
        return self.add('R', 'Device:R', val, fp, {1: a, 2: b}, **kw)

    def C(self, val, a, b, fp=C0603, **kw):
        kw.setdefault('fields', {'LCSC': LCSC_C[(val, fp)]})
        return self.add('C', 'Device:C', val, fp, {1: a, 2: b}, **kw)

    def CP(self, val, pos, neg, fp, **kw):
        return self.add('C', 'Device:C_Polarized', val, fp, {1: pos, 2: neg}, **kw)

    def assign_refs(self):
        counters = {}
        for p in self.parts:
            counters[p.prefix] = counters.get(p.prefix, 0) + 1
            p.ref = '%s%d' % (p.prefix, counters[p.prefix])


GND = 'GND'
V12 = '+12V'
V5 = '+5V'
V5E = '5V_ESP'
V3A = '+3V3A'

# JLCPCB parts (checked 2026-10-16 via jlcpcb.com parts search). B = Basic, P = Preferred Extended.
LCSC_R = {
    ('22', R0603): 'C23345', ('100', R0603): 'C22775', ('470', R0603): 'C23179', ('1k', R0603): 'C21190',
    ('2.2k', R0603): 'C4190', ('4.7k', R0603): 'C23162', ('10k', R0603): 'C25804', ('30k', R0603): 'C22984',
    ('100k', R0603): 'C25803', ('1M', R0603): 'C22935', ('470k 150V', R0805): 'C17709',
    ('22R 0.25W', R1206): 'C17958',
}
LCSC_C = {
    ('100n', C0603): 'C14663', ('1u', C0603): 'C15849', ('10u 25V', C0805): 'C15850',
    ('22u 25V', C0805): 'C45783', ('10u 50V', C1206): 'C13585',
}
HAND = {'Assembly': 'hand-solder (THT)'}   # excluded from the JLC BOM / CPL


def build():
    d = Design()

    # ===================== POWER ===========================================
    d.blk('power', 'IN12', '12 V input from the robot 12 V rail (fuse it externally, <= 2 A): '
          'SS34 reverse-polarity diode, SMBJ18A TVS, 2x10 uF.')
    d.add('J', 'Connector_Generic:Conn_01x02', '12V IN', MKDS2, {1: '12V_RAW', 2: GND}, fields=HAND)
    d.add('D', 'Device:D_Schottky', 'SS34', 'Diode_SMD:D_SMA', {'A': '12V_RAW', 'K': V12}, fields={'LCSC': 'C8678'})
    d.add('D', 'Device:D_Zener', 'SMBJ18A', 'Diode_SMD:D_SMB', {'K': V12, 'A': GND}, fields={'LCSC': 'C19077573'})
    d.C('10u 50V', V12, GND, fp=C1206)
    d.C('10u 50V', V12, GND, fp=C1206)

    d.blk('power', 'LDO5', 'Two AMS1117-5.0 linear regulators (Basic; v3 used 2x 7805). '
          'U_ESP feeds the DevKit through a 2x22R pre-drop (shares the heat: ~0.25 W in the '
          'resistors, ~0.7 W in the LDO at 150 mA) and SS34 (USB back-feed). U_LOG feeds +5V: '
          'IBT-2 logic, 74HC14, IMU, 3.3 V LDO. Give both SOT-223 tabs copper.')
    d.R('22R 0.25W', V12, '12V_DROP', fp=R1206)
    d.R('22R 0.25W', V12, '12V_DROP', fp=R1206)
    d.C('10u 25V', '12V_DROP', GND, fp=C0805)
    d.add('U', 'Regulator_Linear:AMS1117-5.0', 'AMS1117-5.0', 'Package_TO_SOT_SMD:SOT-223-3_TabPin2',
          {'VI': '12V_DROP', 'VO': '5V_ESP_LDO', 'GND': GND}, fields={'LCSC': 'C6187'})
    d.C('22u 25V', '5V_ESP_LDO', GND, fp=C0805)
    d.add('D', 'Device:D_Schottky', 'SS34', 'Diode_SMD:D_SMA',
          {'A': '5V_ESP_LDO', 'K': V5E}, fields={'LCSC': 'C8678'})
    d.C('10u 25V', V5E, GND, fp=C0805)
    d.C('10u 25V', V12, GND, fp=C0805)
    d.add('U', 'Regulator_Linear:AMS1117-5.0', 'AMS1117-5.0', 'Package_TO_SOT_SMD:SOT-223-3_TabPin2',
          {'VI': V12, 'VO': V5, 'GND': GND}, fields={'LCSC': 'C6187'})
    d.C('22u 25V', V5, GND, fp=C0805)
    d.R('2.2k', V5, 'LED_5V')
    d.add('D', 'Device:LED', 'KT-0805G green', LED0805, {'A': 'LED_5V', 'K': GND}, fields={'LCSC': 'C2297'})

    d.blk('power', 'LDO3', 'XC6206P332MR 3.3 V (200 mA): I2C pull-ups, pot excitation, clamps.')
    d.C('1u', V5, GND)
    d.add('U', 'Regulator_Linear:XC6206PxxxMR', 'XC6206P332MR-G', 'Package_TO_SOT_SMD:SOT-23-3',
          {'VI': V5, 'VO': V3A, 'GND': GND}, fields={'LCSC': 'C5446'})
    d.C('1u', V3A, GND)

    d.blk('power', 'VMON', 'Main battery bus monitor: (470k+470k+30k)/30k = 32.3 -> 2.94 V @ 95 V '
          '(10S..20S) to GPIO9 (ADC1). 12 V rail 110k/10k -> GPIO10. Bus GND must be common with '
          'the 12 V GND (non-isolated DC-DC).')
    d.add('J', 'Connector_Generic:Conn_01x02', 'BUS SENSE', PT2, {1: 'BUS_IN', 2: GND}, fields=HAND)
    d.R('470k 150V', 'BUS_IN', 'BUS_VM', fp=R0805)
    d.R('470k 150V', 'BUS_VM', 'BUS_VS', fp=R0805)
    d.R('30k', 'BUS_VS', GND)
    d.C('100n', 'BUS_VS', GND)
    d.add('D', 'Diode:BAV99', 'BAV99', SOT23, {1: GND, 2: V3A, 3: 'BUS_VS'}, fields={'LCSC': 'C2500'})
    d.R('100k', V12, 'V12_SNS')
    d.R('10k', 'V12_SNS', GND)
    d.C('100n', 'V12_SNS', GND)

    for k in (1, 2):
        d.blk('power', 'SSR%d' % k, 'SSR%d driver: DC SSR input (3-32 V, ~10 mA) between +12V and a '
              'low-side 2N7002. LED = SSR on. SSR1 = PV switch (GPIO21), SSR2 = GPIO47 '
              '(second SSR of the back-to-back PV pair, or spare).' % k)
        d.add('J', 'Connector_Generic:Conn_01x02', 'SSR%d PV (+in / -in)' % k, PT2, {1: V12, 2: 'SSR%d_N' % k},
              fields=HAND)
        d.add('Q', 'Device:Q_NMOS_GSD', '2N7002', SOT23, {'G': 'SSR%d_G' % k, 'S': GND, 'D': 'SSR%d_N' % k},
              fields={'LCSC': 'C8545'})
        d.R('100', 'SSR%d_EN' % k, 'SSR%d_G' % k)
        d.R('100k', 'SSR%d_G' % k, GND)
        d.R('4.7k', V12, 'SSR%d_LED' % k)
        d.add('D', 'Device:LED', 'KT-0805Y yellow', LED0805, {'A': 'SSR%d_LED' % k, 'K': 'SSR%d_N' % k},
              fields={'LCSC': 'C2296'})

    # ===================== MCU =============================================
    d.blk('mcu', 'ESP', 'ESP32-S3-DevKitC-1 in 2x22 sockets (hand-soldered). ADC1 GPIO1-10 all used '
          '(pots, IS currents, bus, 12 V). GPIO19/20 used -> program via the "UART" USB port. '
          'GPIO35-37 free (octal PSRAM on R8 variants), GPIO38/48 = on-board RGB LED.')
    esp = {
        '1': 'NC', '2': 'NC', '3': 'NC',
        '4': 'A4_POS', '5': 'A1_ISA', '6': 'A2_ISA', '7': 'A3_ISA',
        '8': 'A3_RPWM', '9': 'A3_LPWM', '10': 'A4_RPWM', '11': 'A4_LPWM',
        '12': 'A4_ISA', '13': 'A3_POS', '14': 'LED_ERR',
        '15': 'BUS_VS', '16': 'V12_SNS', '17': 'A1_RPWM', '18': 'A1_LPWM',
        '19': 'A2_RPWM', '20': 'A2_LPWM', '21': V5E, '22': GND,
        '23': GND, '24': 'NC', '25': 'NC', '26': 'A1_POS', '27': 'A2_POS',
        '28': 'SCL', '29': 'SDA', '30': 'ESTOP_SENSE', '31': 'ACT_EN',
        '32': 'NC', '33': 'NC', '34': 'NC', '35': 'NC', '36': 'NC', '37': 'NC', '38': 'NC',
        '39': 'SSR2_EN', '40': 'SSR1_EN', '41': 'IMU_INT', '42': 'IMU_RST', '43': GND, '44': GND,
    }
    d.add('U', 'solbot:ESP32-S3-DevKitC-1', 'ESP32-S3-DevKitC-1', 'solbot:ESP32-S3-DevKitC-1_Socket',
          {k: v for k, v in esp.items() if v != 'NC'}, nc=[k for k, v in esp.items() if v == 'NC'], fields=HAND)
    d.R('1k', 'LED_ERR', 'LED_ERR_A')
    d.add('D', 'Device:LED', 'KT-0603R red', LED0603, {'A': 'LED_ERR_A', 'K': GND}, fields={'LCSC': 'C2286'})
    d.R('4.7k', V3A, 'SDA')
    d.R('4.7k', V3A, 'SCL')

    d.blk('mcu', 'ESTOP', 'Hardware E-stop -> IBT-2 enable: ACT_EN (GPIO39) runs through the NC E-stop '
          'loop to Q_A; Q_B inverts again, so EN5 (IDC pins 3/4 = R_EN/L_EN) is high only while the '
          'loop is closed AND the ESP drives ACT_EN. ESP reset / loop open -> EN5 low -> bridges off. '
          'Fit a wire bridge if no E-stop is used.')
    d.add('J', 'Connector_Generic:Conn_01x02', 'E-STOP (NC loop)', PT2, {1: 'ESTOP_SRC', 2: 'ESTOP_RET'}, fields=HAND)
    d.R('470', 'ACT_EN', 'ESTOP_SRC')
    d.R('1k', 'ESTOP_RET', 'ACT_GATE')
    d.R('100k', 'ACT_GATE', GND)
    d.C('100n', 'ACT_GATE', GND)
    d.R('10k', 'ESTOP_RET', 'ESTOP_SENSE')
    d.add('Q', 'Device:Q_NMOS_GSD', '2N7002', SOT23, {'G': 'ACT_GATE', 'S': GND, 'D': 'EN_N'}, fields={'LCSC': 'C8545'})
    d.R('10k', V5, 'EN_N')
    d.add('Q', 'Device:Q_NMOS_GSD', '2N7002', SOT23, {'G': 'EN_N', 'S': GND, 'D': 'ACT_EN5'}, fields={'LCSC': 'C8545'})
    d.R('2.2k', V5, 'ACT_EN5')

    d.blk('mcu', 'IMU', 'BNO085 (Adafruit 4754) on cable: 5 V, I2C @0x4A, INT -> GPIO20, RST <- GPIO19.')
    d.add('J', 'Connector_Generic:Conn_01x06', 'IMU BNO085', 'Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical',
          {1: V5, 2: GND, 3: 'SCL', 4: 'SDA', 5: 'IMU_INT', 6: 'IMU_RST'}, fields=HAND)
    d.R('10k', V3A, 'IMU_INT')
    d.C('10u 25V', V5, GND, fp=C0805)

    d.blk('mcu', 'MECH', 'Mounting holes M3.')
    for k in range(4):
        d.add('H', 'Mechanical:MountingHole', 'M3', 'MountingHole:MountingHole_3.2mm_M3', {}, fields={'Assembly': 'none'})

    # ===================== ACTUATORS =======================================
    d.blk('actuators', 'BUF', '2x 74HC14 (Basic) shift 3.3 V -> 5 V and INVERT: firmware drives the PWM '
          'active-low. 10k pull-ups on the ESP side keep the IBT-2 inputs low while the ESP boots '
          '(the EN gate keeps the bridges disabled anyway). Unused inputs tied to GND.')
    sigs = ['A1_RPWM', 'A1_LPWM', 'A2_RPWM', 'A2_LPWM', 'A3_RPWM', 'A3_LPWM', 'A4_RPWM', 'A4_LPWM']
    gates = [('1A', '1Y'), ('2A', '2Y'), ('3A', '3Y'), ('4A', '4Y'), ('5A', '5Y'), ('6A', '6Y')]
    for chip in range(2):
        conns = {'VCC': V5, 'GND': GND}
        nc = []
        for g, (a, y) in enumerate(gates):
            k = chip * 6 + g
            if k < len(sigs):
                conns[a] = sigs[k]
                conns[y] = sigs[k] + '5'
            else:
                conns[a] = GND
                nc.append(y)
        d.add('U', 'solbot:74HC14_1U', '74HC14D', 'Package_SO:SOIC-14_3.9x8.7mm_P1.27mm', conns, nc=nc,
              fields={'LCSC': 'C5605'})
        d.C('100n', V5, GND)
    for net in sigs:
        d.R('10k', V3A, net)

    roles = {1: 'TURN', 2: 'LIFT', 3: 'SLIDE', 4: 'SPARE'}   # A1 steering, A2 tool lift, A3 tool slider
    for n in range(1, 5):
        a = 'A%d' % n
        d.blk('actuators', a, 'Actuator %d (%s). IDC 2x5 (v3 pinout, flat cable 1:1 to the IBT-2 '
              'header): 1 RPWM, 2 LPWM, 3 R_EN, 4 L_EN (both = EN5 gate), 5 R_IS, 6 L_IS, 7 VCC, 8 GND, '
              '9/10 n.c. IS: 1k (DNP if the module has one) -> 1.18 V @ 10 A. Pot: 3V3A via 1k '
              '(keeps the wiper below ~3.0 V, inside the ESP ADC range), wiper -> 4.7k/100n -> ADC1.'
              % (n, roles[n]))
        d.add('J', 'Connector_Generic:Conn_02x05_Odd_Even', 'IBT-2 %s' % roles[n], IDC10,
              {1: a + '_RPWM5', 2: a + '_LPWM5', 3: 'ACT_EN5', 4: 'ACT_EN5', 5: a + '_IS', 6: a + '_IS',
               7: V5, 8: GND}, nc=[9, 10], fields=HAND)
        d.R('1k', a + '_IS', GND)
        d.R('4.7k', a + '_IS', a + '_ISA')
        d.C('100n', a + '_ISA', GND)
        d.add('J', 'Connector_Generic:Conn_01x03', 'POS %s (+ W -)' % roles[n], PT3, {1: a + '_VREF', 2: a + '_WIP', 3: GND},
              fields=HAND)
        d.R('1k', V3A, a + '_VREF')
        d.R('1M', a + '_WIP', GND)
        d.R('4.7k', a + '_WIP', a + '_POS')
        d.C('100n', a + '_POS', GND)
        d.add('D', 'Diode:BAV99', 'BAV99', SOT23, {1: GND, 2: V3A, 3: a + '_POS'}, fields={'LCSC': 'C2500'})

    d.assign_refs()
    return d


if __name__ == '__main__':
    print(len(build().parts), 'parts')
