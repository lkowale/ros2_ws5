"""solbot_main v4 — single source of truth for the board netlist.

Everything (schematic, PCB, BOM) is generated from the parts defined here.
Run `python3 build.py` to regenerate the KiCad files.

Functional summary
  * 4 battery channels (BAT1..BAT4) + 1 PV channel, each: Kelvin shunt + INA241A2
    (bidirectional, -5..110 V CM) + voltage divider + back-to-back P-MOSFET switch
    with a bus-voltage-independent constant-current gate driver (works 10S..20S).
  * VAUX = diode-OR of all channel inputs -> LM5164 (6-100 V) -> 5 V -> AP2112 3V3A.
    Logic stays alive no matter which pack switches are open.
  * ESP32-S3-DevKitC-1 (socketed).
  * 6 x IBT-2 (BTS7960) ports via 74HCT244 level shifters, hardware E-stop gate,
    IS current sense -> ESP ADC1, 6 x pot position inputs -> 2 x ADS1115.
  * BNO085 (Adafruit breakout) on JST-XH 6.
  * I2C bus A: BNO085, ADS1115 0x48/0x49 (actuators)
    I2C bus B: ADS1115 0x48/0x49/0x4A (power), TCA9535 0x20, expansion header.
"""

R0603 = 'Resistor_SMD:R_0603_1608Metric'
R1206 = 'Resistor_SMD:R_1206_3216Metric'
C0603 = 'Capacitor_SMD:C_0603_1608Metric'
C0805 = 'Capacitor_SMD:C_0805_2012Metric'
C1206 = 'Capacitor_SMD:C_1206_3216Metric'
C1210 = 'Capacitor_SMD:C_1210_3225Metric'
LED0603 = 'LED_SMD:LED_0603_1608Metric'
SOT23 = 'Package_TO_SOT_SMD:SOT-23'
D2PAK = 'Package_TO_SOT_SMD:TO-263-2'
SHUNT = 'Resistor_SMD:R_Shunt_Vishay_WSK2512_6332Metric_T1.19mm'
LUG = 'MountingHole:MountingHole_4.3mm_M4_Pad_Via'
PT3 = 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-3-3.5-H_1x03_P3.50mm_Horizontal'
PT2 = 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-2-3.5-H_1x02_P3.50mm_Horizontal'

SHEETS = [
    ('power_path', 'Power path: battery & PV channels, bus'),
    ('supply_sense', 'Logic supply, power ADCs, IO expander'),
    ('mcu', 'ESP32-S3, IMU, E-stop'),
    ('actuators', 'Linear actuators: 6x IBT-2, position & current sense'),
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

    # two-terminal helpers ------------------------------------------------
    def R(self, val, a, b, fp=R0603, **kw):
        return self.add('R', 'Device:R', val, fp, {1: a, 2: b}, **kw)

    def C(self, val, a, b, fp=C0603, **kw):
        return self.add('C', 'Device:C', val, fp, {1: a, 2: b}, **kw)

    def CP(self, val, pos, neg, fp, **kw):
        return self.add('C', 'Device:C_Polarized', val, fp, {1: pos, 2: neg}, **kw)

    def assign_refs(self):
        counters = {}
        for p in self.parts:
            counters[p.prefix] = counters.get(p.prefix, 0) + 1
            p.ref = '%s%d' % (p.prefix, counters[p.prefix])


# ---------------------------------------------------------------------------
GND = 'GND'
V5 = '+5V'
V3A = '+3V3A'     # analog / I2C 3.3 V rail (AP2112 #1)
VPOT = '+3V3_POT'  # pot excitation rail (AP2112 #2)
BUS = 'BUS+'
VAUX = 'VAUX'


def power_channel(d, ch, en_net, is_pv=False):
    """One battery/PV channel: lug -> shunt -> Q_A -> Q_B -> BUS+."""
    n = lambda s: '%s_%s' % (ch, s)
    note = ('%s channel: 1 mOhm Kelvin shunt, INA241A2 (20 V/V, REF=Vs/2 -> bidirectional, '
            '+-82 A FS), back-to-back P-FET switch. Gate driver sinks ~2 mA via MMBTA42, '
            '12 V zener sets Vgs independent of bus voltage (10S..20S).' % ch)
    d.blk('power_path', ch, note)
    d.add('J', 'Mechanical:MountingHole_Pad', ch + '+ (M4 lug)', LUG, {1: n('IN')})
    d.add('R', 'Device:R_Shunt', '1m 1% WSK2512', SHUNT,
          {1: n('IN'), 2: n('SNP'), 3: n('SNN'), 4: n('SH')},
          fields={'MPN': 'Vishay WSK25121L000FEA'})
    fet = {'MPN': 'Vishay SQM120P10_10M1L (-100 V, 10 mOhm)'}
    d.add('Q', 'Device:Q_PMOS_GDS', 'SQM120P10_10M1L', D2PAK,
          {'G': n('G'), 'D': n('SH'), 'S': n('COM')}, fields=fet)
    d.add('Q', 'Device:Q_PMOS_GDS', 'SQM120P10_10M1L', D2PAK,
          {'G': n('G'), 'D': BUS, 'S': n('COM')}, fields=fet)
    d.R('10k', n('G'), n('COM'))
    d.add('D', 'Device:D_Zener', '12V BZT52C12', 'Diode_SMD:D_SOD-123',
          {'K': n('COM'), 'A': n('G')})
    d.add('Q', 'Device:Q_NPN_BEC', 'MMBTA42 (300 V)', SOT23,
          {'B': n('B'), 'E': n('E'), 'C': n('G')})
    d.R('1.2k', n('E'), GND)
    d.R('1k', en_net, n('B'))
    d.R('100k', n('B'), GND)
    # voltage divider 1031k/33k -> 3.20 V @ 100 V
    d.R('499k', n('IN'), n('VM'), fp=R1206)
    d.R('499k', n('VM'), n('VS'), fp=R1206)
    d.R('33k 0.1%', n('VS'), GND)
    d.C('100n', n('VS'), GND)
    # current sense
    d.add('U', 'Amplifier_Current:INA241A2xD', 'INA241A2xD', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
          {'+': n('SNP'), '-': n('SNN'), 'V+': V3A, 'GND': GND, 'REF1': V3A, 'REF2': GND, 5: n('IO')})
    d.C('100n', V3A, GND)
    d.R('1k', n('IO'), n('IS'))
    d.C('100n', n('IS'), GND)
    # keep-alive diode into VAUX
    d.add('D', 'Device:D', 'ES1D (200 V 1 A)', 'Diode_SMD:D_SMA', {'A': n('IN'), 'K': VAUX})
    if is_pv:
        d.add('D', 'Device:D_Zener', 'SMCJ58A (TVS, reverse-PV clamp)', 'Diode_SMD:D_SMC',
              {'K': n('IN'), 'A': GND})


def build():
    d = Design()

    # ===================== POWER PATH ======================================
    for i in range(1, 5):
        power_channel(d, 'BAT%d' % i, 'BAT%d_EN' % i)
    power_channel(d, 'PV', 'PV_EN', is_pv=True)

    d.blk('power_path', 'BUS', 'Common bus: lugs, bulk capacitance, TVS, bus voltage divider. '
          'GND lugs take pack negatives, PV- and load returns.')
    for k in range(2):
        d.add('J', 'Mechanical:MountingHole_Pad', 'BUS+ (M4 lug)', LUG, {1: BUS})
    for k in range(3):
        d.add('J', 'Mechanical:MountingHole_Pad', 'GND (M4 lug)', LUG, {1: GND})
    for k in range(2):
        d.CP('100u 100V', BUS, GND, 'Capacitor_THT:CP_Radial_D12.5mm_P5.00mm')
    for k in range(2):
        d.C('1u 100V', BUS, GND, fp=C1210)
    d.add('D', 'Device:D_Zener', 'SMCJ58A (TVS)', 'Diode_SMD:D_SMC', {'K': BUS, 'A': GND})
    d.R('499k', BUS, 'BUS_VM', fp=R1206)
    d.R('499k', 'BUS_VM', 'BUS_VS', fp=R1206)
    d.R('33k 0.1%', 'BUS_VS', GND)
    d.C('100n', 'BUS_VS', GND)

    # ===================== LOGIC SUPPLY ====================================
    d.blk('supply_sense', 'BUCK', 'LM5164 COT buck VAUX(18..100 V) -> 5 V 1 A, 300 kHz. '
          'RON=5*2500/300=41.2k. Type-3 ripple: RA*CA <= ton*(Vin-Vout)/20mV '
          '-> 221k/3.3n (19-21 mV @ 30-84 V). CB>=75us/(3*RFB1). UVLO on at 18 V.')
    d.CP('22u 100V', VAUX, GND, 'Capacitor_SMD:CP_Elec_8x10')
    d.C('2.2u 100V', VAUX, GND, fp=C1210)
    d.C('2.2u 100V', VAUX, GND, fp=C1210)
    d.add('U', 'Regulator_Switching:LM5164DDA', 'LM5164DDAR',
          'Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias',
          {'GND': GND, 'EP': GND, 'VIN': VAUX, 'EN/UVLO': 'BUCK_EN', 'RON': 'BUCK_RON',
           'FB': 'BUCK_FB', 'BST': 'BUCK_BST', 'SW': 'BUCK_SW'}, nc=['PGOOD'])
    d.R('1M', VAUX, 'BUCK_EN', fp=R1206)
    d.R('91k', 'BUCK_EN', GND)
    d.R('41.2k', 'BUCK_RON', GND)
    d.C('2.2n 50V', 'BUCK_BST', 'BUCK_SW')
    d.add('L', 'Device:L', '47u 1.5A SRN6045TA-470M', 'Inductor_SMD:L_Bourns_SRN6045TA',
          {1: 'BUCK_SW', 2: V5})
    d.R('221k', 'BUCK_SW', 'BUCK_RA')
    d.C('3.3n', 'BUCK_RA', V5)
    d.C('180p', 'BUCK_RA', 'BUCK_FB')
    d.R('158k', V5, 'BUCK_FB')
    d.R('49.9k', 'BUCK_FB', GND)
    d.C('22u 25V', V5, GND, fp=C1206)
    d.C('22u 25V', V5, GND, fp=C1206)
    d.R('2.2k', V5, 'LED_5V')
    d.add('D', 'Device:LED', 'green 5V', LED0603, {'A': 'LED_5V', 'K': GND})

    d.blk('supply_sense', 'LDO', 'AP2112K-3.3: analog / I2C rail +3V3A.')
    d.C('2.2u', V5, GND, fp=C0805)
    d.add('U', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3', 'Package_TO_SOT_SMD:SOT-23-5',
          {'VIN': V5, 'EN': V5, 'GND': GND, 'VOUT': V3A})
    d.C('2.2u', V3A, GND, fp=C0805)

    d.blk('supply_sense', 'PADC', 'Power ADCs on I2C bus B (ADS1115 @3.3 V, PGA +-4.096 V). '
          '0x48: BAT1/BAT2, 0x49: BAT3/BAT4, 0x4A: PV, BUS, board temp.')
    adc_map = [('GND', ['BAT1_VS', 'BAT1_IS', 'BAT2_VS', 'BAT2_IS']),
               (V3A, ['BAT3_VS', 'BAT3_IS', 'BAT4_VS', 'BAT4_IS']),
               ('SDA_B', ['PV_VS', 'PV_IS', 'BUS_VS', 'TEMP'])]
    for addr, ins in adc_map:
        d.add('U', 'Analog_ADC:ADS1115IDGS', 'ADS1115IDGS', 'Package_SO:TSSOP-10_3x3mm_P0.5mm',
              {'VDD': V3A, 'GND': GND, 'SCL': 'SCL_B', 'SDA': 'SDA_B', 'ADDR': addr,
               'AIN0': ins[0], 'AIN1': ins[1], 'AIN2': ins[2], 'AIN3': ins[3]}, nc=['ALERT/RDY'])
        d.C('100n', V3A, GND)
    d.R('10k', V3A, 'TEMP')
    d.add('TH', 'Device:Thermistor_NTC', '10k B3950 (place near FETs)', 'Resistor_SMD:R_0603_1608Metric',
          {1: 'TEMP', 2: GND})
    d.C('100n', 'TEMP', GND)

    d.blk('supply_sense', 'EXP', 'TCA9535 @0x20 (bus B). No internal pull-ups -> pack switches '
          'default OFF at power-up; state survives ESP32 reset/OTA.')
    d.add('U', 'Interface_Expansion:TCA9535PWR', 'TCA9535PWR', 'Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm',
          {'VCC': V3A, 'GND': GND, 'SCL': 'SCL_B', 'SDA': 'SDA_B', 'A0': GND, 'A1': GND, 'A2': GND,
           '~{INT}': 'EXP_INT',
           'P00': 'BAT1_EN', 'P01': 'BAT2_EN', 'P02': 'BAT3_EN', 'P03': 'BAT4_EN',
           'P04': 'IMU_RST', 'P05': 'LED_PWR_K', 'P06': 'EXP_IO0', 'P07': 'EXP_IO1',
           'P10': 'EXP_IO2', 'P11': 'EXP_IO3', 'P12': 'EXP_IO4', 'P13': 'EXP_IO5',
           'P14': 'EXP_IO6', 'P15': 'EXP_IO7', 'P16': 'EXP_IO8', 'P17': 'EXP_IO9'})
    d.C('100n', V3A, GND)
    d.R('10k', V3A, 'EXP_INT')
    d.R('1k', V3A, 'LED_PWR_A')
    d.add('D', 'Device:LED', 'yellow PWR-STAT', LED0603, {'A': 'LED_PWR_A', 'K': 'LED_PWR_K'})
    d.add('J', 'Connector_Generic:Conn_01x12', 'EXP IO', 'Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical',
          {1: V3A, 2: GND, **{i + 3: 'EXP_IO%d' % i for i in range(10)}})

    d.blk('supply_sense', 'I2C', 'I2C pull-ups (4.7k to +3V3A) and bus B expansion connector.')
    d.R('4.7k', V3A, 'SDA_A')
    d.R('4.7k', V3A, 'SCL_A')
    d.R('4.7k', V3A, 'SDA_B')
    d.R('4.7k', V3A, 'SCL_B')
    d.add('J', 'Connector_Generic:Conn_01x04', 'I2C-B ext', 'Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical',
          {1: V3A, 2: GND, 3: 'SDA_B', 4: 'SCL_B'})

    # ===================== MCU =============================================
    d.blk('mcu', 'ESP', 'ESP32-S3-DevKitC-1 (N8/N8R2) in 2x22 sockets. Program & log via the '
          '"UART" USB port only: GPIO19/20 (USB-OTG) are used as I2C bus B. '
          'GPIO35-37 left free (octal PSRAM on R8 variants), GPIO38/48 = on-board RGB LED.')
    esp = {
        # J1
        '1': 'NC', '2': 'NC', '3': 'NC',
        '4': 'A3_ISA', '5': 'A4_ISA', '6': 'A5_ISA', '7': 'A6_ISA',
        '8': 'A4_RPWM', '9': 'A4_LPWM', '10': 'A5_RPWM', '11': 'A5_LPWM',
        '12': 'ESTOP_SENSE', '13': 'IMU_INT', '14': 'LED_ERR',
        '15': 'A1_RPWM', '16': 'A1_LPWM', '17': 'A2_RPWM', '18': 'A2_LPWM',
        '19': 'A3_RPWM', '20': 'A3_LPWM', '21': '5V_ESP', '22': GND,
        # J3 (pin n -> 22+n)
        '23': GND, '24': 'NC', '25': 'NC', '26': 'A1_ISA', '27': 'A2_ISA',
        '28': 'SCL_A', '29': 'SDA_A', '30': 'PV_EN', '31': 'ACT_EN',
        '32': 'NC', '33': 'NC', '34': 'NC', '35': 'NC', '36': 'NC', '37': 'NC', '38': 'NC',
        '39': 'A6_LPWM', '40': 'A6_RPWM', '41': 'SCL_B', '42': 'SDA_B', '43': GND, '44': GND,
    }
    d.add('U', 'solbot:ESP32-S3-DevKitC-1', 'ESP32-S3-DevKitC-1', 'solbot:ESP32-S3-DevKitC-1_Socket',
          {k: v for k, v in esp.items() if v != 'NC'}, nc=[k for k, v in esp.items() if v == 'NC'])
    d.add('D', 'Device:D_Schottky', 'SS34 (blocks USB back-feed)', 'Diode_SMD:D_SMA', {'A': V5, 'K': '5V_ESP'})
    d.C('10u', '5V_ESP', GND, fp=C0805)
    d.R('1k', 'LED_ERR', 'LED_ERR_A')
    d.add('D', 'Device:LED', 'red ERR', LED0603, {'A': 'LED_ERR_A', 'K': GND})

    d.blk('mcu', 'ESTOP', 'Hardware E-stop: ACT_EN (GPIO39) is looped through the NC E-stop '
          'contact; loop open or ESP not driving -> 74HCT244 outputs Hi-Z -> 10k pull-downs '
          '-> all IBT-2 PWM/EN low. Fit a wire bridge if no E-stop is used.')
    d.add('J', 'Connector_Generic:Conn_01x02', 'E-STOP (NC loop)', PT2, {1: 'ESTOP_SRC', 2: 'ESTOP_RET'})
    d.R('470', 'ACT_EN', 'ESTOP_SRC')
    d.R('1k', 'ESTOP_RET', 'ACT_GATE')
    d.R('100k', 'ACT_GATE', GND)
    d.C('100n', 'ACT_GATE', GND)
    d.R('10k', 'ESTOP_RET', 'ESTOP_SENSE')
    d.add('Q', 'Device:Q_NMOS_GSD', '2N7002', SOT23, {'G': 'ACT_GATE', 'S': GND, 'D': 'ACT_OE_N'})
    d.R('10k', V5, 'ACT_OE_N')

    d.blk('mcu', 'IMU', 'BNO085 (Adafruit 4754 breakout) on cable. 5 V supply, I2C bus A @0x4A, '
          'INT -> GPIO3 (not a boot strap unless eFuse set), RST via TCA9535.')
    d.add('J', 'Connector_Generic:Conn_01x06', 'IMU BNO085', 'Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical',
          {1: V5, 2: GND, 3: 'SCL_A', 4: 'SDA_A', 5: 'IMU_INT', 6: 'IMU_RST'})
    d.R('10k', V3A, 'IMU_INT')
    d.C('10u', V5, GND, fp=C0805)

    d.blk('mcu', 'MECH', 'Mounting holes M3.')
    for k in range(4):
        d.add('H', 'Mechanical:MountingHole', 'M3', 'MountingHole:MountingHole_3.2mm_M3', {})

    # ===================== ACTUATORS =======================================
    d.blk('actuators', 'BUF', '74HCT244 level shifters 3.3 V -> 5 V for IBT-2 inputs. '
          '/OE driven by E-stop gate. 10k pull-downs keep IBT-2 inputs low when Hi-Z.')
    hct = [
        {'1A0': 'A1_RPWM', '1A1': 'A1_LPWM', '1A2': 'A2_RPWM', '1A3': 'A2_LPWM',
         '2A0': 'A3_RPWM', '2A1': 'A3_LPWM', '2A2': 'A4_RPWM', '2A3': 'A4_LPWM'},
        {'1A0': 'A5_RPWM', '1A1': 'A5_LPWM', '1A2': 'A6_RPWM', '1A3': 'A6_LPWM',
         '2A0': V3A, '2A1': GND, '2A2': GND, '2A3': GND},
    ]
    outs = []
    for k, m in enumerate(hct):
        conns = {'VCC': V5, 'GND': GND, '1OE': 'ACT_OE_N', '2OE': 'ACT_OE_N'}
        nc = []
        for a, net in m.items():
            conns[a] = net
            y = a.replace('A', 'Y')
            if net in (GND,):
                nc.append(y)
            elif net == V3A:
                conns[y] = 'ACT_EN5'
                outs.append('ACT_EN5')
            else:
                conns[y] = net + '5'
                outs.append(net + '5')
        d.add('U', '74xx:74HCT244', '74HCT244', 'Package_SO:SOIC-20W_7.5x12.8mm_P1.27mm', conns, nc=nc)
        d.C('100n', V5, GND)
    for o in outs:
        d.R('10k', o, GND)

    d.blk('actuators', 'POSREF', 'Pot excitation rail (separate AP2112 so a shorted sensor cable '
          'cannot take down the ADC/I2C rail) + ratiometric reference measurement.')
    d.C('2.2u', V5, GND, fp=C0805)
    d.add('U', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3', 'Package_TO_SOT_SMD:SOT-23-5',
          {'VIN': V5, 'EN': V5, 'GND': GND, 'VOUT': VPOT})
    d.C('2.2u', VPOT, GND, fp=C0805)
    d.R('10k', VPOT, 'VREF_SNS')
    d.R('10k', 'VREF_SNS', GND)
    d.C('100n', 'VREF_SNS', GND)
    d.R('10k', V5, 'V5_SNS')
    d.R('10k', 'V5_SNS', GND)
    d.C('100n', 'V5_SNS', GND)

    d.blk('actuators', 'AADC', 'Actuator position ADCs on I2C bus A: 0x48 = A1..A4, '
          '0x49 = A5, A6, VREF/2, 5V/2.')
    for addr, ins in (('GND', ['A1_POS', 'A2_POS', 'A3_POS', 'A4_POS']),
                      (V3A, ['A5_POS', 'A6_POS', 'VREF_SNS', 'V5_SNS'])):
        d.add('U', 'Analog_ADC:ADS1115IDGS', 'ADS1115IDGS', 'Package_SO:TSSOP-10_3x3mm_P0.5mm',
              {'VDD': V3A, 'GND': GND, 'SCL': 'SCL_A', 'SDA': 'SDA_A', 'ADDR': addr,
               'AIN0': ins[0], 'AIN1': ins[1], 'AIN2': ins[2], 'AIN3': ins[3]}, nc=['ALERT/RDY'])
        d.C('100n', V3A, GND)

    roles = {1: 'steering', 2: 'tool lift', 3: 'tool slider', 4: 'spare', 5: 'spare', 6: 'spare'}
    for n in range(1, 7):
        a = 'A%d' % n
        d.blk('actuators', a, 'Actuator %d (%s). IBT-2 header pinout = module header 1:1 '
              '(RPWM LPWM R_EN L_EN R_IS L_IS VCC GND). R_IS||L_IS -> 1k (DNP if module has '
              'its own) -> 1.18 V @ 10 A (k_ILIS 8500). Pot: 3V3_POT / wiper / GND.' % (n, roles[n]))
        d.add('J', 'Connector_Generic:Conn_01x08', 'IBT-2 #%d' % n,
              'Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical',
              {1: a + '_RPWM5', 2: a + '_LPWM5', 3: 'ACT_EN5', 4: 'ACT_EN5',
               5: a + '_IS', 6: a + '_IS', 7: V5, 8: GND})
        d.R('1k', a + '_IS', GND)
        d.R('4.7k', a + '_IS', a + '_ISA')
        d.C('100n', a + '_ISA', GND)
        d.add('J', 'Connector_Generic:Conn_01x03', 'POS %d' % n, PT3,
              {1: a + '_VREF', 2: a + '_WIP', 3: GND})
        d.R('47', VPOT, a + '_VREF')
        d.R('1M', a + '_WIP', GND)
        d.R('4.7k', a + '_WIP', a + '_POS')
        d.C('100n', a + '_POS', GND)
        d.add('D', 'Diode:BAT54S', 'BAT54S', SOT23, {'A': GND, 'K': V3A, 'COM': a + '_POS'})

    d.assign_refs()
    return d


if __name__ == '__main__':
    d = build()
    print(len(d.parts), 'parts')
