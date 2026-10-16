"""Write project-local symbol + footprint for the ESP32-S3-DevKitC-1 socket."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(HERE, '..', 'lib')

J1 = ['3V3', '3V3', 'RST', 'GPIO4', 'GPIO5', 'GPIO6', 'GPIO7', 'GPIO15', 'GPIO16', 'GPIO17',
      'GPIO18', 'GPIO8', 'GPIO3', 'GPIO46', 'GPIO9', 'GPIO10', 'GPIO11', 'GPIO12', 'GPIO13',
      'GPIO14', '5V', 'GND']
J3 = ['GND', 'TX/GPIO43', 'RX/GPIO44', 'GPIO1', 'GPIO2', 'GPIO42', 'GPIO41', 'GPIO40', 'GPIO39',
      'GPIO38', 'GPIO37', 'GPIO36', 'GPIO35', 'GPIO0', 'GPIO45', 'GPIO48', 'GPIO47', 'GPIO21',
      'GPIO20', 'GPIO19', 'GND', 'GND']

ROW_SPACING = 25.4   # verify against your physical board (see README)


def ptype(name):
    if name == 'GND':
        return 'power_in'
    if name in ('5V',):
        return 'power_in'
    if name == '3V3':
        return 'power_out'
    if name == 'RST':
        return 'input'
    return 'bidirectional'


def symbol():
    pins = []
    for i, nm in enumerate(J1):
        y = 26.67 - i * 2.54
        pins.append('(pin %s line (at -17.78 %.2f 0) (length 5.08) (name "%s" (effects (font (size 1.27 1.27)))) '
                    '(number "%d" (effects (font (size 1.27 1.27)))))' % (ptype(nm), y, nm, i + 1))
    for i, nm in enumerate(J3):
        y = 26.67 - i * 2.54
        pins.append('(pin %s line (at 17.78 %.2f 180) (length 5.08) (name "%s" (effects (font (size 1.27 1.27)))) '
                    '(number "%d" (effects (font (size 1.27 1.27)))))' % (ptype(nm), y, nm, 23 + i))
    return '''(kicad_symbol_lib (version 20220914) (generator solbot_gen)
  (symbol "ESP32-S3-DevKitC-1" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
    (property "Reference" "U" (at 0 30.48 0) (effects (font (size 1.27 1.27))))
    (property "Value" "ESP32-S3-DevKitC-1" (at 0 -30.48 0) (effects (font (size 1.27 1.27))))
    (property "Footprint" "solbot:ESP32-S3-DevKitC-1_Socket" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
    (property "Datasheet" "https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
    (property "ki_description" "ESP32-S3-DevKitC-1 dev board, 2x22 pin headers (J1 = pins 1-22, J3 = pins 23-44)" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
    (symbol "ESP32-S3-DevKitC-1_0_1"
      (rectangle (start -12.7 29.21) (end 12.7 -29.21) (stroke (width 0.254) (type default)) (fill (type background)))
    )
    (symbol "ESP32-S3-DevKitC-1_1_1"
      %s
    )
  )
)
''' % '\n      '.join(pins)


def footprint():
    pads = []
    for i in range(22):
        shape = 'rect' if i == 0 else 'oval'
        pads.append('(pad "%d" thru_hole %s (at 0 %.2f) (size 1.7 1.7) (drill 1.0) (layers "*.Cu" "*.Mask"))'
                    % (i + 1, shape, i * 2.54))
    for i in range(22):
        pads.append('(pad "%d" thru_hole oval (at %.2f %.2f) (size 1.7 1.7) (drill 1.0) (layers "*.Cu" "*.Mask"))'
                    % (23 + i, ROW_SPACING, i * 2.54))
    x0, x1 = -1.6, ROW_SPACING + 1.6
    y0, y1 = -7.6, 55.14   # 62.74 mm board, antenna end at pin-1 side
    return '''(footprint "ESP32-S3-DevKitC-1_Socket" (version 20221018) (generator solbot_gen) (layer "F.Cu")
  (descr "Socket for Espressif ESP32-S3-DevKitC-1 (2x22, 2.54 mm, rows %.2f mm apart). Antenna end = pin 1 side; keep copper away from it.")
  (tags "ESP32-S3 DevKitC socket")
  (attr through_hole)
  (fp_text reference "REF**" (at %.2f -9 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
  (fp_text value "ESP32-S3-DevKitC-1" (at %.2f 26 90) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))
  (fp_text user "ANTENNA" (at %.2f -4.5 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
  (fp_text user "USB" (at %.2f 52.5 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
  (fp_rect (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.12) (type solid)) (fill none) (layer "F.SilkS"))
  (fp_line (start %.2f -1.5) (end %.2f -1.5) (stroke (width 0.12) (type dash)) (layer "F.SilkS"))
  (fp_rect (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))
  (fp_rect (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))
  %s
)
''' % (ROW_SPACING, ROW_SPACING / 2, ROW_SPACING / 2, ROW_SPACING / 2, ROW_SPACING / 2,
       x0, y0, x1, y1, x0, x1, x0 - 0.25, y0 - 0.25, x1 + 0.25, y1 + 0.25, x0, y0, x1, y1,
       '\n  '.join(pads))


def write():
    os.makedirs(os.path.join(LIB, 'solbot.pretty'), exist_ok=True)
    open(os.path.join(LIB, 'solbot.kicad_sym'), 'w').write(symbol())
    open(os.path.join(LIB, 'solbot.pretty', 'ESP32-S3-DevKitC-1_Socket.kicad_mod'), 'w').write(footprint())


if __name__ == '__main__':
    write()
