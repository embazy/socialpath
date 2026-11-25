
#!/usr/bin/env python3
# ================================================================
# SMARTLOCK — KICAD 10 PCB GENERATOR + FREEROUTING DSN EXPORT
#
# BOARD: 46 x 62 mm, 1.6 mm, 2 copper layers
#
# FRONT + BACK COMPONENT PLACEMENT
#
# Output:
#   output_padlock_pcb/
#       padlock_controller.kicad_pcb
#       padlock_controller_YYYYMMDD_HHMMSS.dsn (unique per run)
#       padlock_bom.csv
#       pad_assignment_report.txt
#       footprint_cache/
#
# Placement:
#   - ESP support/GPIO and battery-sense resistor networks are on B.Cu
#     alongside U1; power-stage and charger parts retain their assigned sides.
#     Solenoid connector J2 remains on F.Cu; central LED D3 remains on B.Cu.
#   - Components are distributed across both copper sides with XY collision checks.
#   - R3-R9 and R21-R24 are assigned to B.Cu to reduce ESP signal vias.
#   - A full-width horizontal placement lane around solenoid connector J2 is
#     reserved so no other component shares its row.
#   - Bare ESP8266EX QFN32, external W25Q32 flash and 26 MHz crystal.
#   - Placement audit checks component body/courtyard overlap on both PCB sides.
#   - No M2 mounting-hole NPTH footprints are generated.
#   - No dedicated programming header is fitted; ESP UART pads remain available.
#   - ESP8266EX QFN32 is placed at the lower-right with a host PCB antenna at the right edge.
#   - USB-C J1 fixed at lower-left, mirrored, receptacle opening facing the left edge.
#   - FreeRouting width set to 0.20 mm; edge routing margin reserved on both layers.
#   - Central WS2812B-V6 3.3 V RGB LED position fixed.
#   - Antenna routing keepouts block unrelated F.Cu and B.Cu tracks; B.Cu
#     uses simple rectangular keepouts around an open RF meander corridor.
#   - No copper pours or copper zones are generated; routing is track/via only.
#   - Every electrical pad is assigned through PAD_NETS or explicitly marked NC.
#   - Final placement audit runs before saving.
#
# Power architecture:
#   - Single-cell 3.7 V nominal LiPo (4.2 V fully charged) at J3.
#   - U3 regulates the LiPo to the main 3.3 V ESP8266EX/RGB rail.
#   - U7 independently regulates the LiPo to a 3.0 V rail for the 3 V solenoid.
#   - U7 feedback divider R17=499k / R18=100k sets approximately 3.0 V
#     for the TPS63070 0.5 V feedback reference.
#   - Keep the independent solenoid regulator until coil current and
#     regulator transient capability are verified; do not assume the
#     ESP8266EX 3V3 rail is isolated from the 3.0 V solenoid pulse rail.
#
# Electrical:
#   - ESP8266EX CHIP_EN (pad 7) and supply pins are connected to 3V3.
#   - D3 WS2812B-V6 powered from 3V3; ESP data connects directly.
#   - Obsolete 5 V LED boost and AHCT level-shifter stages removed.
#   - DW01A OD -> FS8205A G1 pad 4.
#   - DW01A OC -> FS8205A G2 pad 5.
#   - USB-C SH shell pads -> GND.
#   - Non-copper and NPTH pads excluded from electrical audit.
#
# IMPORTANT:
#   Antenna geometry is not tuned.
#   No RF matching network is fitted.
#   This generator does not claim routing or DRC completion.
# ================================================================

import os
import sys
import csv
import random
import math
import re
from datetime import datetime
import urllib.request
import pcbnew


# ================================================================
# CONFIGURATION
# ================================================================

OUTPUT_DIR = "output_padlock_pcb"

OUTPUT_FILE = os.path.join(
    OUTPUT_DIR, "padlock_controller.kicad_pcb"
)
# Use a unique DSN filename on every run. Windows locks files opened by
# FreeRouting, so never delete or overwrite the previous run's DSN.
DSN_RUN_TAG = datetime.now().strftime("%Y%m%d_%H%M%S")
DSN_FILE = os.path.join(
    OUTPUT_DIR, "padlock_controller_{}.dsn".format(DSN_RUN_TAG)
)
BOM_FILE = os.path.join(
    OUTPUT_DIR, "padlock_bom.csv"
)
REPORT_FILE = os.path.join(
    OUTPUT_DIR, "pad_assignment_report.txt"
)
FREEROUTING_COMMAND_FILE = os.path.join(
    OUTPUT_DIR, "FREEROUTING_ALL_NETS_COMMAND.txt"
)
FREEROUTING_LAUNCHER_FILE = os.path.join(
    OUTPUT_DIR, "RUN_FREEROUTING_ALL_NETS.bat"
)
FOOTPRINT_CACHE_ROOT = os.path.join(
    OUTPUT_DIR, "footprint_cache"
)

PCB_WIDTH = 46.0
PCB_HEIGHT = 62.0
PCB_THICKNESS = 1.6

# DRC clearances for the selected, manufacturer-defined footprints.
# Keep the board's normal 0.25 mm hole-to-copper rule. The four J1 GND
# contact pads are shortened along their hole-facing axis in the DRC override
# stage so the physical geometry clears the USB-C locating holes.
USB_NPTH_HOLE_CLEARANCE = 0.25
USB_GND_CONTACT_LENGTH = 0.90
RF_FEED_TRACE_WIDTH = 0.20
# FreeRouting was creating 0.15 mm tracks despite the PCB's 0.20 mm minimum.
# Set the default netclass width to the actual board minimum before DSN export.
ROUTED_TRACK_WIDTH = 0.20
ROUTED_VIA_DIAMETER = 0.60
ROUTED_VIA_DRILL = 0.30
# Reserve sufficient room for 0.20 mm tracks to maintain 0.50 mm copper-to-edge.
BOARD_EDGE_TRACK_CLEARANCE = 0.50
BOARD_EDGE_ROUTING_MARGIN = BOARD_EDGE_TRACK_CLEARANCE + ROUTED_TRACK_WIDTH / 2.0
# TPS63070 switch/GND lead geometry has a measured 0.15 mm pad gap. Apply a
# footprint-local override only to the adjacent pins on U3/U7.
TPS63070_ADJACENT_PAD_CLEARANCE = 0.14

BOARD_MARGIN = 1.0
COMPONENT_CLEARANCE = 0.65  # More breathing room between component envelopes
# Extra 2 mm body-to-body exclusion around the ESP8266EX QFN32 and RF support area. This is in
# addition to the ordinary component clearance and applies to BOTH sides.
ESP_CLEARANCE = 2.0  # General clearance from other component envelopes
# USB-C is fixed at the lower-left edge, mirrored so its receptacle faces
# the left board edge. The full transformed footprint envelope is used.
USB_BOTTOM_EDGE_MARGIN = 1.2
USB_LEFT_EDGE_MARGIN = 1.2

# The solenoid wire connector is centred directly above the central RGB LED.
# Its row is kept clear across the board width; all other components are
# distributed through the remaining upper and lower placement areas.
SOLENOID_ROW_EXTRA_CLEARANCE = 0.50
PLACEMENT_GRID_STEP = 0.5  # Half-mm grid for better board-wide placement and routing access
# Finer occupancy map for even board-wide placement. The reserved J2 lane
# naturally removes its cells from consideration, while the remaining cells
# are balanced independently rather than allowing one dense cluster.
PLACEMENT_GRID_COLUMNS = 8
PLACEMENT_GRID_ROWS = 10

LED_CENTRE_X = PCB_WIDTH / 2
LED_CENTRE_Y = PCB_HEIGHT / 2

PACKING_ATTEMPTS = 80  # Alternate orders retained as a collision-safe fallback
RANDOM_SEED = 4217

# ESP8266EX is a bare QFN32 device. External SPI flash, 26 MHz crystal,
# RES12K resistor and a right-edge host PCB antenna are fitted.


# Mounting-hole NPTH footprints and their keepouts have been removed.
MOUNT_HOLES = []
MOUNT_HOLE_CLEARANCE = 0.0


# ================================================================
# FOOTPRINT LIBRARIES
# ================================================================

def find_footprint_root():
    candidates = []

    for variable in (
        "KICAD10_FOOTPRINT_DIR",
        "KICAD_FOOTPRINT_DIR",
        "KICAD9_FOOTPRINT_DIR",
        "KICAD8_FOOTPRINT_DIR"
    ):
        value = os.environ.get(variable)
        if value:
            candidates.append(value)

    candidates.extend([
        r"C:\Program Files\KiCad\10.0\share\kicad\footprints",
        r"C:\Program Files\KiCad\9.0\share\kicad\footprints",
        r"C:\Program Files\KiCad\8.0\share\kicad\footprints"
    ])

    exe_dir = os.path.dirname(os.path.abspath(sys.executable))

    candidates.extend([
        os.path.abspath(os.path.join(
            exe_dir, "..", "share", "kicad", "footprints"
        )),
        os.path.abspath(os.path.join(
            exe_dir, "..", "..", "share", "kicad", "footprints"
        ))
    ])

    for candidate in candidates:
        candidate = os.path.normpath(candidate)

        if not os.path.isdir(candidate):
            continue

        try:
            contents = os.listdir(candidate)
        except OSError:
            continue

        if any(x.endswith(".pretty") for x in contents):
            print("Footprint library root:", candidate)
            return candidate

    raise RuntimeError(
        "Could not locate KiCad footprint libraries."
    )


FOOTPRINT_ROOT = find_footprint_root()


FOOTPRINT_ALIASES = {
    ("Package_DFN_QFN", "QFN-32-1EP_5x5mm_P0.5mm"):
        ("Package_DFN_QFN",
         "QFN-32-1EP_5x5mm_P0.5mm_EP3.1x3.1mm"),

    ("Package_SO", "FS8205A_TSSOP8"):
        ("Package_SO", "TSSOP-8_3x3mm_P0.65mm"),

    ("Package_DFN_QFN", "TPS63070"):
        ("Package_DFN_QFN",
         "Texas_VQFN-HR-15_3x2.5mm_P0.5mm_Merged_7_8_AND_12_13"),

    ("Inductor_SMD", "L_4.0x4.0mm"):
        ("Inductor_SMD", "L_Vishay_IFSC-1515AH_4x4x1.8mm"),

    ("Button_Switch_SMD", "SW_SPST_TL3301AN"):
        ("Button_Switch_SMD",
         "SW_Push_1P1T_NO_E-Switch_TL3301NxxxxxG"),

    ("Connector_USB", "USB_C_Receptacle_USB2.0_16P"):
        ("Connector_USB",
         "USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal"),

    ("Connector", "Conn_01x02_Pin"):
        ("Connector_Wire",
         "SolderWire-1sqmm_1x02_P5.4mm_D1.4mm_OD2.7mm"),

    ("Connector", "Conn_01x02_Battery"):
        ("Connector_JST",
         "JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical"),

    ("MountingHole", "MountingHole_2.2mm"):
        ("MountingHole", "MountingHole_2.2mm_M2")
}


# ================================================================
# TPS63070 EXACT FOOTPRINT DOWNLOAD
# ================================================================

TPS63070_FOOTPRINT_NAME = (
    "Texas_VQFN-HR-15_3x2.5mm_P0.5mm_Merged_7_8_AND_12_13"
)

TPS63070_FOOTPRINT_LIBRARY = "Package_DFN_QFN"

TPS63070_FOOTPRINT_URLS = [
    (
        "https://gitlab.com/dani007200964/kicad-footprints/-/raw/"
        "Texas_VQFN-HR-15_3x2.5mm_P0.5mm_Merged_7_8_AND_12_13/"
        "Package_DFN_QFN.pretty/"
        + TPS63070_FOOTPRINT_NAME + ".kicad_mod"
    ),
    (
        "https://gitlab.com/kicad/libraries/kicad-footprints/-/raw/"
        "master/Package_DFN_QFN.pretty/"
        + TPS63070_FOOTPRINT_NAME + ".kicad_mod"
    ),
    (
        "https://raw.githubusercontent.com/KiCad/kicad-footprints/"
        "master/Package_DFN_QFN.pretty/"
        + TPS63070_FOOTPRINT_NAME + ".kicad_mod"
    )
]


def validate_kicad_mod_file(filename, expected_name):
    if not os.path.isfile(filename):
        return False

    try:
        with open(filename, "r", encoding="utf-8-sig") as f:
            content = f.read()
    except (OSError, UnicodeError):
        return False

    content = content.lstrip()

    if not content.startswith("(footprint"):
        return False

    if expected_name not in content[:1000]:
        return False

    if "(pad " not in content:
        return False

    if '(pad "7" ' not in content:
        return False

    if '(pad "13" ' not in content:
        return False

    if '(pad "8" ' in content:
        return False

    if '(pad "12" ' in content:
        return False

    return True


def download_tps63070_footprint():
    cache_library = os.path.join(
        FOOTPRINT_CACHE_ROOT,
        TPS63070_FOOTPRINT_LIBRARY + ".pretty"
    )

    os.makedirs(cache_library, exist_ok=True)

    target_file = os.path.join(
        cache_library,
        TPS63070_FOOTPRINT_NAME + ".kicad_mod"
    )

    if validate_kicad_mod_file(
        target_file, TPS63070_FOOTPRINT_NAME
    ):
        print("Using cached TPS63070 footprint:", target_file)
        return cache_library, target_file

    errors = []
    temporary_file = target_file + ".download"

    print("\nDownloading exact TPS63070 merged-pad footprint...")

    for url in TPS63070_FOOTPRINT_URLS:
        try:
            print("Trying:", url)

            request = urllib.request.Request(
                url,
                headers={
                    "User-Agent": "SMARTLOCK-KiCad-Footprint-Loader/1.2"
                }
            )

            with urllib.request.urlopen(
                request, timeout=40
            ) as response:
                if response.status != 200:
                    raise RuntimeError(
                        "HTTP status {}".format(response.status)
                    )

                data = response.read()

            if not data:
                raise RuntimeError("Downloaded file is empty.")

            with open(temporary_file, "wb") as f:
                f.write(data)

            if not validate_kicad_mod_file(
                temporary_file, TPS63070_FOOTPRINT_NAME
            ):
                raise RuntimeError(
                    "Downloaded footprint failed validation."
                )

            os.replace(temporary_file, target_file)

            print("TPS63070 footprint cached:", target_file)
            return cache_library, target_file

        except Exception as exc:
            errors.append("{}: {}".format(url, exc))

            try:
                if os.path.exists(temporary_file):
                    os.remove(temporary_file)
            except OSError:
                pass

    raise RuntimeError(
        "Could not download TPS63070 footprint:\n"
        + "\n".join(errors)
    )


def resolve_footprint(library_id, footprint_name):
    library, name = FOOTPRINT_ALIASES.get(
        (library_id, footprint_name),
        (library_id, footprint_name)
    )

    library_path = os.path.join(
        FOOTPRINT_ROOT, library + ".pretty"
    )

    filename = os.path.join(
        library_path, name + ".kicad_mod"
    )

    return library_path, name, filename


def load_footprint(library_id, footprint_name):
    library_path, name, filename = resolve_footprint(
        library_id, footprint_name
    )

    if os.path.isfile(filename):
        footprint = pcbnew.FootprintLoad(
            library_path, name
        )

        if footprint is not None:
            return footprint

    original_library_path = os.path.join(
        FOOTPRINT_ROOT, library_id + ".pretty"
    )

    original_filename = os.path.join(
        original_library_path,
        footprint_name + ".kicad_mod"
    )

    if os.path.isfile(original_filename):
        footprint = pcbnew.FootprintLoad(
            original_library_path, footprint_name
        )

        if footprint is not None:
            return footprint

    if (
        library_id == "Package_DFN_QFN"
        and footprint_name in (
            "TPS63070",
            TPS63070_FOOTPRINT_NAME
        )
    ):
        cache_library, cache_file = download_tps63070_footprint()

        if not validate_kicad_mod_file(
            cache_file, TPS63070_FOOTPRINT_NAME
        ):
            raise RuntimeError(
                "TPS63070 cache validation failed."
            )

        print(
            "Loading cached TPS63070 with pcbnew.FootprintLoad...",
            flush=True
        )
        footprint = pcbnew.FootprintLoad(
            cache_library, TPS63070_FOOTPRINT_NAME
        )
        print(
            "TPS63070 FootprintLoad returned:",
            "None" if footprint is None else type(footprint).__name__,
            flush=True
        )

        if footprint is not None:
            return footprint

        raise RuntimeError(
            "Downloaded TPS63070 footprint could not load."
        )

    raise RuntimeError(
        "Missing footprint.\nResolved: {}\nOriginal: {}".format(
            filename, original_filename
        )
    )


# ================================================================
# COMPONENT DEFINITIONS
# ================================================================

COMPONENTS = [
    ("U1", "ESP8266EX QFN32", "Package_DFN_QFN",
     "QFN-32-1EP_5x5mm_P0.5mm_EP3.1x3.1mm", 180, "B", "Espressif ESP8266EX bare chip"),
    ("U2", "W25Q32JV 4MB SPI FLASH", "Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm", 0, "B", "W25Q32JVSIQ"),
    ("Y1", "26MHz CRYSTAL", "Crystal", "Crystal_SMD_3225-4Pin_3.2x2.5mm", 0, "B", "26MHz 3225 crystal; verify CL"),
    ("R25", "12k RES12K", "Resistor_SMD", "R_0402_1005Metric", 0, "B", "12k 1%"),
    ("C18", "100nF ESP8266 DECOUPLING", "Capacitor_SMD", "C_0402_1005Metric", 0, "B", "100nF"),
    ("C19", "12pF XTAL LOAD", "Capacitor_SMD", "C_0402_1005Metric", 0, "B", "12pF; verify crystal CL"),
    ("C20", "12pF XTAL LOAD", "Capacitor_SMD", "C_0402_1005Metric", 0, "B", "12pF; verify crystal CL"),
    ("L2", "0R RF SERIES LINK", "Inductor_SMD", "L_0402_1005Metric", 0, "B", "0R RF tuning link"),
    ("C21", "DNP RF SHUNT MATCH", "Capacitor_SMD", "C_0402_1005Metric", 0, "B", "DNP RF match"),
    ("C22", "DNP RF SHUNT MATCH", "Capacitor_SMD", "C_0402_1005Metric", 0, "B", "DNP RF match"),

    ("U4", "MCP73831 4.2V CHARGER", "Package_TO_SOT_SMD",
     "SOT-23-5", 0, "F", "MCP73831T-2ACI/OT"),

    ("Q1", "AO3400A", "Package_TO_SOT_SMD",
     "SOT-23", 0, "F", "AO3400A"),

    ("D1", "SOLENOID FLYBACK", "Diode_SMD",
     "D_SMA", 0, "F", "SS34"),

    ("J1", "USB-C CHARGE", "Connector_USB",
     "USB_C_Receptacle_USB2.0_16P", 270, "F", "GCT USB4105"),

    ("J2", "M141C-3V SOLENOID (3.0V RAIL)", "Connector_Wire",
     "SolderWire-1sqmm_1x02_P5.4mm_D1.4mm_OD2.7mm",
     0, "F", "Solenoid connector"),

    ("U3", "TPS63070 3V3", "Package_DFN_QFN",
     "TPS63070", 0, "B", "TPS63070RNMR"),

    ("U7", "TPS63070 3V SOLENOID", "Package_DFN_QFN",
     "TPS63070", 0, "B", "TPS63070RNMR"),

    ("L1", "1.5uH 3V3", "Inductor_SMD",
     "L_4.0x4.0mm", 0, "B", "1.5uH shielded inductor"),

    ("L3", "1.5uH 3V3", "Inductor_SMD",
     "L_4.0x4.0mm", 0, "B", "1.5uH shielded inductor"),

    ("U6", "DW01A", "Package_TO_SOT_SMD",
     "SOT-23-6", 0, "B", "DW01A"),

    ("Q2", "FS8205A DUAL N-MOSFET", "Package_SO",
     "FS8205A_TSSOP8", 0, "B", "FS8205A TSSOP-8"),

    ("J3", "BATTERY JST-PH", "Connector_JST",
     "JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
     0, "B", "B2B-PH-K"),

    ("R3", "10k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "10k"),

    ("R4", "10k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "10k"),

    ("R5", "10k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "10k"),

    ("R6", "10k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "10k"),

    ("R7", "2k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "2k"),

    ("R8", "100R", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "100R"),

    ("R9", "100k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "100k"),


    ("R13", "560k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "560k"),

    ("R14", "100k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "100k"),

    ("R17", "499k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "499k"),

    ("R18", "100k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "100k"),

    ("R19", "470R", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "470R"),

    ("R20", "1k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "1k"),

    ("R21", "5k1", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "5.1k"),

    ("R22", "5k1", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "5.1k"),

    ("R23", "330k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "330k"),

    ("R24", "100k", "Resistor_SMD",
     "R_0402_1005Metric", 0, "B", "100k"),

    ("C1", "100nF", "Capacitor_SMD",
     "C_0402_1005Metric", 0, "B", "100nF"),

    ("C2", "10uF", "Capacitor_SMD",
     "C_0603_1608Metric", 0, "F", "10uF"),

    ("C3", "100nF", "Capacitor_SMD",
     "C_0402_1005Metric", 0, "F", "100nF"),

    ("C4", "10uF", "Capacitor_SMD",
     "C_0603_1608Metric", 0, "B", "10uF"),

    ("C5", "22uF", "Capacitor_SMD",
     "C_0805_2012Metric", 0, "B", "22uF"),

    ("C6", "100nF", "Capacitor_SMD",
     "C_0402_1005Metric", 0, "B", "100nF"),

    ("C7", "10uF", "Capacitor_SMD",
     "C_0603_1608Metric", 0, "B", "10uF"),

    ("C10", "10uF", "Capacitor_SMD",
     "C_0603_1608Metric", 0, "F", "10uF"),

    ("C11", "22uF", "Capacitor_SMD",
     "C_0805_2012Metric", 0, "B", "22uF"),

    ("C12", "100nF", "Capacitor_SMD",
     "C_0402_1005Metric", 0, "B", "100nF"),

    ("C13", "4.7uF", "Capacitor_SMD",
     "C_0603_1608Metric", 0, "F", "4.7uF"),

    ("C14", "4.7uF", "Capacitor_SMD",
     "C_0603_1608Metric", 0, "F", "4.7uF"),

    ("C15", "100nF", "Capacitor_SMD",
     "C_0402_1005Metric", 0, "B", "100nF"),

    ("C16", "100nF", "Capacitor_SMD",
     "C_0402_1005Metric", 0, "F", "100nF"),

    ("C17", "10uF", "Capacitor_SMD",
     "C_0805_2012Metric", 0, "F", "10uF")
]


CENTRAL_LED = (
    "D3", "WS2812B-V6 3V3 RGB", "LED_SMD",
    "LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm",
    0, "F", "WS2812B-V6"
)


# ================================================================
# NETS
# ================================================================

NET_NAMES = [
    "GND", "USB_VBUS", "USB_CC1", "USB_CC2",
    "BAT+", "BAT-", "BAT_FET_COMMON",
    "3V3", "3V_SOLENOID",
    "3V3_AUX", "3V_SOL_AUX",
    "SOLENOID_GATE", "SOLENOID_LOW",
    "ESP_RST", "ESP_GPIO0", "ESP_GPIO2", "ESP_GPIO4",
    "ESP_GPIO5", "ESP_GPIO12", "ESP_GPIO13", "ESP_GPIO14",
    "ESP_GPIO15", "ESP_GPIO16", "ESP_TX", "ESP_RX",
    "BATTERY_SENSE",
    "FLASH_CS", "FLASH_MISO", "FLASH_IO2", "FLASH_MOSI", "FLASH_CLK", "FLASH_IO3",
    "XTAL_IN", "XTAL_OUT", "RES12K", "RF_IN", "RF_ANT",
    "LED_DATA_IN",
    "CHARGE_STATUS", "CHARGE_PROG",
    "U3_SW_A", "U3_SW_B",
    "U7_SW_A", "U7_SW_B",
    "U3_FB", "U7_FB",
    "DW_VCC", "DW_CS", "DW_OD", "DW_OC"
]


# ================================================================
# PAD ASSIGNMENTS
# ================================================================

PAD_NETS = {
    "U1": {
        "1":"3V3", "2":"RF_IN", "3":"3V3", "4":"3V3", "5":"NC_VDD_RTC",
        "6":"BATTERY_SENSE", "7":"3V3", "8":"ESP_GPIO16", "9":"ESP_GPIO14",
        "10":"ESP_GPIO12", "11":"3V3", "12":"ESP_GPIO13", "13":"ESP_GPIO15",
        "14":"ESP_GPIO2", "15":"ESP_GPIO0", "16":"ESP_GPIO4", "17":"3V3",
        "18":"FLASH_IO2", "19":"FLASH_IO3", "20":"FLASH_CS", "21":"FLASH_CLK",
        "22":"FLASH_MISO", "23":"FLASH_MOSI", "24":"ESP_GPIO5", "25":"ESP_RX",
        "26":"ESP_TX", "27":"XTAL_OUT", "28":"XTAL_IN", "29":"3V3", "30":"3V3",
        "31":"RES12K", "32":"ESP_RST", "33":"GND"
    },
    "U2": {"1":"FLASH_CS","2":"FLASH_MISO","3":"FLASH_IO2","4":"GND","5":"FLASH_MOSI","6":"FLASH_CLK","7":"FLASH_IO3","8":"3V3"},
    "Y1": {"1":"XTAL_IN","2":"GND","3":"XTAL_OUT","4":"GND"},
    "R25": {"1":"RES12K","2":"GND"},
    "C18": {"1":"3V3","2":"GND"},
    "C19": {"1":"XTAL_IN","2":"GND"},
    "C20": {"1":"XTAL_OUT","2":"GND"},
    "L2": {"1":"RF_IN","2":"RF_ANT"},
    "C21": {"1":"RF_IN","2":"GND"},
    "C22": {"1":"RF_ANT","2":"GND"},
    "U3": {
        "1": "BAT+", "2": "NC_PG", "3": "3V3_AUX",
        "4": "GND", "5": "U3_FB", "6": "NC_FB2",
        "7": "3V3", "8": "3V3", "7,8": "3V3",
        "9": "U3_SW_B", "10": "GND", "11": "U3_SW_A",
        "12": "BAT+", "13": "BAT+", "12,13": "BAT+",
        "14": "BAT+", "15": "GND"
    },

    "U4": {
        "1": "CHARGE_STATUS", "2": "GND",
        "3": "BAT+", "4": "USB_VBUS", "5": "CHARGE_PROG"
    },

    "U6": {
        "1": "DW_OD", "2": "DW_CS", "3": "DW_OC",
        "4": "NC_DW", "5": "DW_VCC", "6": "BAT-"
    },

    "Q2": {
        "1": "BAT_FET_COMMON", "8": "BAT_FET_COMMON",
        "2": "BAT-", "3": "BAT-", "4": "DW_OD",
        "5": "DW_OC", "6": "GND", "7": "GND"
    },

    "U7": {
        "1": "BAT+", "2": "NC_PG", "3": "3V_SOL_AUX",
        "4": "GND", "5": "U7_FB", "6": "NC_FB2",
        "7": "3V_SOLENOID", "8": "3V_SOLENOID",
        "7,8": "3V_SOLENOID", "9": "U7_SW_B",
        "10": "GND", "11": "U7_SW_A",
        "12": "BAT+", "13": "BAT+", "12,13": "BAT+",
        "14": "BAT+", "15": "GND"
    },

    "Q1": {
        "1": "SOLENOID_GATE", "2": "GND", "3": "SOLENOID_LOW"
    },

    "D1": {
        "1": "3V_SOLENOID", "2": "SOLENOID_LOW"
    },

    "J3": {
        "1": "BAT+", "2": "BAT-"
    },

    "J2": {
        "1": "3V_SOLENOID", "2": "SOLENOID_LOW"
    },

    "J1": {
        "A1": "GND", "A4": "USB_VBUS",
        "A5": "USB_CC1", "A6": "NC_USB_DATA",
        "A7": "NC_USB_DATA", "A8": "NC_USB",
        "A9": "USB_VBUS", "A12": "GND",
        "B1": "GND", "B4": "USB_VBUS",
        "B5": "USB_CC2", "B6": "NC_USB_DATA",
        "B7": "NC_USB_DATA", "B8": "NC_USB",
        "B9": "USB_VBUS", "B12": "GND",
        # KiCad USB-C footprints use S1-S4 for the four shield tabs.
        # Some vendor/library variants label these SH1-SH4 or SH.
        "S1": "GND", "S2": "GND", "S3": "GND", "S4": "GND",
        "SH": "GND", "SH1": "GND", "SH2": "GND",
        "SH3": "GND", "SH4": "GND"
    },

    "D3": {
        "1": "3V3", "2": "NC_LED_DO",
        "3": "GND", "4": "LED_DATA_IN"
    },


    "R13": {
        "1": "3V3", "2": "U3_FB"
    },

    "R14": {
        "1": "U3_FB", "2": "GND"
    },

    "R17": {
        "1": "3V_SOLENOID", "2": "U7_FB"
    },

    "R18": {
        "1": "U7_FB", "2": "GND"
    },



    "R3": {
        "1": "3V3", "2": "ESP_RST"
    },

    "R4": {
        "1": "3V3", "2": "ESP_GPIO0"
    },

    "R5": {
        "1": "3V3", "2": "ESP_GPIO2"
    },

    "R6": {
        "1": "ESP_GPIO15", "2": "GND"
    },

    "R7": {
        "1": "CHARGE_PROG", "2": "GND"
    },

    "R8": {
        "1": "SOLENOID_GATE", "2": "ESP_GPIO4"
    },

    "R9": {
        "1": "SOLENOID_GATE", "2": "GND"
    },


    "R19": {
        "1": "BAT+", "2": "DW_VCC"
    },

    "R20": {
        "1": "DW_CS", "2": "GND"
    },

    "R21": {
        "1": "USB_CC1", "2": "GND"
    },

    "R22": {
        "1": "USB_CC2", "2": "GND"
    },

    "R23": {
        "1": "BAT+", "2": "BATTERY_SENSE"
    },

    "R24": {
        "1": "BATTERY_SENSE", "2": "GND"
    },

    "L1": {
        "1": "U3_SW_A", "2": "U3_SW_B"
    },

    "L3": {
        "1": "U7_SW_A", "2": "U7_SW_B"
    },

    "C4": {
        "1": "BAT+", "2": "GND"
    },

    "C5": {
        "1": "3V3", "2": "GND"
    },

    "C6": {
        "1": "3V3_AUX", "2": "GND"
    },

    "C7": {
        "1": "BAT+", "2": "GND"
    },

    "C10": {
        "1": "BAT+", "2": "GND"
    },

    "C11": {
        "1": "3V_SOLENOID", "2": "GND"
    },

    "C12": {
        "1": "3V_SOL_AUX", "2": "GND"
    },

    "C13": {
        "1": "USB_VBUS", "2": "GND"
    },

    "C14": {
        "1": "BAT+", "2": "GND"
    },

    "C15": {
        "1": "DW_VCC", "2": "BAT-"
    },

    "C1": {
        "1": "3V3", "2": "GND"
    },

    "C2": {
        "1": "3V3", "2": "GND"
    },

    "C3": {
        "1": "3V3", "2": "GND"
    },

    "C16": {
        "1": "3V3", "2": "GND"
    },

    "C17": {
        "1": "3V3", "2": "GND"
    }
}


# ================================================================
# GEOMETRY
# ================================================================

def mm(value):
    return pcbnew.FromMM(float(value))


def point(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def to_mm(value):
    return float(pcbnew.ToMM(int(value)))


def item_bounds(item):
    box = item.GetBoundingBox()

    return (
        int(box.GetX()),
        int(box.GetY()),
        int(box.GetX() + box.GetWidth()),
        int(box.GetY() + box.GetHeight())
    )


def union_bounds(items):
    if not items:
        return None

    return (
        min(r[0] for r in items),
        min(r[1] for r in items),
        max(r[2] for r in items),
        max(r[3] for r in items)
    )


def expand_rectangle(rect, clearance):
    x1, y1, x2, y2 = rect
    gap = mm(clearance)

    return (
        x1-gap,
        y1-gap,
        x2+gap,
        y2+gap
    )


def rectangle_dimensions(rect):
    return (
        rect[2]-rect[0],
        rect[3]-rect[1]
    )


def rectangle_overlap(a, b):
    return not (
        a[2] <= b[0] or b[2] <= a[0] or
        a[3] <= b[1] or b[3] <= a[1]
    )


def rectangle_inside_board(rect):
    margin = mm(BOARD_MARGIN)

    return (
        rect[0] >= margin
        and rect[1] >= margin
        and rect[2] <= mm(PCB_WIDTH)-margin
        and rect[3] <= mm(PCB_HEIGHT)-margin
    )


def mounting_hole_keepouts():
    result = []
    radius = MOUNT_HOLE_CLEARANCE

    for _, x, y in MOUNT_HOLES:
        result.append((
            mm(x-radius),
            mm(y-radius),
            mm(x+radius),
            mm(y+radius)
        ))

    return result


# ================================================================
# COURTYARD BOUNDS
# ================================================================

def footprint_pad_items(footprint):
    """Return all electrical PAD objects without iterating SWIG vectors.

    KiCad's GetPadCount() defaults to INCLUDE_NPTH.  The USB-C footprint has
    20 electrical pads plus two NPTH locating holes, so comparing that default
    count (22) with the electrical pads returned by GetUniquePadNumbers() and
    FindPadByNumber() incorrectly reports two missing pads.  Count only
    electrical pads with EXCLUDE_NPTH, then walk every unique pad number and
    its duplicate-number pads using FindPadByNumber().

    This avoids FOOTPRINT.Pads(), GetPads()' non-iterable SWIG vector, and
    RunOnChildren()'s unmarshalable std::function callback.
    """
    result = []
    errors = []

    # Count only copper/electrical pads. NPTH locating holes are mechanical
    # and must not be passed to net assignment or pad-number validation.
    try:
        # KiCad 10 exports this enum as pcbnew.DO_NOT_INCLUDE_NPTH.
        # INCLUDE_NPTH is the default and counts J1's two locating holes.
        exclude_npth = getattr(pcbnew, "DO_NOT_INCLUDE_NPTH", None)
        if exclude_npth is None:
            # Some older builds expose the enum as a nested type.
            include_type = getattr(pcbnew, "INCLUDE_NPTH_T", None)
            exclude_npth = getattr(include_type, "EXCLUDE_NPTH", None) if include_type else None
        if exclude_npth is None:
            raise RuntimeError("pcbnew.DO_NOT_INCLUDE_NPTH enum is unavailable")
        physical_count = int(footprint.GetPadCount(exclude_npth))
    except Exception as exc:
        physical_count = -1
        errors.append("GetPadCount(EXCLUDE_NPTH): {}".format(exc))

    try:
        raw_numbers = footprint.GetUniquePadNumbers()
        unique_numbers = sorted(set(str(n) for n in list(raw_numbers)))
    except Exception as exc:
        unique_numbers = []
        errors.append("GetUniquePadNumbers: {}".format(exc))

    for number in unique_numbers:
        try:
            pad = footprint.FindPadByNumber(number)
        except Exception as exc:
            errors.append("FindPadByNumber({}): {}".format(number, exc))
            continue

        duplicate_guard = 0
        while pad is not None and duplicate_guard < 512:
            # Prevent returning the same SWIG proxy twice.
            try:
                pointer = pad.this
            except Exception:
                pointer = None
            if pointer is not None and any(
                getattr(existing, "this", None) == pointer for existing in result
            ):
                break
            result.append(pad)
            duplicate_guard += 1
            try:
                next_pad = footprint.FindPadByNumber(number, pad)
            except Exception as exc:
                errors.append("duplicate lookup {}: {}".format(number, exc))
                break
            if next_pad is None:
                break
            try:
                if next_pad.this == pad.this:
                    break
            except Exception:
                if next_pad == pad:
                    break
            pad = next_pad

    # KiCad 10's DO_NOT_INCLUDE_NPTH enum has returned an inflated count
    # for some footprints (e.g. QFN32: 42 reported vs 33 actual numbered
    # electrical pads). The unique-number walk above is authoritative: it
    # returns each numbered copper pad, including duplicate-number pads.
    # Keep the reported count as diagnostics only; do not reject a footprint
    # when KiCad's count API disagrees with the actual pad objects.
    if physical_count >= 0 and len(result) != physical_count:
        print(
            "NOTE: KiCad pad-count API reports {}; enumerated {} numbered pads for {}".format(
                physical_count, len(result), footprint.GetFPID().GetLibItemName()
            ),
            flush=True
        )

    if physical_count < 0 and not result:
        detail = "; ".join(errors) if errors else "no pad numbers returned"
        raise RuntimeError("KiCad could not retrieve footprint pads: " + detail)

    return result


def find_footprint_pad(footprint, pad_number):
    """Return the first electrical pad with the requested pad number.

    Uses footprint_pad_items(), which retrieves KiCad 10 SWIG pad objects
    without relying on the non-iterable Pads()/GetPads() wrappers.
    """
    wanted = str(pad_number)
    for pad in footprint_pad_items(footprint):
        try:
            if str(pad.GetNumber()) == wanted:
                return pad
        except Exception:
            continue
    raise RuntimeError(
        "Footprint {!r} has no electrical pad numbered {!r}".format(
            footprint.GetFPIDAsString() if hasattr(footprint, "GetFPIDAsString") else footprint.GetReference(),
            wanted
        )
    )


def footprint_placement_bounds(footprint):
    # Use the transformed physical footprint envelope, not just courtyard
    # graphics. A number of library footprints have incomplete/missing
    # courtyards; using only F_CrtYd/B_CrtYd then packs their pads or bodies
    # on top of neighbouring back-side parts. Include courtyard, fabrication
    # and silkscreen outlines on either side, plus every pad. All coordinates
    # are read from pcbnew after the footprint has been flipped and placed.
    envelope_layers = {
        pcbnew.F_CrtYd,
        pcbnew.B_CrtYd,
        pcbnew.F_Fab,
        pcbnew.B_Fab,
        pcbnew.F_SilkS,
        pcbnew.B_SilkS,
    }

    parts = []

    try:
        graphics = footprint.GraphicalItems()
    except Exception:
        graphics = []

    for graphic in graphics:
        try:
            if graphic.GetLayer() in envelope_layers:
                parts.append(item_bounds(graphic))
        except Exception:
            pass

    # Pads are always part of the physical envelope, even when a library
    # footprint has no courtyard or fabrication outline.
    try:
        for pad in footprint_pad_items(footprint):
            try:
                parts.append(item_bounds(pad))
            except Exception:
                pass
    except Exception:
        pass

    bounds = union_bounds(parts)
    if bounds is not None:
        return bounds

    # Last-resort fallback for unusual custom footprint types.
    return item_bounds(footprint)


# ================================================================
# COMPONENT PREPARATION
# ================================================================


def validate_esp8266ex_qfn32(footprint):
    """Validate the actual bare ESP8266EX QFN32 pad numbering."""
    actual = {str(p.GetNumber()) for p in footprint_pad_items(footprint)}
    expected = {str(n) for n in range(1, 34)}
    if actual != expected:
        raise RuntimeError("ESP8266EX QFN32 must expose pads 1-33; got {}".format(sorted(actual)))
    return footprint

def prepare_component(data):
    ref, value, library, name, rotation, original_side, mpn = data

    # Preserve each component's original XY placement group (F/B) from the
    # COMPONENTS definition. Back-group parts are packed in their original
    # reserved positions, then converted to F.Cu without changing their XY.
    side = str(original_side).upper()
    if side not in ("F", "B"):
        raise RuntimeError("Invalid side for {}: {}".format(ref, side))

    print("Preparing {}: loading {}:{}".format(ref, library, name), flush=True)
    footprint = load_footprint(library, name)
    print("Preparing {}: footprint loaded".format(ref), flush=True)

    footprint.SetReference(ref)
    footprint.SetValue(value)
    footprint.SetOrientationDegrees(rotation)

    # Start every footprint on the front side at the local origin.
    footprint.SetLayer(pcbnew.F_Cu)
    footprint.SetPosition(point(0, 0))

    if ref == "U1":
        footprint.SetOrientationDegrees(0)
        validate_esp8266ex_qfn32(footprint)
        footprint.SetOrientationDegrees(rotation)

    print("Preparing {}: base properties set".format(ref), flush=True)

    if side == "B":
        # Do not flip a detached footprint here. In KiCad 10 this can
        # terminate pcbnew inside SWIG for some custom footprints. The
        # footprint is flipped only after BOARD.Add() in pack_side().
        print(
            "Preparing {}: deferring B.Cu flip until footprint is on board".format(ref),
            flush=True
        )

    print("Preparing {}: calculating placement bounds".format(ref), flush=True)
    bounds = footprint_placement_bounds(footprint)
    width, height = rectangle_dimensions(bounds)

    return {
        "reference": ref,
        "value": value,
        "library": library,
        "footprint_name": name,
        "part_number": mpn,
        "side": side,
        "footprint": footprint,
        "local_bounds": bounds,
        "width": width,
        "height": height,
        "area": width*height
    }


def prepare_all_components():
    prepared = []

    print("\nLoading footprints...\n")

    for data in COMPONENTS:
        ref = data[0]
        try:
            item = prepare_component(data)
        except Exception as exc:
            raise RuntimeError(
                "Footprint preparation failed for {} ({}): {}".format(
                    ref, data[1], exc
                )
            ) from exc
        prepared.append(item)

        print(
            "{:<4} {:<24} {:>5.1f} x {:>5.1f} mm {}".format(
                item["reference"],
                item["value"][:24],
                to_mm(item["width"]),
                to_mm(item["height"]),
                item["side"]
            )
        )

    led = prepare_component(CENTRAL_LED)

    return prepared, led


# ================================================================
# NET CREATION AND PAD ASSIGNMENT
# ================================================================

def create_nets(board):
    nets = {}

    for name in NET_NAMES:
        net = pcbnew.NETINFO_ITEM(board, name)
        board.Add(net)
        nets[name] = net

    return nets


def write_freerouting_all_nets_instructions():
    """Write a launcher that routes every remaining connection.

    No copper pours are generated. Every electrical pad is mapped to a net
    or explicitly marked as an intentional no-connect.
    GND pads remain explicitly assigned.
    """
    dsn_name = os.path.basename(DSN_FILE)
    ses_name = os.path.splitext(dsn_name)[0] + ".ses"
    command = (
        r'java -jar "C:\path\to\freerouting.jar" '
        '-de "{}" -do "{}"'.format(dsn_name, ses_name)
    )

    with open(FREEROUTING_COMMAND_FILE, "w", encoding="utf-8") as f:
        f.write(
            "ALL NETS ARE ENABLED FOR FREEROUTING. NO COPPER POURS ARE GENERATED.\n\n"
            "Run RUN_FREEROUTING_ALL_NETS.bat, or use this command from the "
            "output folder after replacing the JAR path:\n\n"
            + command
            + "\n\n"
            "No copper zones or pours are generated; all connections must be routed with tracks and vias.\n"
            "All non-anchor components are placed below central LED D3; solenoid connector J2 remains above D3.\n"
            "ESP GPIO/reset and battery-sense resistor networks R3-R9 and R21-R24 are assigned to B.Cu.\n"
            "All electrical pads are assigned to named nets or explicitly marked intentional NC.\n"
            "F.Cu and B.Cu antenna keepouts block unrelated routing; B.Cu has only the RF meander and dedicated feed corridor open.\n"
            "After importing the SES file into KiCad, run DRC.\n"
        )

    launcher_lines = [
        "@echo off",
        "setlocal",
        'cd /d "%~dp0"',
        "echo FreeRouting receives all nets; no copper pours are generated.",
        "echo All electrical component pads are assigned or explicitly marked NC.",
        "set /p FR_JAR=Full path to FreeRouting JAR: ",
        'if not exist "%FR_JAR%" (',
        "  echo ERROR: The FreeRouting JAR was not found.",
        "  pause",
        "  exit /b 1",
        ")",
        'java -jar "%FR_JAR%" -de "{}" -do "{}"'.format(dsn_name, ses_name),
        "if errorlevel 1 (",
        "  echo FreeRouting returned an error.",
        ") else (",
        "  echo Routing finished. All nets were included.",
        "  echo Import the SES session file into KiCad and run DRC.",
        ")",
        "pause",
        "",
    ]
    with open(FREEROUTING_LAUNCHER_FILE, "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\r\n".join(launcher_lines))


def expanded_pad_mapping(mapping):
    expanded = {}

    for key, value in mapping.items():
        key_text = str(key).strip()
        expanded[key_text] = value

        for number in key_text.split(","):
            expanded[number.strip()] = value

    return expanded


def pad_has_copper_layers(pad):
    npth_attribute = getattr(
        pcbnew, "PAD_ATTRIB_NPTH", None
    )

    try:
        if (
            npth_attribute is not None
            and pad.GetAttribute() == npth_attribute
        ):
            return False
    except Exception:
        pass

    try:
        layers = pad.GetLayerSet()

        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            try:
                if layers.Contains(layer):
                    return True
            except Exception:
                pass

    except Exception:
        pass

    try:
        return (
            pad.IsOnLayer(pcbnew.F_Cu)
            or pad.IsOnLayer(pcbnew.B_Cu)
        )
    except Exception:
        return False


def assign_component_nets(
    item, nets, report, audit_errors
):
    reference = item["reference"]

    mapping = expanded_pad_mapping(
        PAD_NETS.get(reference, {})
    )

    for pad in footprint_pad_items(item["footprint"]):
        pad_number = str(pad.GetNumber()).strip()

        if not pad_has_copper_layers(pad):
            report.append(
                "{} pad {}: ignored non-copper/mechanical pad".format(
                    reference,
                    pad_number or "<blank>"
                )
            )
            continue

        if not pad_number:
            report.append(
                "{} pad <blank>: UNASSIGNED COPPER PAD".format(
                    reference
                )
            )

            audit_errors.append(
                "{} pad <blank> (copper)".format(reference)
            )
            continue

        net_name = mapping.get(pad_number)

        # Robustly catch the USB-C shell tabs even if the installed KiCad
        # footprint revision uses a different shield-pad spelling.
        if net_name is None and reference == "J1":
            shell_name = pad_number.upper().replace("_", "").replace("-", "")
            if shell_name in {
                "S1", "S2", "S3", "S4",
                "SH", "SH1", "SH2", "SH3", "SH4",
                "SHIELD1", "SHIELD2", "SHIELD3", "SHIELD4"
            }:
                net_name = "GND"

        if net_name is None:
            report.append(
                "{} pad {}: UNASSIGNED".format(
                    reference, pad_number
                )
            )

            audit_errors.append(
                "{} pad {}".format(reference, pad_number)
            )
            continue

        if net_name.startswith("NC_"):
            # Intentional NC pads are accounted for, but must not receive a
            # fabricated net: doing so would create an artificial airwire.
            report.append(
                "{} pad {}: ACCOUNTED INTENTIONAL NO-CONNECT ({})".format(
                    reference, pad_number, net_name
                )
            )
            continue

        if net_name not in nets:
            raise RuntimeError(
                "Net '{}' not declared for {} pad {}".format(
                    net_name, reference, pad_number
                )
            )

        pad.SetNet(nets[net_name])

        # Verify the pad actually retained the intended net assignment.
        # This catches footprint/API assignment failures before the PCB is saved.
        try:
            expected_code = nets[net_name].GetNetCode()
            actual_code = pad.GetNetCode()
            if int(actual_code) != int(expected_code):
                raise RuntimeError(
                    "Pad net assignment did not persist: {} pad {} expected {} (code {}), got code {}".format(
                        reference, pad_number, net_name, expected_code, actual_code
                    )
                )
        except AttributeError:
            # Older pcbnew bindings may not expose GetNetCode on PAD.
            # SetNet above remains the authoritative assignment call.
            pass

        report.append(
            "{} pad {} -> {}".format(
                reference, pad_number, net_name
            )
        )
        if net_name == "GND":
            report.append(
                "GND PAD VERIFIED: {} pad {} assigned to GND net".format(
                    reference, pad_number
                )
            )


# ================================================================
# PLACEMENT ENGINE
# ================================================================

def reserved_rectangles(side):
    return list(mounting_hole_keepouts()) + [
        (mm(40.0), mm(34.0), mm(PCB_WIDTH), mm(PCB_HEIGHT))
    ]


def generate_packing_orders(items):
    orders = [
        sorted(
            items,
            key=lambda x: x["area"],
            reverse=True
        ),

        sorted(
            items,
            key=lambda x: max(x["width"], x["height"]),
            reverse=True
        ),

        sorted(
            items,
            key=lambda x: x["height"],
            reverse=True
        ),

        sorted(
            items,
            key=lambda x: x["width"],
            reverse=True
        ),

        sorted(
            items,
            key=lambda x: x["reference"]
        )
    ]

    rng = random.Random(RANDOM_SEED)

    for _ in range(PACKING_ATTEMPTS):
        shuffled = list(items)
        rng.shuffle(shuffled)
        orders.append(shuffled)

    return orders


def _pad_net_name(pad):
    """Return a KiCad pad net name safely across pcbnew API variants."""
    try:
        name = pad.GetNetname()
        if name:
            return str(name)
    except Exception:
        pass
    try:
        net = pad.GetNet()
        if net:
            return str(net.GetNetname())
    except Exception:
        pass
    return ""


def _item_pad_positions(item, planned_rect=None):
    """Return (net, x, y) pad positions for a fixed or planned footprint."""
    footprint = item["footprint"]
    if planned_rect is None:
        dx = dy = 0
    else:
        local = item["local_bounds"]
        dx = planned_rect[0] - local[0]
        dy = planned_rect[1] - local[1]
    result = []
    for pad in footprint_pad_items(footprint):
        net_name = _pad_net_name(pad)
        if not net_name or net_name == "GND":
            continue
        pos = pad.GetPosition()
        result.append((net_name, pos.x + dx, pos.y + dy))
    return result


def attempt_pack(order, side, fixed_rectangles, fixed_items=None):
    """Spread components across the board while favouring proximity of shared nets.

    Every candidate is checked against transformed courtyard envelopes,
    board edges, antenna reservations and fixed clearances. The score first
    limits occupied-area growth, then rewards nearby legal neighbours and
    especially short distances between pads sharing an electrical net.
    """
    if fixed_items is None:
        fixed_items = []
    occupied = list(reserved_rectangles(side))
    occupied.extend(fixed_rectangles)
    placed_rectangles = list(fixed_rectangles)
    placed_items = [(item, None) for item in fixed_items]

    margin = mm(BOARD_MARGIN)
    board_width = mm(PCB_WIDTH)
    board_height = mm(PCB_HEIGHT)
    # Use the complete legal board area on both sides of the central LED.
    # Fixed anchors, the full-width solenoid row, antenna keepouts, board
    # edges and already-placed courtyards are the only placement exclusions.
    grid_step = max(1, mm(PLACEMENT_GRID_STEP))

    def candidate_rect(item, x, y):
        return (x, y, x + item["width"], y + item["height"])

    def bounds_of(rectangles, extra=None):
        values = list(rectangles)
        if extra is not None:
            values.append(extra)
        if not values:
            return (margin, margin, margin, margin)
        return (min(r[0] for r in values), min(r[1] for r in values),
                max(r[2] for r in values), max(r[3] for r in values))

    def compact_score(item, rect, candidate_base_pads, existing_by_net):
        """Balance board coverage while keeping electrically related parts close."""
        old = bounds_of(placed_rectangles)
        new = bounds_of(placed_rectangles, rect)
        old_area = max(0, old[2]-old[0]) * max(0, old[3]-old[1])
        new_area = max(0, new[2]-new[0]) * max(0, new[3]-new[1])
        growth_mm2 = (new_area - old_area) / float(mm(1.0) ** 2)

        cx = (rect[0] + rect[2]) / 2.0
        cy = (rect[1] + rect[3]) / 2.0
        nearest = float("inf")
        for old_rect in placed_rectangles:
            ox = min(max(cx, old_rect[0]), old_rect[2])
            oy = min(max(cy, old_rect[1]), old_rect[3])
            d = (cx-ox)**2 + (cy-oy)**2
            nearest = min(nearest, d)
        if nearest == float("inf"):
            nearest = 0.0
        nearest_mm2 = nearest / float(mm(1.0) ** 2)

        # A 6 x 8 occupancy map discourages repeatedly filling the same
        # neighbourhood. Footprint centres are used so large courtyards do
        # not artificially occupy many cells.
        cell_w = board_width / float(PLACEMENT_GRID_COLUMNS)
        cell_h = board_height / float(PLACEMENT_GRID_ROWS)
        cell_x = min(PLACEMENT_GRID_COLUMNS - 1,
                     max(0, int(cx / max(1.0, cell_w))))
        cell_y = min(PLACEMENT_GRID_ROWS - 1,
                     max(0, int(cy / max(1.0, cell_h))))
        cell_load = 0
        row_load = 0
        column_load = 0
        for old_rect in placed_rectangles:
            ox = (old_rect[0] + old_rect[2]) / 2.0
            oy = (old_rect[1] + old_rect[3]) / 2.0
            old_cell_x = min(PLACEMENT_GRID_COLUMNS - 1,
                             max(0, int(ox / max(1.0, cell_w))))
            old_cell_y = min(PLACEMENT_GRID_ROWS - 1,
                             max(0, int(oy / max(1.0, cell_h))))
            if old_cell_x == cell_x and old_cell_y == cell_y:
                cell_load += 1
            if old_cell_y == cell_y:
                row_load += 1
            if old_cell_x == cell_x:
                column_load += 1

        # Translate pad coordinates to this candidate and measure proximity
        # to already placed pads sharing a net. This keeps local circuits
        # together without allowing the entire board to collapse into a clump.
        local = item["local_bounds"]
        dx = rect[0] - local[0]
        dy = rect[1] - local[1]
        net_distance_weighted = 0.0
        net_weight_total = 0.0
        # Common rails are deliberately low-weight: almost every component
        # shares 3V3 or GND, so using them as strong placement anchors pulls
        # unrelated circuits together. Dedicated signal nets carry much more
        # useful locality information for the ESP module's GPIO and power nets.
        net_weights = {
            "3V3": 0.20,
            "GND": 0.0,
            "BAT+": 0.15,
            "BAT-": 0.0,
            "USB_VBUS": 0.10,
        }
        for net_name, base_x, base_y in candidate_base_pads:
            peers = existing_by_net.get(net_name)
            if not peers:
                continue
            if net_name in net_weights:
                weight = net_weights[net_name]
            elif net_name.startswith("SPI_"):
                weight = 8.0
            elif net_name.startswith("XTAL_"):
                weight = 6.0
            elif net_name in ("LED_DATA_IN", "ESP_RST", "BATTERY_SENSE"):
                weight = 3.5
            elif net_name.startswith("ESP_GPIO") or net_name.startswith("SOLENOID_"):
                weight = 2.5
            else:
                weight = 1.5
            if weight <= 0:
                continue
            px, py = base_x + dx, base_y + dy
            nearest_pad = min((px-qx)**2 + (py-qy)**2 for qx, qy in peers)
            net_distance_weighted += nearest_pad * weight
            net_weight_total += weight
        net_penalty_mm2 = ((net_distance_weighted / net_weight_total) /
                           float(mm(1.0) ** 2)) if net_weight_total else 0.0

        # Lower scores are preferred. Dedicated signal nets are a strong
        # placement anchor; board coverage still spreads unrelated circuits
        # into free regions. This specifically keeps U2 and its WP/HOLD
        # pull-ups near U1 without clustering every component on 3V3/GND.
        # Occupancy is the dominant term: first spread parts into empty cells,
        # then balance the row and column totals. This prevents a sequence of
        # locally attractive placements from forming a dense strip or corner.
        # Signal-net locality remains important, but cannot override a badly
        # overloaded cell. The weaker growth/nearest terms avoid compacting
        # the entire layout into one small bounding box.
        return (growth_mm2 * 0.035 - nearest_mm2 * 0.075
                + net_penalty_mm2 * 2.5
                + cell_load * 85.0
                + row_load * 18.0
                + column_load * 12.0
                + rect[1] / float(mm(1.0)) * 1.0e-5
                + rect[0] / float(mm(1.0)) * 1.0e-7)

    plan = {}
    for item in order:
        # Build net-neighbour data once per component, not once per candidate.
        candidate_base_pads = _item_pad_positions(item, None)
        existing_by_net = {}
        for old_item, old_rect in placed_items:
            for net_name, px, py in _item_pad_positions(old_item, old_rect):
                existing_by_net.setdefault(net_name, []).append((px, py))
        width = item["width"]
        height = item["height"]
        max_x = board_width - margin - width
        max_y = board_height - margin - height
        # Scan from the top board margin to the bottom board margin. The
        # solenoid-row rectangle and fixed LED/connector envelopes are already
        # in occupied, so the packer can use every other legal region.
        min_y = margin
        if max_x < margin or max_y < min_y:
            return None

        best = None
        for y in range(min_y, max_y + 1, grid_step):
            for x in range(margin, max_x + 1, grid_step):
                rect = candidate_rect(item, x, y)
                expanded = expand_rectangle(rect, COMPONENT_CLEARANCE / 2.0)
                if not rectangle_inside_board(expanded):
                    continue
                if any(rectangle_overlap(expanded, old) for old in occupied):
                    continue
                score = compact_score(item, rect, candidate_base_pads, existing_by_net)
                candidate = (score, y, x, rect, expanded)
                if best is None or candidate[:3] < best[:3]:
                    best = candidate

        if best is None:
            return None
        _, _, _, chosen, chosen_expanded = best
        plan[item["reference"]] = chosen
        occupied.append(chosen_expanded)
        placed_rectangles.append(chosen)
        placed_items.append((item, chosen))

    return plan


def place_by_bounds(item, target_x, target_y):
    footprint = item["footprint"]

    x1, y1, _, _ = item["local_bounds"]

    footprint.SetPosition(
        pcbnew.VECTOR2I(
            int(target_x-x1),
            int(target_y-y1)
        )
    )


def _flip_footprint_to_back(board, footprint, reference):
    footprint.SetLayer(pcbnew.F_Cu)
    footprint.SetPosition(point(0, 0))
    board.Add(footprint)
    flip = getattr(footprint, "SetLayerAndFlip", None)
    if callable(flip):
        flip(pcbnew.B_Cu)
    else:
        direction = getattr(pcbnew, "FLIP_DIRECTION_LEFT_RIGHT", None)
        if direction is None:
            raise RuntimeError("KiCad has no supported B.Cu flip API for " + reference)
        footprint.Flip(point(0, 0), direction)
        footprint.SetLayer(pcbnew.B_Cu)
    if footprint.GetLayer() != pcbnew.B_Cu:
        raise RuntimeError("Could not flip {} to B.Cu".format(reference))


def _set_footprint_center(footprint, center_x, center_y):
    bounds = footprint_placement_bounds(footprint)
    cx, cy = (bounds[0] + bounds[2]) // 2, (bounds[1] + bounds[3]) // 2
    pos = footprint.GetPosition()
    footprint.SetPosition(pcbnew.VECTOR2I(
        int(pos.x + mm(center_x) - cx), int(pos.y + mm(center_y) - cy)
    ))
    return footprint_placement_bounds(footprint)


def place_fixed_esp(board, nets, esp, report, audit_errors, fixed_obstacles=None):
    """Place the bare QFN32 at the lower-right, with LNA pad 2 facing the PCB edge."""
    esp["side"] = "B"
    fp = esp["footprint"]
    _flip_footprint_to_back(board, fp, "U1")
    best = None
    for angle in (0, 90, 180, 270):
        fp.SetOrientationDegrees(angle)
        fp.SetPosition(point(0, 0))
        pos = find_footprint_pad(fp, "2").GetPosition()
        if best is None or pos.x > best[0]:
            best = (pos.x, angle)
    fp.SetOrientationDegrees(best[1])
    fp.SetPosition(point(0, 0))
    bounds = _set_footprint_center(fp, 36.4, 49.0)
    if not rectangle_inside_board(bounds):
        raise RuntimeError("ESP8266EX QFN32 does not fit at its right-edge anchor")
    esp["local_bounds"] = bounds
    esp["width"], esp["height"] = rectangle_dimensions(bounds)
    assign_component_nets(esp, nets, report, audit_errors)
    report.extend([
        "U1: bare Espressif ESP8266EX QFN32 on B.Cu at lower-right",
        "U1 pad 2 LNA faces the right-hand PCB antenna matching network",
        "External W25Q32 flash, 26 MHz crystal and RES12K resistor are fitted"
    ])
    print("ESP8266EX bare QFN32 placed on B.Cu; orientation {} bounds {}".format(
        best[1], tuple(round(to_mm(v), 2) for v in bounds)), flush=True)
    return expand_rectangle(bounds, ESP_CLEARANCE)


ESP_SUPPORT_REFS = {"U2", "Y1", "R25", "C18", "C19", "C20", "L2", "C21", "C22"}


def place_esp_support_components(board, nets, prepared, esp_item, report, audit_errors):
    """Place external flash, crystal, decoupling, RES12K and RF matching parts."""
    by_ref = {item["reference"]: item for item in prepared}
    fixed = {
        "U2": (27.5, 55.5, 0), "Y1": (30.5, 46.5, 0),
        "R25": (32.8, 44.0, 0), "C18": (34.2, 55.5, 0),
        "C19": (33.1, 48.5, 90), "C20": (33.1, 44.5, 90),
    }
    placed = []
    for ref, (x, y, angle) in fixed.items():
        item = by_ref[ref]
        item["side"] = "B"
        fp = item["footprint"]
        _flip_footprint_to_back(board, fp, ref)
        fp.SetOrientationDegrees(angle)
        bounds = _set_footprint_center(fp, x, y)
        if not rectangle_inside_board(bounds):
            raise RuntimeError("{} is outside the board".format(ref))
        item["local_bounds"] = bounds
        item["width"], item["height"] = rectangle_dimensions(bounds)
        assign_component_nets(item, nets, report, audit_errors)
        placed.append(item)
    lna = find_footprint_pad(esp_item["footprint"], "2").GetPosition()
    lx, ly = to_mm(lna.x), to_mm(lna.y)
    for ref, x, y, angle in (
        ("L2", lx + 1.25, ly, 0),
        ("C21", lx + 0.55, ly + 1.05, 90),
        ("C22", lx + 1.95, ly + 1.05, 90),
    ):
        item = by_ref[ref]
        item["side"] = "B"
        fp = item["footprint"]
        _flip_footprint_to_back(board, fp, ref)
        fp.SetOrientationDegrees(angle)
        bounds = _set_footprint_center(fp, x, y)
        if not rectangle_inside_board(bounds):
            raise RuntimeError("{} RF matching footprint is outside the board".format(ref))
        item["local_bounds"] = bounds
        item["width"], item["height"] = rectangle_dimensions(bounds)
        assign_component_nets(item, nets, report, audit_errors)
        placed.append(item)
    report.append("ESP support group: W25Q32, 26 MHz crystal, load capacitors, decoupling and 12k RES12K")
    report.append("RF pi network: L2=0R initial series link; C21/C22 DNP pending VNA tuning")
    return placed

def place_fixed_usb(board, nets, usb, report, audit_errors):
    """Place mirrored USB-C J1 on B.Cu at lower-left, opening facing left."""
    usb["side"] = "B"
    footprint = usb["footprint"]

    # A B.Cu left-right flip reverses the apparent horizontal direction.
    # Use 90 degrees before flipping (the former 270-degree setting faced
    # right) so the physical receptacle faces the left board edge.
    footprint.SetOrientationDegrees(90)
    footprint.SetLayer(pcbnew.F_Cu)
    footprint.SetPosition(point(0, 0))
    board.Add(footprint)

    flip_method = getattr(footprint, "SetLayerAndFlip", None)
    if callable(flip_method):
        flip_method(pcbnew.B_Cu)
    else:
        flip_direction = getattr(pcbnew, "FLIP_DIRECTION_LEFT_RIGHT", None)
        if flip_direction is None:
            raise RuntimeError("KiCad pcbnew has no supported footprint flip API for J1")
        footprint.Flip(point(0, 0), flip_direction)
        footprint.SetLayer(pcbnew.B_Cu)
    if footprint.GetLayer() != pcbnew.B_Cu:
        raise RuntimeError("USB-C J1 could not be moved to B.Cu")

    usb["local_bounds"] = footprint_placement_bounds(footprint)
    width, height = rectangle_dimensions(usb["local_bounds"])
    target_x = mm(USB_LEFT_EDGE_MARGIN)

    # Keep J1 at the lower-left edge, facing left. The PCB antenna is now
    # reserved at the upper-left edge, so the USB connector uses the bottom
    # edge anchor instead of the former lower-antenna-relative coordinate.
    target_y = mm(PCB_HEIGHT - BOARD_MARGIN) - height
    place_by_bounds(usb, target_x, target_y)

    actual = footprint_placement_bounds(footprint)
    if not rectangle_inside_board(actual):
        raise RuntimeError(
            "Mirrored USB-C J1 does not fit at the lower-left edge on B.Cu. "
            "Measured envelope: {} x {} mm".format(
                round(to_mm(actual[2]-actual[0]), 2),
                round(to_mm(actual[3]-actual[1]), 2)
            )
        )

    assign_component_nets(usb, nets, report, audit_errors)
    print(
        "Fixed mirrored USB-C J1 on B.Cu at left edge above antenna, receptacle facing left:",
        round(to_mm(actual[0]), 2), round(to_mm(actual[1]), 2),
        "orientation 90 degrees before B.Cu flip"
    )
    return expand_rectangle(actual, COMPONENT_CLEARANCE)


def place_fixed_solenoid(
    board, nets, solenoid, led, report, audit_errors
):
    """Place J2 directly above D3, centred on the LED's X coordinate."""
    solenoid["side"] = "F"
    footprint = solenoid["footprint"]
    footprint.SetOrientationDegrees(0)
    footprint.SetLayer(pcbnew.F_Cu)
    board.Add(footprint)

    # Use the actual transformed LED envelope and J2's measured envelope.
    # KiCad coordinates increase downwards, so a smaller Y is above the LED.
    led_bounds = footprint_placement_bounds(led["footprint"])
    j2_width, j2_height = rectangle_dimensions(solenoid["local_bounds"])
    led_centre_x = (led_bounds[0] + led_bounds[2]) // 2
    target_left = led_centre_x - j2_width // 2
    target_top = led_bounds[1] - mm(COMPONENT_CLEARANCE) - j2_height

    place_by_bounds(
        solenoid,
        target_left,
        target_top
    )

    actual = footprint_placement_bounds(footprint)
    if not rectangle_inside_board(actual):
        raise RuntimeError(
            "Solenoid connector J2 does not fit directly above central LED D3. "
            "Measured envelope: {} x {} mm; calculated top-left: ({}, {}). "
            "Move the LED or revise the board-edge clearance.".format(
                round(to_mm(actual[2]-actual[0]), 2),
                round(to_mm(actual[3]-actual[1]), 2),
                round(to_mm(actual[0]), 2),
                round(to_mm(actual[1]), 2)
            )
        )

    # Confirm the requested vertical relationship and the physical gap.
    if rectangle_overlap(
        actual,
        expand_rectangle(led_bounds, COMPONENT_CLEARANCE)
    ):
        raise RuntimeError(
            "Solenoid connector J2 overlaps the clearance envelope of LED D3."
        )

    assign_component_nets(solenoid, nets, report, audit_errors)
    print(
        "Fixed solenoid connector J2 directly above central RGB LED D3:",
        round(to_mm(actual[0]), 2),
        round(to_mm(actual[1]), 2),
        "X-centred on LED; orientation 0 degrees"
    )
    return expand_rectangle(actual, COMPONENT_CLEARANCE)


def pack_side(
    board,
    nets,
    prepared,
    side,
    report,
    audit_errors,
    fixed_rectangles=None,
    fixed_items=None
):
    if fixed_rectangles is None:
        fixed_rectangles = []
    if fixed_items is None:
        fixed_items = []

    side = str(side).upper()
    if side not in ("F", "B"):
        raise RuntimeError("Unsupported component side: {}".format(side))

    items = [item for item in prepared if item["side"] == side]

    print(
        "\nPacking side:",
        "F.Cu" if side == "F" else "B.Cu",
        "Components:",
        len(items)
    )

    # Attach every footprint to the BOARD before measuring its complete
    # physical envelope. This is retained for compatibility with KiCad 10
    # footprint geometry and collision checking. This revision sends all
    # component footprints through the F.Cu path; no B.Cu flips are requested.
    for item in items:
        item["side"] = side
        footprint = item["footprint"]

        footprint.SetLayer(pcbnew.F_Cu)
        footprint.SetPosition(point(0, 0))

        # KiCad 10 may crash when SetLayerAndFlip() is called on a detached
        # custom footprint, so attach it before mirroring.
        board.Add(footprint)

        if side == "B":
            print(
                "Flipping {} to B.Cu after BOARD.Add()".format(
                    item["reference"]
                ),
                flush=True
            )

            set_layer_and_flip = getattr(
                footprint, "SetLayerAndFlip", None
            )

            if callable(set_layer_and_flip):
                set_layer_and_flip(pcbnew.B_Cu)
            else:
                flip_direction = getattr(
                    pcbnew, "FLIP_DIRECTION_LEFT_RIGHT", None
                )
                if flip_direction is None:
                    raise RuntimeError(
                        "KiCad pcbnew has no supported footprint flip API "
                        "for {}".format(item["reference"])
                    )
                footprint.Flip(point(0, 0), flip_direction)
                footprint.SetLayer(pcbnew.B_Cu)

            if footprint.GetLayer() != pcbnew.B_Cu:
                raise RuntimeError(
                    "Back-side flip failed for {}".format(
                        item["reference"]
                    )
                )
        else:
            footprint.SetLayer(pcbnew.F_Cu)

        # Always refresh the complete physical envelope after board
        # attachment and side transformation. This includes courtyard,
        # fab/silkscreen outlines and pads, and is reused by the final audit.
        item["local_bounds"] = footprint_placement_bounds(footprint)
        item["width"], item["height"] = rectangle_dimensions(
            item["local_bounds"]
        )
        item["area"] = item["width"] * item["height"]

        print(
            "Measured {} after {} transform: {:.2f} x {:.2f} mm".format(
                item["reference"],
                "B.Cu" if side == "B" else "F.Cu",
                to_mm(item["width"]),
                to_mm(item["height"])
            ),
            flush=True
        )

    # Assign nets before planning so the placer can cluster components by
    # their actual electrical connections, not merely by physical size.
    for item in items:
        assign_component_nets(item, nets, report, audit_errors)

    # Only now, with final transformed bounds, generate and test packing plans.
    # First place parts with dedicated nets to the fixed ESP/connector anchors.
    # The ESP8266EX integrates its flash and crystal, so only external GPIO, reset,
    # battery-sense and power connections remain to be routed.
    common_rails = {"GND", "3V3", "BAT+", "BAT-", "USB_VBUS"}
    fixed_net_names = set()
    for fixed_item in fixed_items:
        for net_name, _, _ in _item_pad_positions(fixed_item, None):
            if net_name not in common_rails:
                fixed_net_names.add(net_name)

    def fixed_signal_affinity(item):
        item_nets = {
            net_name for net_name, _, _ in _item_pad_positions(item, None)
            if net_name not in common_rails
        }
        shared = item_nets.intersection(fixed_net_names)
        return (len(shared), item["area"])

    priority_order = sorted(items, key=fixed_signal_affinity, reverse=True)
    packing_orders = [priority_order] + list(generate_packing_orders(items))
    successful_plan = None

    for attempt_number, order in enumerate(
        packing_orders,
        1
    ):
        plan = attempt_pack(
            order,
            side,
            fixed_rectangles,
            fixed_items
        )

        if plan is not None:
            successful_plan = plan
            print("Packing attempt:", attempt_number)
            break

    if successful_plan is None:
        raise RuntimeError(
            "Could not fit all components on {}. ".format(
                "F.Cu" if side == "F" else "B.Cu"
            )
            + "No PCB was saved. The placement algorithm will not silently "
            "overlap components. All footprints remain included; inspect "
            "the transformed courtyard dimensions and board reservations."
        )

    # Apply the plan using each transformed footprint's measured local
    # courtyard origin. Then immediately audit actual post-placement bounds.
    for item in items:
        x1, y1, _, _ = successful_plan[item["reference"]]

        place_by_bounds(item, x1, y1)

        actual_bounds = footprint_placement_bounds(item["footprint"])
        if not rectangle_inside_board(actual_bounds):
            raise RuntimeError(
                "{} is outside the board after placement.".format(
                    item["reference"]
                )
            )

        print(
            "Placed",
            item["reference"],
            round(to_mm(x1), 2),
            round(to_mm(y1), 2),
            "F.Cu" if side == "F" else "B.Cu"
        )

    # Independent same-side collision check on the actual transformed and
    # positioned footprints. This catches any disagreement between the
    # planner's rectangles and pcbnew's final courtyard bounding boxes.
    for index, item_a in enumerate(items):
        rect_a = expand_rectangle(
            footprint_placement_bounds(item_a["footprint"]),
            COMPONENT_CLEARANCE / 2
        )

        for item_b in items[index + 1:]:
            rect_b = expand_rectangle(
                footprint_placement_bounds(item_b["footprint"]),
                COMPONENT_CLEARANCE / 2
            )

            if rectangle_overlap(rect_a, rect_b):
                raise RuntimeError(
                    "Post-placement courtyard collision on {}: {} / {}".format(
                        "F.Cu" if side == "F" else "B.Cu",
                        item_a["reference"],
                        item_b["reference"]
                    )
                )

    return items


# ================================================================
# CENTRAL LED
# ================================================================

def place_central_led(board, nets, led, report, audit_errors):
    """Place the fixed central LED on B.Cu and measure its mirrored envelope."""
    footprint = led["footprint"]
    led["side"] = "B"
    footprint.SetLayer(pcbnew.F_Cu)
    footprint.SetPosition(point(LED_CENTRE_X, LED_CENTRE_Y))
    board.Add(footprint)

    flip_method = getattr(footprint, "SetLayerAndFlip", None)
    if callable(flip_method):
        flip_method(pcbnew.B_Cu)
    else:
        flip_direction = getattr(pcbnew, "FLIP_DIRECTION_LEFT_RIGHT", None)
        if flip_direction is None:
            raise RuntimeError("KiCad pcbnew has no supported footprint flip API for D3")
        footprint.Flip(footprint.GetPosition(), flip_direction)
        footprint.SetLayer(pcbnew.B_Cu)

    if footprint.GetLayer() != pcbnew.B_Cu:
        raise RuntimeError("Central LED D3 could not be moved to B.Cu")

    bounds = footprint_placement_bounds(footprint)
    if not rectangle_inside_board(bounds):
        raise RuntimeError("Central LED does not fit on the board after B.Cu flip.")

    assign_component_nets(led, nets, report, audit_errors)
    print("Central WS2812B anchor on B.Cu:", LED_CENTRE_X, LED_CENTRE_Y)
    return expand_rectangle(bounds, COMPONENT_CLEARANCE)


# ================================================================
# FINAL PLACEMENT AUDIT
# ================================================================

def verify_placement(placed, led, solenoid_row_band):
    all_items = list(placed) + [led]

    # Verify that every footprint is on its assigned copper side.
    for item in all_items:
        expected_layer = (
            pcbnew.F_Cu if item["side"] == "F" else pcbnew.B_Cu
        )
        if item["side"] not in ("F", "B"):
            raise RuntimeError(
                "{} has invalid side {}.".format(
                    item["reference"], item["side"]
                )
            )
        if item["footprint"].GetLayer() != expected_layer:
            raise RuntimeError(
                "{} footprint layer does not match {}.".format(
                    item["reference"], item["side"]
                )
            )

    # Validate physical collisions across the board and keep the complete
    # horizontal solenoid lane clear of every other component footprint.
    for index, item_a in enumerate(all_items):
        rect_a = footprint_placement_bounds(item_a["footprint"])
        if not rectangle_inside_board(rect_a):
            raise RuntimeError(
                "{} is outside the board.".format(item_a["reference"])
            )
        if item_a["reference"] != "J2" and rectangle_overlap(
            rect_a,
            solenoid_row_band
        ):
            raise RuntimeError(
                "{} occupies the reserved solenoid row. Move it outside "
                "the full-width J2 placement lane.".format(item_a["reference"])
            )

        for item_b in all_items[index+1:]:
            rect_b = footprint_placement_bounds(item_b["footprint"])
            clearance_a = (
                ESP_CLEARANCE
                if item_a["reference"] == "U1"
                else COMPONENT_CLEARANCE / 2.0
            )
            clearance_b = (
                ESP_CLEARANCE
                if item_b["reference"] == "U1"
                else COMPONENT_CLEARANCE / 2.0
            )
            a_clear = expand_rectangle(rect_a, clearance_a)
            b_clear = expand_rectangle(rect_b, clearance_b)
            if rectangle_overlap(a_clear, b_clear):
                if item_a["side"] != item_b["side"]:
                    collision_type = "Opposite-side physical overlap"
                else:
                    collision_type = "Same-side courtyard collision ({})".format(
                        "F.Cu" if item_a["side"] == "F" else "B.Cu"
                    )
                raise RuntimeError(
                    "{}: {} / {}. Components on opposite sides must not "
                    "overlap through the PCB.".format(
                        collision_type,
                        item_a["reference"],
                        item_b["reference"]
                    )
                )


    # There are no separate board-corner M2 mounting holes in this revision.
    # Mechanical/alignment holes inside connector footprints are nevertheless
    # protected because the complete connector footprint envelope is reserved
    # against components on BOTH board sides during packing and final audit.
    hole_rectangles = mounting_hole_keepouts()
    for item in all_items:
        item_clearance = ESP_CLEARANCE if item["reference"] == "U1" else COMPONENT_CLEARANCE/2
        rect = expand_rectangle(
            footprint_placement_bounds(item["footprint"]),
            item_clearance
        )
        for hole_rect in hole_rectangles:
            if rectangle_overlap(rect, hole_rect):
                raise RuntimeError(
                    "Mounting-hole clearance collision: {}".format(
                        item["reference"]
                    )
                )

    # Verify central LED anchor remains fixed at board centre on B.Cu.
    if led["side"] != "B":
        raise RuntimeError("Central LED must remain on B.Cu.")
    led_pos = led["footprint"].GetPosition()
    if (
        abs(to_mm(led_pos.x)-LED_CENTRE_X) > 0.01
        or abs(to_mm(led_pos.y)-LED_CENTRE_Y) > 0.01
    ):
        raise RuntimeError("Central LED moved from its fixed anchor.")


# ================================================================
# BOARD OUTLINE
# ================================================================

def add_line(board, x1, y1, x2, y2):
    line = pcbnew.PCB_SHAPE(board)

    line.SetShape(pcbnew.SHAPE_T_SEGMENT)
    line.SetStart(point(x1, y1))
    line.SetEnd(point(x2, y2))
    line.SetLayer(pcbnew.Edge_Cuts)
    line.SetWidth(mm(0.05))

    board.Add(line)


def create_outline(board):
    corners = [
        (0, 0),
        (PCB_WIDTH, 0),
        (PCB_WIDTH, PCB_HEIGHT),
        (0, PCB_HEIGHT)
    ]

    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1) % 4]

        add_line(
            board,
            x1, y1,
            x2, y2
        )


def create_board_edge_keepouts(board):
    """Keep FreeRouting copper at least 0.50 mm from every board edge.

    These are ordinary rectangular rule areas (no polygon holes), exported
    on both copper layers. The 0.60 mm centreline reservation accounts for
    the 0.20 mm routed track width plus the required 0.50 mm copper clearance.
    """
    margin = BOARD_EDGE_ROUTING_MARGIN
    rectangles = [
        (0.0, 0.0, PCB_WIDTH, margin),
        (0.0, PCB_HEIGHT-margin, PCB_WIDTH, PCB_HEIGHT),
        (0.0, margin, margin, PCB_HEIGHT-margin),
        (PCB_WIDTH-margin, margin, PCB_WIDTH, PCB_HEIGHT-margin),
    ]
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        for x1, y1, x2, y2 in rectangles:
            zone = pcbnew.ZONE(board)
            zone.SetLayer(layer)
            if hasattr(zone, "SetIsRuleArea"):
                zone.SetIsRuleArea(True)
            for method_name in (
                "SetDoNotAllowTracks", "SetDoNotAllowVias",
                "SetDoNotAllowFootprints", "SetDoNotAllowZoneFills"
            ):
                method = getattr(zone, method_name, None)
                if callable(method):
                    method(True)
            outline = zone.Outline()
            outline.NewOutline()
            for x, y in ((x1,y1),(x2,y1),(x2,y2),(x1,y2)):
                outline.Append(int(mm(x)), int(mm(y)))
            board.Add(zone)
    print("Created 0.60 mm board-edge routing keepouts on F.Cu and B.Cu")


# ================================================================
# MOUNTING HOLES
# ================================================================

def create_mounting_holes(board):
    for reference, x, y in MOUNT_HOLES:
        footprint = load_footprint(
            "MountingHole",
            "MountingHole_2.2mm"
        )

        footprint.SetReference(reference)
        footprint.SetValue("M2 NPTH")
        footprint.SetPosition(point(x, y))

        board.Add(footprint)


# ================================================================
# TRACK HELPERS
# ================================================================

def add_net_track(
    board,
    net,
    start,
    end,
    width,
    layer=pcbnew.F_Cu
):
    track = pcbnew.PCB_TRACK(board)

    track.SetStart(start)
    track.SetEnd(end)
    track.SetWidth(mm(width))
    track.SetLayer(layer)
    track.SetNet(net)

    board.Add(track)

    return track


def add_polyline(
    board,
    net,
    coordinates,
    width,
    layer=pcbnew.F_Cu
):
    tracks = []

    for index in range(len(coordinates)-1):
        x1, y1 = coordinates[index]
        x2, y2 = coordinates[index+1]

        tracks.append(
            add_net_track(
                board,
                net,
                point(x1, y1),
                point(x2, y2),
                width,
                layer
            )
        )

    return tracks


def add_gnd_via(board, net, x, y, size=0.60, drill=0.30):
    """Add a through via that ties the two GND copper layers together."""
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(point(x, y))
    via.SetWidth(mm(size))
    via.SetDrill(mm(drill))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    if hasattr(via, "SetViaType") and hasattr(pcbnew, "VIATYPE_THROUGH"):
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetNet(net)
    board.Add(via)
    return via


def create_gnd_stitching(board, nets, report):
    """Add ground stitching vias so the two GND pours are electrically joined.

    The previous generator only described 14 stitching vias in its launcher;
    it did not actually place any.  That left the F.Cu and B.Cu GND zone fills
    as separate copper islands, which KiCad reported as unconnected zone
    items.  Place real through-vias in clear board areas, while keeping a wider
    exclusion around U1 so its ESP8266EX bare-chip design is not obstructed.
    """
    obstacles = []
    for footprint in board.GetFootprints():
        bounds = footprint_placement_bounds(footprint)
        reference = footprint.GetReference()
        clearance = 4.0 if reference == "U1" else 0.75
        obstacles.append(expand_rectangle(bounds, clearance))

    candidates = []
    # A 1 mm search grid gives enough choice to distribute the stitches.
    for yi in range(2, int(PCB_HEIGHT - 1)):
        y = yi + 0.5
        # Keep the reserved full-width J2 connector lane free of vias.
        if 22.5 <= y <= 28.5:
            continue
        for xi in range(2, int(PCB_WIDTH - 1)):
            x = xi + 0.5
            via_box = (
                mm(x - 0.35), mm(y - 0.35),
                mm(x + 0.35), mm(y + 0.35)
            )
            if any(rectangle_overlap(via_box, obstacle) for obstacle in obstacles):
                continue
            # Stay inside the copper-zone outline and away from board edges.
            if x < 1.5 or x > PCB_WIDTH - 1.5 or y < 1.5 or y > PCB_HEIGHT - 1.5:
                continue
            candidates.append((x, y))

    selected = []
    # Seed near the board centre, then greedily spread the remaining vias
    # across the available copper rather than clustering them together.
    while candidates and len(selected) < 14:
        if not selected:
            chosen = min(
                candidates,
                key=lambda q: (q[0] - PCB_WIDTH / 2.0) ** 2
                + (q[1] - PCB_HEIGHT / 2.0) ** 2
            )
        else:
            chosen = max(
                candidates,
                key=lambda q: min(
                    (q[0] - px) ** 2 + (q[1] - py) ** 2
                    for px, py in selected
                )
            )
        selected.append(chosen)
        candidates.remove(chosen)
        candidates = [
            q for q in candidates
            if (q[0] - chosen[0]) ** 2 + (q[1] - chosen[1]) ** 2 >= 16.0
        ]

    for x, y in selected:
        add_gnd_via(board, nets["GND"], x, y, 0.60, 0.30)

    report.append(
        "GND stitching: {} real 0.60/0.30 mm through-vias added outside component and ESP8266EX antenna clearances".format(
            len(selected)
        )
    )
    if len(selected) < 8:
        print(
            "WARNING: only {} GND stitching vias could be placed; inspect remaining zone islands".format(
                len(selected)
            )
        )
    else:
        print("Added {} distributed GND stitching vias:".format(len(selected)), selected)



# ================================================================
# FOOTPRINT-SPECIFIC DRC CLEARANCES
# ================================================================

def apply_drc_clearance_overrides(board):
    """Apply narrow, documented overrides for the selected footprints."""
    settings = board.GetDesignSettings()
    hole_clearance_applied = False

    # KiCad 10 exposes the board-level hole-to-copper constraint through
    # BOARD_DESIGN_SETTINGS.m_HoleClearance. Retain the normal 0.25 mm rule;
    # the USB-C contact pad geometry is modified below to meet it.
    try:
        settings.m_HoleClearance = mm(USB_NPTH_HOLE_CLEARANCE)
        hole_clearance_applied = True
    except Exception:
        pass

    if not hole_clearance_applied:
        try:
            settings.SetHoleClearance(mm(USB_NPTH_HOLE_CLEARANCE))
            hole_clearance_applied = True
        except Exception:
            pass

    if not hole_clearance_applied:
        print(
            "WARNING: Could not set KiCad board hole clearance to {:.2f} mm; "
            "check the KiCad 10 Python API property name.".format(
                USB_NPTH_HOLE_CLEARANCE
            )
        )
    else:
        print(
            "Board hole-to-copper clearance set to {:.2f} mm for J1 footprint fit".format(
                USB_NPTH_HOLE_CLEARANCE
            )
        )

    # Keep the 0.25 mm hole-to-copper rule. Shorten the four USB-C ground
    # contacts A1/A12/B1/B12 along their hole-facing axis so the physical
    # copper-to-hole gap meets the rule instead of suppressing it.
    usb_ground_pads_changed = 0
    for footprint in board.GetFootprints():
        if footprint.GetReference() != "J1":
            continue
        for pad in footprint_pad_items(footprint):
            if str(pad.GetNumber()).strip() not in {"A1", "A12", "B1", "B12"}:
                continue
            size = pad.GetSize()
            new_size = pcbnew.VECTOR2I(int(size.x), int(mm(USB_GND_CONTACT_LENGTH)))
            pad.SetSize(new_size)
            usb_ground_pads_changed += 1
    print("Shortened {} J1 GND contacts to {:.2f} mm for 0.25 mm hole clearance".format(
        usb_ground_pads_changed, USB_GND_CONTACT_LENGTH
    ))

    # Apply the normal 0.25 mm local clearance to J1's NPTH locating pads.
    usb_holes_changed = 0
    for footprint in board.GetFootprints():
        if footprint.GetReference() != "J1":
            continue
        for pad in footprint_pad_items(footprint):
            is_npth = False
            try:
                is_npth = pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH
            except Exception:
                pass
            if is_npth:
                local_clearance = getattr(pad, "SetLocalClearance", None)
                if callable(local_clearance):
                    local_clearance(mm(USB_NPTH_HOLE_CLEARANCE))
                    usb_holes_changed += 1
    print("Applied {:.2f} mm local clearance to {} J1 NPTH locating pads".format(
        USB_NPTH_HOLE_CLEARANCE, usb_holes_changed
    ))

    target_refs = {"U3", "U7"}
    target_pads = {"9", "10", "11"}
    changed = 0
    for footprint in board.GetFootprints():
        reference = footprint.GetReference()
        if reference not in target_refs:
            continue
        for pad in footprint_pad_items(footprint):
            if str(pad.GetNumber()).strip() not in target_pads:
                continue
            try:
                pad.SetLocalClearance(mm(TPS63070_ADJACENT_PAD_CLEARANCE))
                changed += 1
            except Exception as exc:
                raise RuntimeError(
                    "Could not apply TPS63070 local pad clearance to {} pad {}: {}".format(
                        reference, pad.GetNumber(), exc
                    )
                )

    print(
        "Applied {:.2f} mm local clearance to {} adjacent TPS63070 pads on U3/U7".format(
            TPS63070_ADJACENT_PAD_CLEARANCE, changed
        )
    )


def apply_routing_constraints(board):
    """Set every available netclass to the board's 0.20 mm track minimum."""
    settings = board.GetDesignSettings()
    changed = False

    def set_class_width(netclass):
        nonlocal changed
        if netclass is None:
            return
        setter = getattr(netclass, "SetTrackWidth", None)
        if callable(setter):
            setter(mm(ROUTED_TRACK_WIDTH))
            changed = True
        clearance_setter = getattr(netclass, "SetClearance", None)
        if callable(clearance_setter):
            clearance_setter(mm(0.25))
        # Export the same manufacturable via dimensions expected by the board
        # setup. FreeRouting's 0.45/0.20 mm defaults caused the reported DRCs.
        for method_name, value in (
            ("SetViaDiameter", ROUTED_VIA_DIAMETER),
            ("SetViaDrill", ROUTED_VIA_DRILL),
        ):
            method = getattr(netclass, method_name, None)
            if callable(method):
                method(mm(value))

    default_getter = getattr(settings, "GetDefaultNetClass", None)
    if callable(default_getter):
        try:
            set_class_width(default_getter())
        except Exception:
            pass

    # KiCad versions expose the netclass collection through different Python
    # wrappers. Update every class, not only the default class, so no signal
    # net is exported with FreeRouting's 0.15 mm fallback width.
    for getter_name in ("GetNetClasses", "GetNetClassMap"):
        getter = getattr(settings, getter_name, None)
        if not callable(getter):
            continue
        try:
            classes = getter()
            try:
                values = list(classes.values())
            except Exception:
                try:
                    values = list(classes)
                except Exception:
                    values = []
            for netclass in values:
                set_class_width(netclass)
        except Exception:
            pass

    # Set any direct board-level defaults/minimums exposed by this pcbnew build.
    for method_name in ("SetMinTrackWidth", "SetDefaultTrackWidth"):
        method = getattr(settings, method_name, None)
        if callable(method):
            try:
                method(mm(ROUTED_TRACK_WIDTH))
                changed = True
            except Exception:
                pass
    for attr_name in ("m_TrackMinWidth", "m_DefaultTrackWidth"):
        try:
            setattr(settings, attr_name, mm(ROUTED_TRACK_WIDTH))
            changed = True
        except Exception:
            pass

    if not changed:
        print("WARNING: pcbnew did not expose a netclass width setter; inspect DSN resolution rules.")
    else:
        print("All available netclasses and board defaults set to {:.2f} mm track width".format(
            ROUTED_TRACK_WIDTH
        ))



# ================================================================
# GND COPPER POURS
# ================================================================

def create_gnd_copper_pours(board, nets):
    """Create and refill GND zones on both F.Cu and B.Cu.

    The B.Cu zone clears around the RF antenna trace and feed by normal net
    clearance. The F.Cu antenna keepout prevents copper directly above the
    radiator. Both zones are inset from the board edge.
    """
    edge_inset = 0.50
    x1 = mm(edge_inset)
    y1 = mm(edge_inset)
    x2 = mm(PCB_WIDTH - edge_inset)
    y2 = mm(PCB_HEIGHT - edge_inset)

    created = []
    for layer, layer_name in (
        (pcbnew.F_Cu, "F.Cu"),
        (pcbnew.B_Cu, "B.Cu"),
    ):
        zone = pcbnew.ZONE(board)
        zone.SetLayer(layer)
        zone.SetNet(nets["GND"])
        zone.SetIsRuleArea(False)
        # Remove isolated same-net copper islands. The previous DRC report
        # listed 14 zone-to-zone unconnected island pairs at the board corner.
        island_setter = getattr(zone, "SetIslandRemovalMode", None)
        island_mode = getattr(pcbnew, "ISLAND_REMOVAL_MODE_ALWAYS", None)
        if callable(island_setter) and island_mode is not None:
            island_setter(island_mode)

        # Explicit clearances keep the pour away from signal pads and tracks.
        if hasattr(zone, "SetLocalClearance"):
            zone.SetLocalClearance(mm(0.25))
        if hasattr(zone, "SetMinThickness"):
            zone.SetMinThickness(mm(0.20))
        if hasattr(zone, "SetThermalReliefGap"):
            zone.SetThermalReliefGap(mm(0.25))
        if hasattr(zone, "SetThermalReliefSpokeWidth"):
            zone.SetThermalReliefSpokeWidth(mm(0.30))
        if hasattr(zone, "SetPadConnection"):
            if hasattr(pcbnew, "ZONE_CONNECTION_FULL"):
                # Solid connections eliminate starved thermal-relief errors
                # on the compact TPS63070 GND pad groups and improve ground
                # conduction into both copper planes.
                zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
            elif hasattr(pcbnew, "ZONE_CONNECTION_THERMAL"):
                zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)

        outline = zone.Outline()
        outline.NewOutline()
        for x, y in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
            outline.Append(int(x), int(y))

        board.Add(zone)
        created.append(zone)
        print("Created GND copper zone on", layer_name)

    # Refill now so the saved PCB contains actual zone geometry, not only
    # empty zone outlines. This is also what KiCad will use for pad connectivity.
    try:
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
    except Exception as exc:
        raise RuntimeError(
            "Could not refill the F.Cu/B.Cu GND copper zones: {}".format(exc)
        )

    print("Refilled GND copper zones on both F.Cu and B.Cu")
    return created


def add_3v3_plane_stitch_vias(board, net):
    """Find two clear sites and stitch the F.Cu/B.Cu 3V3 planes together."""
    obstacles = [
        expand_rectangle(footprint_placement_bounds(fp), 0.70)
        for fp in board.GetFootprints()
    ]
    candidates = []
    for yi in range(4, int(PCB_HEIGHT - 3)):
        for xi in range(4, int(PCB_WIDTH - 3)):
            x, y = float(xi), float(yi)
            # Keep the reserved solenoid connector row clear.
            if 22.5 <= y <= 28.5:
                continue
            via_box = (mm(x - 0.35), mm(y - 0.35), mm(x + 0.35), mm(y + 0.35))
            if any(rectangle_overlap(via_box, obstacle) for obstacle in obstacles):
                continue
            # Prefer central free copper, but avoid placing both stitches together.
            distance = (x - PCB_WIDTH / 2.0) ** 2 + (y - PCB_HEIGHT / 2.0) ** 2
            candidates.append((distance, x, y))
    candidates.sort()
    placed = []
    for _, x, y in candidates:
        if any((x-px)**2 + (y-py)**2 < 25.0 for px, py in placed):
            continue
        add_gnd_via(board, net, x, y, 0.60, 0.30)
        placed.append((x, y))
        if len(placed) == 2:
            break
    if len(placed) < 2:
        raise RuntimeError(
            "Could not find two clear locations for 3V3 plane stitching vias; "
            "the copper planes may remain as separate conduction areas."
        )
    print("Added two dynamically placed 3V3 plane stitching vias:", placed)
    return placed


def create_3v3_copper_pours(board, nets):
    """Create broad 3V3 planes on both copper layers to join the power pads.

    The module, LED, and regulator/passive network share the main 3V3 rail.
    These planes are
    now deliberately enabled; the normal net clearance and antenna keepouts
    still apply. No plane is created for 3V_SOLENOID or any switched net.
    """
    edge_inset = 0.65
    x1, y1 = mm(edge_inset), mm(edge_inset)
    x2, y2 = mm(PCB_WIDTH - edge_inset), mm(PCB_HEIGHT - edge_inset)
    created = []
    for layer, layer_name in ((pcbnew.F_Cu, "F.Cu"), (pcbnew.B_Cu, "B.Cu")):
        zone = pcbnew.ZONE(board)
        zone.SetLayer(layer)
        zone.SetNet(nets["3V3"])
        zone.SetIsRuleArea(False)
        if hasattr(zone, "SetLocalClearance"):
            zone.SetLocalClearance(mm(0.25))
        if hasattr(zone, "SetMinThickness"):
            zone.SetMinThickness(mm(0.20))
        if hasattr(zone, "SetThermalReliefGap"):
            zone.SetThermalReliefGap(mm(0.25))
        if hasattr(zone, "SetThermalReliefSpokeWidth"):
            zone.SetThermalReliefSpokeWidth(mm(0.30))
        if hasattr(zone, "SetPadConnection") and hasattr(pcbnew, "ZONE_CONNECTION_FULL"):
            zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        island_setter = getattr(zone, "SetIslandRemovalMode", None)
        island_mode = getattr(pcbnew, "ISLAND_REMOVAL_MODE_ALWAYS", None)
        if callable(island_setter) and island_mode is not None:
            island_setter(island_mode)
        outline = zone.Outline()
        outline.NewOutline()
        for x, y in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
            outline.Append(int(x), int(y))
        board.Add(zone)
        created.append(zone)
        print("Created 3V3 copper zone on", layer_name)
    add_3v3_plane_stitch_vias(board, nets["3V3"])
    try:
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    except Exception as exc:
        raise RuntimeError("Could not refill the 3V3 copper zones: {}".format(exc))
    print("Refilled 3V3 copper zones on both F.Cu and B.Cu")
    # Two through-vias stitch the broad 3V3 pours together so FreeRouting does
    # not leave separate same-net copper areas as an unroutable conductionarea airwire.
    for vx, vy in ((12.0, 32.0), (34.0, 32.0)):
        add_gnd_via(board, nets["3V3"], vx, vy, 0.60, 0.30)
    try:
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    except Exception as exc:
        raise RuntimeError("Could not refill copper zones after 3V3 stitching: {}".format(exc))
    print("Added two 3V3 through-vias to stitch the F.Cu/B.Cu power pours")
    return created


# ================================================================
# SILKSCREEN
# ================================================================

def add_text(
    board,
    text,
    x,
    y,
    size=0.8,
    layer=None
):
    if layer is None:
        layer = pcbnew.F_SilkS

    item = pcbnew.PCB_TEXT(board)

    item.SetText(text)
    item.SetPosition(point(x, y))
    item.SetTextSize(point(size, size))
    item.SetTextThickness(mm(0.12))
    item.SetLayer(layer)

    board.Add(item)


def create_labels(board):
    add_text(
        board, "SMARTLOCK",
        17.0, 60.0, 0.9
    )

    add_text(
        board, "PCB ANTENNA KEEP CLEAR",
        40.2, 40.0, 0.45
    )

    add_text(
        board, "BATTERY",
        17.0, 60.0, 0.65,
        pcbnew.B_SilkS
    )


# ================================================================
# BOM
# ================================================================

def write_bom():
    with open(
        BOM_FILE,
        "w",
        newline="",
        encoding="utf-8"
    ) as f:
        writer = csv.writer(f)

        writer.writerow([
            "Reference",
            "Value",
            "Manufacturer Part Number",
            "Footprint",
            "Side"
        ])

        for data in COMPONENTS:
            ref, value, library, name, rotation, original_side, mpn = data

            resolved_library, resolved_name, _ = (
                resolve_footprint(library, name)
            )

            footprint_label = resolved_library + ":" + resolved_name
            if ref == "U1":
                footprint_label += " (ESP8266EX QFN32 bare chip)"
            writer.writerow([
                ref,
                value,
                mpn,
                footprint_label,
                str(original_side).upper()
            ])

        writer.writerow([
            "D3",
            "WS2812B-V6 3V3 RGB",
            "WS2812B-V6",
            "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm",
            "F"
        ])


# ================================================================
# PAD REPORT
# ================================================================

def write_report(report, audit_errors):
    with open(
        REPORT_FILE,
        "w",
        encoding="utf-8"
    ) as f:
        f.write("SMARTLOCK PAD ASSIGNMENT REPORT\n")
        f.write("================================\n\n")

        front_count = sum(1 for data in COMPONENTS if str(data[5]).upper() == "F")
        back_count = sum(1 for data in COMPONENTS if str(data[5]).upper() == "B") + 1
        f.write("COMPONENT SIDES: TWO-SIDED\n")
        f.write("FRONT-SIDE COMPONENTS: {} (plus central LED D3 is on B.Cu)\n".format(front_count))
        f.write("BACK-SIDE COMPONENTS: {} including central LED D3\n\n".format(back_count))

        for line in report:
            f.write(line + "\n")

        f.write("\nGND PAD ASSIGNMENTS\n")
        f.write("====================\n")
        gnd_lines = [line for line in report if line.startswith("GND PAD VERIFIED:")]
        for line in gnd_lines:
            f.write(line + "\n")
        f.write("TOTAL COPPER PADS ASSIGNED TO GND: {}\n".format(len(gnd_lines)))
        f.write("NOTE: These are pad-to-net assignments, not completed copper routes.\n")
        f.write("\nUNASSIGNED COPPER PADS\n")
        f.write("======================\n")

        if audit_errors:
            for error in audit_errors:
                f.write(error + "\n")
        else:
            f.write(
                "No unassigned copper pads detected.\n"
            )

        f.write("\nR19: 470 ohms.\n")
        f.write(
            "U1 ESP8266EX QFN32 with external W25Q32 flash, 26MHz crystal, RES12K resistor and RF pi network.\n"
        )
        f.write(
            "DW01A OD: connected to FS8205A G1, pad 4.\n"
        )
        f.write(
            "DW01A OC: connected to FS8205A G2, pad 5.\n"
        )
        f.write(
            "J1 USB-C shell tabs S1-S4 / SH1-SH4: assigned to GND.\n"
        )
        f.write(
            "Non-copper paste-only pads excluded from audit.\n"
        )
        f.write(
            "Non-plated mechanical holes excluded from audit.\n"
            "Every copper pad must have a PAD_NETS entry; intentional NC entries are documented and left netless.\n"
        )
        f.write(
            "RF: B.Cu printed right-edge antenna; L2 series link and C21/C22 DNP shunt matching positions.\n"
        )
        f.write(
            "Copper pours: DISABLED; no copper zones are generated.\n"
        )
        f.write(
            "ESP8266EX printed antenna: maintain the F.Cu keepout and tune against the final enclosure.\n"
        )
        f.write(
            "A net assignment is not a copper route.\n"
        )
        f.write(
            "No copper zones or pours are generated; all connections must be routed with tracks and vias.\n"
            "All non-anchor components are placed below central LED D3; solenoid connector J2 remains above D3.\n"
            "ESP GPIO/reset and battery-sense resistor networks R3-R9 and R21-R24 are assigned to B.Cu.\n"
        )
        f.write(
            "Antenna geometry and matching values are prototype only and have NOT been RF tuned.\n"
        )
        f.write(
            "All non-anchor components are placed below central LED D3; solenoid connector J2 remains above D3.\n"
            "ESP GPIO/reset and battery-sense resistor networks R3-R9 and R21-R24 are assigned to B.Cu.\n"
        )


# ================================================================
# FREEROUTING DSN EXPORT
# ================================================================

def enforce_dsn_minimum_track_width(dsn_path):
    """Raise every Specctra (width ...) rule below 0.20 mm to 0.20 mm."""
    with open(dsn_path, "r", encoding="utf-8", errors="replace") as handle:
        source = handle.read()
    changed = 0
    pattern = re.compile(r"(\(width\s+)([0-9]+(?:\.[0-9]*)?)(\s*\))", re.IGNORECASE)
    def replace_width(match):
        nonlocal changed
        value = float(match.group(2))
        if value + 1e-9 < ROUTED_TRACK_WIDTH:
            changed += 1
            return match.group(1) + "{:.3f}".format(ROUTED_TRACK_WIDTH) + match.group(3)
        return match.group(0)
    updated = pattern.sub(replace_width, source)
    if changed:
        with open(dsn_path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(updated)
    print("DSN track-width rules normalized: {} rule(s) raised to {:.2f} mm".format(
        changed, ROUTED_TRACK_WIDTH
    ))

def export_dsn(board):
    """Export a FreeRouting DSN from the saved board using an absolute path.

    KiCad 10's standalone pcbnew overload is
    ExportSpecctraDSN(BOARD, full_filename).  The C++ wrapper catches
    exporter exceptions and reports only False, so retry from the saved PCB
    object as well as the in-memory board.  This also avoids depending on the
    process working directory when the generator is launched from KiCad's
    installation folder. Each run uses a timestamped DSN filename, avoiding
    Windows sharing violations when an earlier DSN remains open in FreeRouting.
    """
    print("\nExporting FreeRouting DSN...")

    exporter = getattr(pcbnew, "ExportSpecctraDSN", None)
    if exporter is None:
        raise RuntimeError(
            "This KiCad pcbnew installation does not expose ExportSpecctraDSN. "
            "The PCB has been saved, but a DSN file could not be generated."
        )

    dsn_path = os.path.abspath(os.path.normpath(DSN_FILE))
    pcb_path = os.path.abspath(os.path.normpath(OUTPUT_FILE))
    dsn_dir = os.path.dirname(dsn_path)
    os.makedirs(dsn_dir, exist_ok=True)

    attempts = []

    # Attempt 1: export the board currently being generated.
    try:
        result = exporter(board, dsn_path)
        if result and os.path.isfile(dsn_path) and os.path.getsize(dsn_path) > 0:
            enforce_dsn_minimum_track_width(dsn_path)
            print("DSN exported:", dsn_path)
            write_freerouting_all_nets_instructions()
            print("FreeRouting all-nets command:", os.path.abspath(FREEROUTING_COMMAND_FILE))
            return
        attempts.append(
            "in-memory board export returned {!r}{}".format(
                result,
                " and produced no non-empty DSN" if not (
                    os.path.isfile(dsn_path) and os.path.getsize(dsn_path) > 0
                ) else ""
            )
        )
    except Exception as exc:
        attempts.append("in-memory board export raised: {}".format(exc))

    # Attempt 2: reload the exact PCB file that was just saved, then export it.
    # This gives the exporter a clean board object and ensures the DSN matches
    # the saved .kicad_pcb rather than unsaved in-memory state.
    try:
        saved_board = pcbnew.LoadBoard(pcb_path)
        if saved_board is None:
            attempts.append("pcbnew.LoadBoard returned None for {}".format(pcb_path))
        else:
            result = exporter(saved_board, dsn_path)
            if result and os.path.isfile(dsn_path) and os.path.getsize(dsn_path) > 0:
                enforce_dsn_minimum_track_width(dsn_path)
                print("DSN exported from the saved PCB:", dsn_path)
                write_freerouting_all_nets_instructions()
                print("FreeRouting all-nets command:", os.path.abspath(FREEROUTING_COMMAND_FILE))
                return
            attempts.append(
                "saved-board export returned {!r}{}".format(
                    result,
                    " and produced no non-empty DSN" if not (
                        os.path.isfile(dsn_path) and os.path.getsize(dsn_path) > 0
                    ) else ""
                )
            )
    except Exception as exc:
        attempts.append("saved-board export raised: {}".format(exc))

    raise RuntimeError(
        "KiCad's native Specctra DSN exporter failed on both the in-memory "
        "board and the saved PCB. The PCB remains saved at: {}. "
        "Exporter diagnostics: {}. The failure is inside KiCad's DSN export "
        "routine (which suppresses its underlying C++ exception); it is not "
        "a FreeRouting routing failure.".format(pcb_path, " | ".join(attempts))
    )


# ================================================================
# BUILD
# ================================================================

def create_esp8266_printed_antenna(board, nets, esp_item, report):
    """Add a prototype right-edge PCB radiator from the RF pi-network output."""
    l2 = next(fp for fp in board.GetFootprints() if fp.GetReference() == "L2")
    p = find_footprint_pad(l2, "2").GetPosition()
    sx, sy = to_mm(p.x), to_mm(p.y)
    route = [
        (sx, sy), (42.0, sy), (44.5, sy),
        (44.5, sy + 6.0), (41.5, sy + 6.0),
        (41.5, sy - 6.0), (44.5, sy - 6.0),
        (44.5, sy - 12.0), (41.5, sy - 12.0),
    ]
    if min(y for _, y in route) < 2.0 or max(y for _, y in route) > PCB_HEIGHT - 2.0:
        raise RuntimeError("RF antenna meander does not fit in the right-edge corridor")
    add_polyline(board, nets["RF_ANT"], route, 0.80, pcbnew.B_Cu)
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.F_Cu)
    if hasattr(zone, "SetIsRuleArea"):
        zone.SetIsRuleArea(True)
    for name in ("SetDoNotAllowTracks", "SetDoNotAllowVias", "SetDoNotAllowFootprints", "SetDoNotAllowZoneFills"):
        fn = getattr(zone, name, None)
        if callable(fn):
            fn(True)
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in ((40.0, 39.0), (PCB_WIDTH, 39.0), (PCB_WIDTH, 59.0), (40.0, 59.0)):
        outline.Append(int(mm(x)), int(mm(y)))
    board.Add(zone)
    report.append("B.Cu right-edge folded PCB antenna starts at L2 pad 2; F.Cu 40-46 x 39-59 mm keepout")
    report.append("RF feed: U1 LNA -> L2 series -> printed antenna; C21/C22 are shunt DNP pads")
    report.append("Prototype radiator and pi values require VNA tuning on final PCB/enclosure")

def build_board():
    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )

    print("\nSMARTLOCK PCB GENERATOR")
    print("=======================")

    print("Board:", PCB_WIDTH, "x", PCB_HEIGHT)
    print("Copper layers: 2")
    print("Assembly: all components below central LED except solenoid connector above it")
    print("Thickness:", PCB_THICKNESS)
    front_count = len(COMPONENTS)
    print("Component placement: automatic two-sided collision-aware autoplace; physical antenna reservation and full-width J2 row reserved")
    print("Placement: 8 x 10 occupancy-balanced autoplace; no component shares J2's horizontal row")
    print("Components defined:", front_count)
    print("Separate M2 board mounting holes: none; connector mechanical holes reserved on both sides")
    print("RF: B.Cu printed right-edge meander antenna with pi matching footprints")
    print("Central LED: fixed at board centre")
    print("ESP8266EX bare QFN32: B.Cu lower-right; external flash/crystal and edge PCB antenna")
    print("General component clearance: {:.2f} mm per footprint envelope".format(COMPONENT_CLEARANCE))
    print("Placement strategy: balanced occupancy grid; U1 support group and reserved antenna corridor")
    print("Two-sided component placement: all legal board areas used outside the solenoid row")
    print("USB-C J1: fixed left edge, mirrored, receptacle opening facing left")
    print("Solenoid J2: fixed above central LED D3; its full-width horizontal row is reserved")
    print("R19: 470 ohms")
    print("USB-C shell tabs S1-S4 / SH1-SH4 -> GND")
    print("Components: support group beside U1; remaining parts packed around fixed anchors")
    print("Copper pours are disabled; all remaining nets require tracks/vias")

    prepared, led = prepare_all_components()

    board = pcbnew.BOARD()

    board.SetCopperLayerCount(2)

    board.GetDesignSettings().SetBoardThickness(
        mm(PCB_THICKNESS)
    )

    nets = create_nets(board)

    create_outline(board)
    create_board_edge_keepouts(board)

    report = []
    audit_errors = []

    # Place the central LED first.
    led_clearance = place_central_led(
        board,
        nets,
        led,
        report,
        audit_errors
    )

    # J1 is a fixed bottom-side edge connector on the left edge, above the antenna,
    # mirrored with its receptacle opening facing left. Reserve its physical envelope on BOTH
    # copper sides.
    usb_item = next(
        item for item in prepared
        if item["reference"] == "J1"
    )
    usb_clearance = place_fixed_usb(
        board,
        nets,
        usb_item,
        report,
        audit_errors
    )

    # J2 is fixed directly above and X-centred on the central LED rather than
    # left to the automatic packer, preserving the requested relative placement.
    solenoid_item = next(
        item for item in prepared
        if item["reference"] == "J2"
    )
    solenoid_clearance = place_fixed_solenoid(
        board,
        nets,
        solenoid_item,
        led,
        report,
        audit_errors
    )

    # Reserve a full-width horizontal lane at J2's row. The band follows the
    # measured, transformed J2 courtyard and adds 0.50 mm above/below it.
    # This prevents any other component from appearing on the solenoid row,
    # rather than merely preventing a local collision with the connector.
    j2_bounds = footprint_placement_bounds(solenoid_item["footprint"])
    solenoid_row_band = (
        mm(BOARD_MARGIN),
        max(mm(BOARD_MARGIN), j2_bounds[1] - mm(SOLENOID_ROW_EXTRA_CLEARANCE)),
        mm(PCB_WIDTH - BOARD_MARGIN),
        min(mm(PCB_HEIGHT - BOARD_MARGIN),
            j2_bounds[3] + mm(SOLENOID_ROW_EXTRA_CLEARANCE))
    )
    print(
        "Reserved full-width solenoid row:",
        round(to_mm(solenoid_row_band[1]), 2), "to",
        round(to_mm(solenoid_row_band[3]), 2), "mm Y"
    )

    # Place U1 below the central LED, after fixed connectors so their XY envelopes are reserved.
    esp_item = next(
        item for item in prepared
        if item["reference"] == "U1"
    )
    esp_clearance = place_fixed_esp(
        board, nets, esp_item, report, audit_errors,
        fixed_obstacles=[led, usb_item, solenoid_item]
    )
    esp_support_items = place_esp_support_components(
        board, nets, prepared, esp_item, report, audit_errors
    )
    create_esp8266_printed_antenna(board, nets, esp_item, report)

    for fixed_item in (usb_item, esp_item, led):
        fixed_bounds = footprint_placement_bounds(fixed_item["footprint"])
        if rectangle_overlap(fixed_bounds, solenoid_row_band):
            raise RuntimeError(
                "Fixed component {} falls in the reserved solenoid row. "
                "Revise its fixed anchor before packing.".format(
                    fixed_item["reference"]
                )
            )

    # Preserve the two-sided component assignments and distribute all
    # non-anchor components through every legal region of the board, excluding
    # the fixed anchors and full-width J2 row.
    # Pack F.Cu first, then reserve every front-side component envelope while
    # packing B.Cu to avoid opposite-side body/pad overlap in XY.
    support_clearances = [
        expand_rectangle(footprint_placement_bounds(item["footprint"]), COMPONENT_CLEARANCE / 2.0)
        for item in esp_support_items
    ]
    fixed_anchor_rectangles = [
        led_clearance, usb_clearance, solenoid_clearance, esp_clearance,
        solenoid_row_band
    ] + support_clearances
    fixed_anchor_items = [led, usb_item, solenoid_item, esp_item] + esp_support_items

    front_items = [
        item for item in prepared
        if item["reference"] not in ({"U1", "J1", "J2"} | ESP_SUPPORT_REFS)
        and item["side"] == "F"
    ]
    bottom_items = [
        item for item in prepared
        if item["reference"] not in ({"U1", "J1", "J2"} | ESP_SUPPORT_REFS)
        and item["side"] == "B"
    ]

    print("\nTwo-sided placement plan:")
    print("  F.Cu passives/components:", ", ".join(
        item["reference"] for item in front_items
    ))
    print("  B.Cu passives/components:", ", ".join(
        item["reference"] for item in bottom_items
    ))

    front = pack_side(
        board, nets, front_items, "F", report, audit_errors,
        fixed_anchor_rectangles,
        fixed_anchor_items
    )

    # Cross-side exclusion: the final audit requires opposite-side footprints
    # to clear one another physically, so carry the expanded F.Cu envelopes
    # into the B.Cu packer's occupied rectangles.
    front_side_reservations = [
        expand_rectangle(
            footprint_placement_bounds(item["footprint"]),
            COMPONENT_CLEARANCE / 2.0
        )
        for item in front
    ]
    front_side_fixed_items = fixed_anchor_items + front

    back = pack_side(
        board, nets, bottom_items, "B", report, audit_errors,
        fixed_anchor_rectangles + front_side_reservations,
        front_side_fixed_items
    )

    # Verify the one intentional front-side component remains on F.Cu.
    if solenoid_item["footprint"].GetLayer() != pcbnew.F_Cu:
        raise RuntimeError("Solenoid connector J2 must remain on F.Cu")

    placed = [esp_item, usb_item, solenoid_item] + esp_support_items + front + back

    # Final placement audit before saving.
    verify_placement(
        placed,
        led,
        solenoid_row_band
    )

    # Bare-chip RF path and host PCB antenna have been added above.

    # Apply pad geometry and local-clearance changes before zone fill so both
    # copper planes are calculated from the final footprints.
    # Apply physical pad geometry and local-clearance changes before zone fill,
    # so the pours are calculated from the final USB-C and regulator pad shapes.
    apply_drc_clearance_overrides(board)

    # Deliberately do not create copper zones, pours, or stitching vias.
    # Every connection, including GND and 3V3, is left for track routing.

    # Match every available FreeRouting netclass to the 0.20 mm PCB rule.
    apply_routing_constraints(board)

    create_labels(board)

    write_bom()

    write_report(
        report,
        audit_errors
    )

    if audit_errors:
        print("\nUNASSIGNED COPPER PADS:")

        for error in audit_errors:
            print("  ", error)

        raise RuntimeError(
            "Pad assignment audit failed. "
            "See pad_assignment_report.txt. "
            "PCB was not saved."
        )

    print("\nSaving KiCad PCB...")

    pcbnew.SaveBoard(
        OUTPUT_FILE,
        board
    )

    # DSN export is attempted after the PCB is saved.
    # If the exporter is unavailable, the PCB remains saved.
    export_dsn(board)

    print("\n======================================")
    print("SMARTLOCK PCB CREATED")
    print("======================================")

    print("PCB:", os.path.abspath(OUTPUT_FILE))
    print("DSN:", os.path.abspath(DSN_FILE))
    print("FreeRouting all-nets command:", os.path.abspath(FREEROUTING_COMMAND_FILE))
    print("FreeRouting all-nets launcher:", os.path.abspath(FREEROUTING_LAUNCHER_FILE))
    print("BOM:", os.path.abspath(BOM_FILE))
    print("Report:", os.path.abspath(REPORT_FILE))

    print("\nComponents:", len(placed)+1)
    print("Front:", len(front) + 1, "(front-side packed components plus solenoid J2)")
    print("Back:", len(back) + 3, "(back-side packed components, including D3, J1 and U1)")

    print("\nBATTERY PROTECTION:")
    print("1. DW01A OD drives FS8205A G1 (pad 4).")
    print("2. DW01A OC drives FS8205A G2 (pad 5).")
    print("3. FS8205A common drains remain on BAT_FET_COMMON.")
    print("4. Verify exact purchased FS8205A package and pinout.")

    print("\nUSB-C:")
    print("1. CC1 and CC2 use separate 5.1k pull-down resistors.")
    print("2. Four connector shell anchors assigned to GND.")
    print("3. Two non-plated alignment holes are mechanical only.")
    print("4. USB data pads remain intentionally unconnected.")

    print("\nESP8266EX BARE CHIP:")
    print("1. U1 is a bare ESP8266EX QFN32, not an ESP8266EX bare-chip design module.")
    print("2. External W25Q32 4MB SPI flash and 26MHz crystal are fitted.")
    print("3. RES12K pin 31 is connected through R25=12k to GND.")
    print("4. Printed B.Cu antenna is fed from L2; C21/C22 are DNP shunt positions.")
    print("5. Verify crystal load capacitors against the selected crystal CL.")
    print("6. RF geometry and matching values require VNA tuning.")

    print("\nNEXT:")
    print("1. Open PCB in KiCad.")
    print("2. Confirm all components except J2 and D3 are below the LED.")
    print("3. Confirm the full-width horizontal row around J2 contains no other component.")
    print("3. Export DSN using a compatible Specctra exporter.")
    print("4. Run RUN_FREEROUTING_ALL_NETS.bat; import the SES result and verify every airwire, including RF.")
    print("5. Import the SES session file into KiCad.")
    print("6. Run KiCad DRC after importing the SES session.")
    print("7. No copper pours are present; route all nets with tracks/vias, then run DRC.")

    print("\nIMPORTANT ENGINEERING CHECKS:")
    print("1. Verify ESP8266EX QFN32 footprint pad numbering and exposed-pad grounding.")
    print("2. Verify exact USB-C footprint pad numbering.")
    print("3. Verify switch footprint pad numbering.")
    print("4. Verify WS2812B-V6 footprint pad numbering.")
    print("5. Verify FS8205A package and protection topology.")
    print("6. Check charger current against actual battery.")
    print("7. Check solenoid current, MOSFET heating and diode rating.")
    print("8. Confirm capacitor voltage ratings.")
    print("9. Confirm regulator feedback voltages.")
    print("10. Check antenna tuning with final enclosure fitted.")
    print("11. Run electrical and physical DRC after routing.")

    print("\nDSN export is not proof of electrical correctness.")
    print("The board has not been electrically verified.")
    print("The antenna has not been RF tuned.\n")


if __name__ == "__main__":
    build_board()