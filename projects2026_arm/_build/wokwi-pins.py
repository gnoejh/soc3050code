#!/usr/bin/env python3
"""The pin names Wokwi accepts for the Nucleo-C031C6 - as a page students can read.

A `diagram.json` connection names a board pin by the string Wokwi's board file
gives it, and those strings are visible nowhere a student would look: `PB0`
does not exist, `PB0.1` and `PB0.2` and `D10` all do and all reach the same
MCU pin, and `GND` is really `GND.1` through `GND.9`.  Nothing in the editor
lists them.  This script turns the board file into one table, rendered beside
the decks and linked from the top bar of every slide, so the answer to "what
do I type" is one click away during a lab.

Two files, one derived from the other:

    _targets/c031c6-pins.json   the facts: every Wokwi name -> the MCU pin it
                                reaches.  Committed, so a clone needs no
                                network.  Derived from Wokwi's board file with
                                --refresh; not the board file itself, which
                                carries no licence and is not ours to vendor.
    _slides/board-pins.html     the page.  build-slides.py calls render() so a
                                normal deck render keeps it current.

Usage:
    python _build/wokwi-pins.py                     # JSON -> HTML
    python _build/wokwi-pins.py --refresh           # Wokwi's board file -> JSON -> HTML
    python _build/wokwi-pins.py --refresh --from board.json   # from a local copy

The notes column - which pins the course reserves, and the board-file quirks
worth a warning - is the NOTES table below and is maintained by hand.  It is
course knowledge, not something the board file can say.
"""
import datetime
import html
import json
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FACTS = os.path.join(BASE, "_targets", "c031c6-pins.json")
OUT_NAME = "board-pins.html"

SOURCE = ("https://raw.githubusercontent.com/wokwi/wokwi-boards/main/"
          "boards/st-nucleo-c031c6/board.json")
WOKWI_PART = "board-st-nucleo-c031c6"
BOARD_DOC = "https://docs.wokwi.com/parts/board-st-nucleo-c031c6"
# Relative: build-slides.py copies the PDF from _docs/ to sit beside this page.
BOARD_MANUAL = "UM2953_Nucleo64_MB1717.pdf"

# Course knowledge about MCU pins.  Keyed by the MCU pin (the board file's
# "target"), not by a Wokwi name, so an alias inherits its pin's note.
NOTES = {
    "PA2":  "USART2 TX, alternate function 1 - the serial monitor that printf writes to. Taken.",
    "PA3":  "USART2 RX - the serial monitor. Taken.",
    "PA5":  "LD4, the on-board green LED. Lesson 04 uses it as the heartbeat.",
    "PA13": "SWDIO - the ST-LINK debug line on real hardware. Leave it alone.",
    "PA14": "SWCLK - the ST-LINK debug line on real hardware. Leave it alone.",
    "PC14": "LSE crystal pad (32 kHz) on the real board.",
    "PC15": "LSE crystal pad (32 kHz) on the real board.",
    "PF0":  "HSE crystal pad on the real board (not fitted).",
    "PF1":  "HSE crystal pad on the real board (not fitted).",
    "PB0":  "LED bar, bit 0, in lesson 04. Note there is no plain PB0 - see the trap list.",
    "PB1":  "LED bar, bit 1, in lesson 04.",
    "PB2":  "LED bar, bit 2, in lesson 04.",
    "PB3":  "LED bar, bit 3, in lesson 04.",
    "PB4":  "LED bar, bit 4, in lesson 04.",
    "PB5":  "LED bar, bit 5, in lesson 04.",
    "PB6":  "LED bar, bit 6, in lesson 04.",
    "PB7":  "LED bar, bit 7, in lesson 04.",
    "PD2":  "Two names, PD2 and PD2.2, and no PD2.1 - a quirk of the board file, harmless.",
}

# Names in the board file that do not reach an MCU pin at all.
POWER = {
    "GND":        "Ground. Nine names, all the same net; GND.1 is the one the on-board LEDs use.",
    "power(3.3)": "3.3 V rail. VDD, 3V3.1 and 3V3.2 - and, oddly, PD3: the board file maps that name to the rail, not to the MCU pin, so do not use it expecting PD3.",
    "power(5)":   "5 V rail: 5V.1-5V.3, VIN.1/VIN.2, E5V, AVDD.1/AVDD.2 are all this net in the simulator.",
    "IOREF":      "I/O reference voltage, 3.3 V on this board.",
    "NRST":       "Reset. Pull it low to reset the MCU.",
}


# --------------------------------------------------------------------------
# --refresh: Wokwi's board file -> the facts file
# --------------------------------------------------------------------------
def load_board_file(path_or_none):
    """Wokwi's board.json carries // comments and the odd trailing comma, so
    it is JSON5-ish rather than JSON.  Strip both before parsing."""
    if path_or_none:
        raw = open(path_or_none, encoding="utf-8").read()
    else:
        import urllib.request
        with urllib.request.urlopen(SOURCE, timeout=30) as r:
            raw = r.read().decode("utf-8")
    txt = re.sub(r"//[^\n]*", "", raw)
    txt = re.sub(r",(\s*[}\]])", r"\1", txt)
    return json.loads(txt)


def refresh(local):
    board = load_board_file(local)
    pins = {name: spec["target"] for name, spec in board["pins"].items()}
    leds = {led["id"]: led["pins"] for led in board.get("leds", [])}
    facts = {
        "board": board["name"],
        "mcu": board["mcu"],
        "wokwi_part": WOKWI_PART,
        "source": SOURCE,
        "derived": datetime.date.today().isoformat(),
        "note": ("Derived from Wokwi's board definition: every name the "
                 "diagram.json editor accepts for this board, and the MCU pin "
                 "or rail it reaches. Regenerate with "
                 "python _build/wokwi-pins.py --refresh"),
        "pins": pins,
        "leds": leds,
    }
    os.makedirs(os.path.dirname(FACTS), exist_ok=True)
    with open(FACTS, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(facts, fh, indent=2)
        fh.write("\n")
    print("wrote %s: %d names -> %d targets"
          % (os.path.relpath(FACTS, BASE), len(pins), len(set(pins.values()))))


# --------------------------------------------------------------------------
# the page
# --------------------------------------------------------------------------
def pin_key(target):
    m = re.match(r"^P([A-Z])(\d+)$", target)
    return (0, m.group(1), int(m.group(2))) if m else (1, target, 0)


def group_by_target(pins):
    by = {}
    for name, target in pins.items():
        by.setdefault(target, []).append(name)
    return by


def header_label(names):
    return ", ".join(n for n in names if re.match(r"^[DA]\d+$", n))


def render(out_dir):
    facts = json.load(open(FACTS, encoding="utf-8"))
    pins = facts["pins"]
    by = group_by_target(pins)
    mcu_targets = sorted((t for t in by if re.match(r"^P[A-Z]\d+$", t)), key=pin_key)
    other_targets = [t for t in by if t not in mcu_targets]

    e = html.escape

    def code_list(names):
        return " ".join("<code>%s</code>" % e(n) for n in names)

    rows = []
    for t in mcu_targets:
        names = by[t]
        rows.append("<tr><td><b>%s</b></td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
            e(t), code_list(names), e(header_label(names)), e(NOTES.get(t, ""))))
    mcu_table = "\n".join(rows)

    rows = []
    for t in other_targets:
        rows.append("<tr><td><b>%s</b></td><td>%s</td><td>%s</td></tr>" % (
            e(t), code_list(by[t]), e(POWER.get(t, ""))))
    power_table = "\n".join(rows)

    rows = []
    for led_id, p in facts.get("leds", {}).items():
        rows.append("<tr><td><code>%s</code></td><td><code>%s</code></td><td><code>%s</code></td></tr>"
                    % (e(led_id), e(p.get("A", "")), e(p.get("C", ""))))
    led_table = "\n".join(rows)

    page = PAGE.format(
        board=e(facts["board"]), mcu=e(facts["mcu"]), part=e(facts["wokwi_part"]),
        source=e(facts["source"]), derived=e(facts["derived"]), doc=BOARD_DOC,
        manual=BOARD_MANUAL,
        n_names=len(pins), n_targets=len(by),
        mcu_table=mcu_table, power_table=power_table, led_table=led_table)

    os.makedirs(out_dir, exist_ok=True)
    dest = os.path.join(out_dir, OUT_NAME)
    with open(dest, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(page)
    return dest


PAGE = """<!DOCTYPE html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Nucleo-C031C6 pin names</title>
<style>
  :root {{ --bg:#fff; --fg:#1a1c1f; --muted:#5b6270; --rule:#dfe3ea; --accent:#1f5fa8; --card:#f7f9fc; --code-bg:#f1f3f7; --warn:#a8531f; }}
  @media (prefers-color-scheme: dark) {{
    :root:not([data-theme="light"]) {{ --bg:#14171c; --fg:#e6e9ee; --muted:#9aa3b2; --rule:#2b313b; --accent:#78b4f0; --card:#1b1f26; --code-bg:#1e232b; --warn:#f0a070; }}
  }}
  :root[data-theme="dark"] {{ --bg:#14171c; --fg:#e6e9ee; --muted:#9aa3b2; --rule:#2b313b; --accent:#78b4f0; --card:#1b1f26; --code-bg:#1e232b; --warn:#f0a070; }}
  body {{ background:var(--bg); color:var(--fg); margin:0; padding:2.5rem 1rem 5rem;
         font:16px/1.55 -apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif; }}
  .wrap {{ max-width: 980px; margin: 0 auto; }}
  h1 {{ font-size:1.9rem; margin:0 0 .3rem; letter-spacing:-.02em; }}
  h2 {{ font-size:1.25rem; margin:2.2rem 0 .6rem; }}
  .sub {{ color:var(--muted); margin:0 0 1rem; }}
  a {{ color:var(--accent); }}
  code {{ font-family: ui-monospace,"Cascadia Mono",Consolas,monospace; font-size:.9em;
          background:var(--code-bg); padding:.08em .35em; border-radius:4px; white-space:nowrap; }}
  pre {{ background:var(--code-bg); padding:.8rem 1rem; border-radius:8px; overflow-x:auto; font-size:.88rem; line-height:1.45; }}
  pre code {{ background:none; padding:0; white-space:pre; }}
  table {{ border-collapse:collapse; margin:.6rem 0 1rem; width:100%; font-size:.93rem; }}
  th, td {{ border:1px solid var(--rule); padding:.4rem .6rem; text-align:left; vertical-align:top; }}
  th {{ background:var(--card); }}
  td:first-child {{ white-space:nowrap; }}
  .trap {{ background:var(--card); border-left:4px solid var(--warn); padding:.7rem 1rem; margin:.7rem 0; border-radius:0 8px 8px 0; }}
  .trap b {{ color:var(--warn); }}
  .foot {{ color:var(--muted); font-size:.85rem; margin-top:2.5rem; border-top:1px solid var(--rule); padding-top:1rem; }}
  @media (max-width: 640px) {{ table {{ display:block; overflow-x:auto; }} }}
</style>
<div class="wrap">
  <p class="sub"><a href="index.html">&larr; all decks</a></p>
  <h1>Nucleo-C031C6: the pin names <code>diagram.json</code> accepts</h1>
  <p class="sub">{board} &middot; MCU <code>{mcu}</code> &middot; Wokwi part <code>{part}</code> &middot;
     {n_names} names reaching {n_targets} pins and rails.</p>

  <p>Every wire in a Wokwi circuit is one line in <code>diagram.json</code>, and the board end of it
  names a pin <b>by the exact string in Wokwi's board file</b>. Those strings are not printed on the
  board image and the editor does not list them. This page does. It is generated from that board file,
  so it is what Wokwi will accept, not what a datasheet says the pin is called.</p>
  <p>For what a header pin physically <em>is</em> - which connector, which solder bridge, what the
  schematic shows - the authority is the board's own manual,
  <a href="{manual}" target="_blank" rel="noopener">ST UM2953, STM32 Nucleo-64 boards (MB1717)</a>.
  Its Arduino and morpho connector tables are where the <code>D10</code> and <code>A3</code> labels
  below come from. Use both: the manual for the hardware, this page for the spelling.</p>

  <h2>How a connection is written</h2>
  <p>From lesson 04, the wire that puts LED bit 0 on PB0:</p>
<pre><code>[ "led0:A",  "nucleo:PB0.1",  "green",  [] ]
   |          |               |        |
   |          |               |        +-- routing hints; leave empty and Wokwi draws the wire
   |          |               +-- wire colour, cosmetic
   |          +-- board id in "parts", a colon, then a name from the tables below
   +-- the part's id, a colon, then that part's own pin name (an LED has A and C)</code></pre>
  <p>The serial monitor is a pseudo-part: <code>"$serialMonitor:RX"</code> and
  <code>"$serialMonitor:TX"</code>, wired to <code>nucleo:PA2</code> and <code>nucleo:PA3</code>.</p>

  <h2>Three traps</h2>
  <div class="trap"><b>There is no plain <code>PB0</code>.</b> When a signal reaches two header
  positions the board file gives each a suffix: <code>PB0.1</code>, <code>PB0.2</code>,
  <code>PB12.1</code>, <code>GND.1</code> &hellip; <code>GND.9</code>. Either suffixed name works; the
  bare one is rejected. PB1 through PB7 appear once, so they have no suffix. That is why lesson 04's
  <code>led0</code> line is the odd one out.</div>
  <div class="trap"><b>The Arduino labels are aliases.</b> <code>D10</code> and <code>PB0.1</code> and
  <code>PB0.2</code> are the same MCU pin. <code>D13</code> is <code>PA5</code>, the LED. Use the
  <code>Pxn</code> form in this course, because that is the name the reference manual and your
  <code>GPIOx-&gt;</code> code use.</div>
  <div class="trap"><b>Some pins are already spoken for.</b> PA2/PA3 carry <code>printf</code>,
  PA13/PA14 are the debugger, PC14/PC15 and PF0/PF1 are crystal pads on real hardware. The notes column
  says so.</div>

  <h2>MCU pins</h2>
  <table>
    <tr><th>MCU pin</th><th>Names Wokwi accepts</th><th>Arduino header</th><th>Notes for this course</th></tr>
{mcu_table}
  </table>

  <h2>Power, ground and reset</h2>
  <table>
    <tr><th>Net</th><th>Names Wokwi accepts</th><th>What it is</th></tr>
{power_table}
  </table>

  <h2>On-board LEDs</h2>
  <p>These are part of the board and need no wire. LD4 lights when its pin is driven high.</p>
  <table>
    <tr><th>Wokwi id</th><th>Anode</th><th>Cathode</th></tr>
{led_table}
  </table>

  <p class="foot">Derived {derived} from Wokwi's board definition
  (<a href="{source}" target="_blank" rel="noopener">board.json</a>) by
  <code>_build/wokwi-pins.py</code>; the facts live in <code>_targets/c031c6-pins.json</code>.
  Wokwi's own page for the board, including the peripherals it does <em>not</em> simulate:
  <a href="{doc}" target="_blank" rel="noopener">{doc}</a>.
  If Wokwi renames a pin, re-run the script with <code>--refresh</code> rather than editing this page.</p>
</div>
</html>
"""


def main(argv):
    local = None
    if "--from" in argv:
        local = argv[argv.index("--from") + 1]
    if "--refresh" in argv:
        refresh(local)
    if not os.path.isfile(FACTS):
        print("no %s - run with --refresh first" % os.path.relpath(FACTS, BASE))
        return 1
    dest = render(os.path.join(BASE, "_slides"))
    print("wrote %s" % os.path.relpath(dest, BASE))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
