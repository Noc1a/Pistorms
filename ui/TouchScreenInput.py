#!/usr/bin/env python3
#
# Copyright (c) 2016 mindsensors.com
# Redesigned layout for 320x240 landscape framebuffer.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.

import os, sys, inspect, time

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir  = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

# ── Layout constants for 320x240 landscape ───────────────────────────────────
_LM    = 1    # left margin (px)
_W     = 53   # letter column width  →  1 + 53*6 = 319 ≈ 320

_HDR_Y = 2    # Cancel / Submit row  top
_HDR_H = 30   # Cancel / Submit row  height
_HDR_W = 160  # Each header button half-width (2 × 160 = 320)

_TBX_Y = 34   # Textbox top
_TBX_H = 25   # Textbox height

_CTL_Y = 61   # Control row top (Shft/Abc/?-!/123/Clr/Bsp/</>)
_CTL_H = 32   # Control row height
_CTL_W = 40   # Control button width (8 × 40 = 320)

_LTR_Y = 95   # First letter row top  (61+32+2 = 95)
_LTR_H = 34   # Letter button height
_LTR_G = 2    # Gap between letter rows
_LTR_COLS = 6
_LTR_ROWS = 4
_N_KEYS   = _LTR_COLS * _LTR_ROWS  # 24 letters visible at once

# Total: 2+30+2+25+2+32+2+(34+2)*4-2+3 = 240 ✓


class TouchScreenInput:
    led_on_func  = None
    led_off_func = None

    def __init__(self, screen, left_padding=True):
        self.scrn = screen
        self.lm = _LM
        self.w  = _W

    def bind_led_off_func(self, func_name):
        self.led_off_func = func_name

    def bind_led_on_func(self, func_name):
        self.led_on_func = func_name

    def update_textbox(self, txt, hide):
        if hide:
            txt = "*" * len(txt)
        sz  = 20;  top = _TBX_Y + 4
        if len(txt) > 13: sz = 17; top = _TBX_Y + 5
        if len(txt) > 19: sz = 15; top = _TBX_Y + 6
        if len(txt) > 24: sz = 13; top = _TBX_Y + 7
        self.scrn.fillRect(2, _TBX_Y + 2, 316, _TBX_H - 4, fill=(220, 220, 220), display=False)
        if len(txt) > 0:
            self.scrn.drawAutoText(txt, _LM + 8, top, fill=(0, 0, 0), size=sz, display=False)
        self.scrn.refresh()

    def redraw(self, layout, start):
        symbols = "., !?@#$%^&*()_-+=[]{}<>\\/|~`'\""
        numbers = "0123456789"
        letters = "abcdefghijklmnopqrstuvwxyz"
        used = ""
        if   layout == "abc": used = letters
        elif layout == "ABC": used = letters.upper()
        elif layout == "?@!": used = symbols
        elif layout == "123": used = numbers

        if start < 0: start = 0

        keys = []
        for row in range(_LTR_ROWS):
            for col in range(_LTR_COLS):
                idx = start + row * _LTR_COLS + col
                x = _LM + col * _W
                y = _LTR_Y + row * (_LTR_H + _LTR_G)
                if idx < len(used):
                    ch = used[idx]
                    self.scrn.drawButton(
                        x, y, width=_W, height=_LTR_H,
                        text=ch, display=False, align="xcenter", font_size=16
                    )
                else:
                    ch = ""
                    self.scrn.fillRect(x, y, _W, _LTR_H, fill=(30, 30, 30), display=False)
                keys.append(ch)
        self.scrn.refresh()
        return keys

    def getInput(self, hide=False):
        if self.led_off_func is not None:
            self.led_off_func()

        # ── Static UI ────────────────────────────────────────────────────────
        self.scrn.fillRect(0, 0, 320, 240, fill=(0, 0, 0), display=False)

        # Cancel / Submit
        self.scrn.drawButton(0,        _HDR_Y, _HDR_W, _HDR_H, text="  Cancel", display=False, align="left", font_size=16)
        self.scrn.drawButton(_HDR_W,   _HDR_Y, _HDR_W, _HDR_H, text="  Submit", display=False, align="left", font_size=16)

        # Textbox border + fill
        self.scrn.fillRect(0,  _TBX_Y,     320,     _TBX_H,     fill=(100, 100, 100), display=False)
        self.scrn.fillRect(2,  _TBX_Y + 2, 316,     _TBX_H - 4, fill=(220, 220, 220), display=False)

        # Control row: 8 buttons × 40px
        for i, label in enumerate(["Shft", "Abc", "?-!", "123", "Clr", "Bsp", "<", ">"]):
            self.scrn.drawButton(i * _CTL_W, _CTL_Y, _CTL_W, _CTL_H,
                                 text=label, display=False, align="xcenter", font_size=13)

        exit      = False
        usr_input = ""
        layout    = "abc"
        index     = 0
        upper     = False
        keys      = self.redraw(layout, index)

        toSubmit = {}
        while not exit:
            # ── Read control row ──────────────────────────────────────────────
            shft = self.scrn.checkButton(0 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            abc  = self.scrn.checkButton(1 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            sym  = self.scrn.checkButton(2 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            num  = self.scrn.checkButton(3 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            clr  = self.scrn.checkButton(4 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            bsp  = self.scrn.checkButton(5 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            prev = self.scrn.checkButton(6 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)
            nxt  = self.scrn.checkButton(7 * _CTL_W, _CTL_Y, _CTL_W, _CTL_H)

            # ── Read header ───────────────────────────────────────────────────
            cancel = self.scrn.checkButton(0,      _HDR_Y, _HDR_W, _HDR_H)
            submit = self.scrn.checkButton(_HDR_W, _HDR_Y, _HDR_W, _HDR_H)

            # ── Read 24 letter buttons (row-major) ────────────────────────────
            letter_hits = []
            for row in range(_LTR_ROWS):
                for col in range(_LTR_COLS):
                    letter_hits.append(
                        self.scrn.checkButton(
                            _LM + col * _W, _LTR_Y + row * (_LTR_H + _LTR_G),
                            _W, _LTR_H
                        )
                    )

            # ── Layout controls ───────────────────────────────────────────────
            if   shft: upper = not upper
            elif abc:  layout = "abc"; index = 0
            elif sym:  layout = "?@!"; index = 0
            elif num:  layout = "123"; index = 0
            elif prev: index = max(0, index - _N_KEYS)
            elif nxt:  index += _N_KEYS

            layout = layout.upper() if upper else layout.lower()

            if shft or abc or sym or num or prev or nxt:
                keys = self.redraw(layout, index)
                continue

            # ── Text input ────────────────────────────────────────────────────
            if bsp:
                if len(usr_input) == 0:
                    for _ in range(3):
                        if self.led_on_func  is not None and self.led_off_func is not None:
                            self.led_on_func()
                        time.sleep(0.1)
                        if self.led_off_func is not None and self.led_on_func  is not None:
                            self.led_off_func()
                        time.sleep(0.1)
                else:
                    usr_input = usr_input[:-1]

            if clr:
                usr_input = ""
            else:
                for i, hit in enumerate(letter_hits):
                    if hit and keys[i]:
                        usr_input += keys[i]
                        break

            if clr or bsp or any(h and keys[i] for i, h in enumerate(letter_hits)):
                self.update_textbox(usr_input, hide)
                continue

            # ── Exit ──────────────────────────────────────────────────────────
            if submit:
                exit = True
                toSubmit = {"submitted": True,  "response": usr_input}
            elif cancel:
                exit = True
                toSubmit = {"submitted": False, "response": usr_input}

            time.sleep(0.05)

        return toSubmit
