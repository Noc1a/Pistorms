#!/usr/bin/env python
# Revised mindsensorsUI for Framebuffer (FB1)
# Provides full compatibility with PiStorms original API

import os
import sys
import numpy as np
_sys_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sys'))
if _sys_dir not in sys.path: sys.path.insert(0, _sys_dir)
from PIL import Image, ImageDraw, ImageFont
from PiStormsCom import PiStormsCom

class mindsensorsUI():
    ## Color Constants
    PS_BLACK = (0,0,0)
    PS_BLUE = (0,0,255)
    PS_RED = (255,0,0)
    PS_GREEN = (0,255,0)
    PS_CYAN = (0,255,255)
    PS_MAGENTA = (255,0,255)
    PS_YELLOW = (255,255,0)
    PS_WHITE = (255,255,255)

    ## Default Screen Dimensions (Physical)
    PS_SCREENWIDTH = 240
    PS_SCREENHEIGHT = 320

    ## Mode Constants
    PS_MODE_TERMINAL = 0
    PS_MODE_POPUP = 1
    PS_MODE_DEAD = 2

    def __init__(self, name="PiStorms", rotation=3):
        """
        Initializes the UI device using the Framebuffer /dev/fb1.
        @param name: Display title (for compatibility).
        @param rotation: Screen rotation (0-3).
        """
        self.fb_path = "/dev/fb1"
        self.comm = PiStormsCom()
        self.currentRotation = rotation
        self.currentMode = self.PS_MODE_TERMINAL
        
        # Terminal management
        self.terminalBuffer = [""] * 20
        self.terminalCursor = 0
        
        # UI State
        self.drawArrowsbool = False
        self.buttonText = ["OK", "Cancel"]
        self.popupText = ["Do you wish to continue?"]
        self.x = 0  # Last touch X
        self.y = 0  # Last touch Y
        self.myname = name

        # Initialize the Pillow drawing canvas (Fixed landscape for FB1)
        # Most FB1 drivers for PiStorms are set to 320x240
        self.buffer = Image.new("RGB", (320, 240))
        self.draw = ImageDraw.Draw(self.buffer)
        
        self.clearScreen()

    def refresh(self):
        """
        Pushes the current Pillow buffer to the Framebuffer (/dev/fb1).
        Converts RGB888 to RGB565 for the LCD hardware.
        """
        img_np = np.array(self.buffer)
        # Fast NumPy conversion to 16-bit RGB565
        r = (img_np[:, :, 0] >> 3).astype(np.uint16)
        g = (img_np[:, :, 1] >> 2).astype(np.uint16)
        b = (img_np[:, :, 2] >> 3).astype(np.uint16)
        rgb565 = ((r << 11) | (g << 5) | b).astype(np.uint16)
        
        try:
            with open(self.fb_path, "wb") as f:
                f.write(rgb565.tobytes())
        except IOError:
            sys.stderr.write("Error: Cannot write to /dev/fb1. Check permissions.\n")

    # --- Mode & Display Management ---

    def setMode(self, mode=0):
        """ Sets the UI mode: Terminal, Popup, or Dead. """
        self.currentMode = mode if 0 <= mode <= 2 else self.PS_MODE_DEAD
        self.refresh()

    def getMode(self):
        """ Returns the current UI mode. """
        return self.currentMode

    def dumpTerminal(self):
        """ Clears the terminal buffer. """
        self.terminalBuffer = [""] * 20
        self.terminalCursor = 0
        if self.currentMode == self.PS_MODE_TERMINAL:
            self.clearScreen()

    def clearScreen(self, display=True):
        """ Clears the screen to black. """
        self.draw.rectangle((0, 0, 320, 240), fill=(0,0,0))
        if display: self.refresh()

    # --- Drawing Methods ---

    def fillRect(self, x, y, width, height, fill=(255,255,255), outline=None, display=True):
        """ Draws a filled rectangle. """
        self.draw.rectangle([x, y, x + width, y + height], fill=fill, outline=outline)
        if display: self.refresh()

    def fillCircle(self, x, y, radius, fill=(255,255,255), display=True):
        """ Draws a filled circle. """
        self.draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=fill)
        if display: self.refresh()

    def drawCircle(self, x, y, radius, fill=None, outline=(0,0,0), display=True):
        """ Draws an empty circle with an outline. """
        self.draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=fill, outline=outline)
        if display: self.refresh()

    def fillRoundRect(self, x, y, width, height, radius, fill=(255,255,255), display=True):
        """ Draws a rectangle with rounded corners. """
        self.draw.rectangle([x + radius, y, x + width - radius, y + height], fill=fill)
        self.draw.rectangle([x, y + radius, x + width, y + height - radius], fill=fill)
        self.draw.ellipse([x, y, x + radius * 2, y + radius * 2], fill=fill)
        self.draw.ellipse([x + width - radius * 2, y, x + width, y + radius * 2], fill=fill)
        self.draw.ellipse([x, y + height - radius * 2, x + radius * 2, y + height], fill=fill)
        self.draw.ellipse([x + width - radius * 2, y + height - radius * 2, x + width, y + height], fill=fill)
        if display: self.refresh()

    def drawAutoText(self, text, x, y, fill=(255,255,255), size=20, align="left", display=True):
        """ Draws text with specified size and color. """
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", size)
        except:
            font = ImageFont.load_default()
        self.draw.text((x, y), str(text), fill=fill, font=font)
        if display: self.refresh()

    # --- Terminal Methods ---

    def termGotoLine(self, lineno):
        """ Moves the terminal cursor to a specific line. """
        self.terminalCursor = lineno

    def termPrintAt(self, lineno, text):
        """ Prints text at a specific line, clearing the line first. """
        self.terminalCursor = lineno
        self.terminalBuffer[self.terminalCursor] = str(text)
        self.refreshLine(lineno)

    def termPrint(self, text):
        """ Appends text to the current terminal line. """
        self.terminalBuffer[self.terminalCursor] += str(text)
        self.refreshLine(self.terminalCursor)

    def termPrintln(self, text):
        """ Prints text and moves to the next line. """
        if self.terminalCursor > 10:
            self.dumpTerminal()
        self.termPrint(text)
        self.terminalCursor += 1

    def termReplaceLastLine(self, text):
        """ Replaces the content of the current line. """
        self.terminalBuffer[self.terminalCursor] = ""
        self.termPrintAt(self.terminalCursor, text)

    def refreshLine(self, lineNum, display=True):
        """ Refreshes a single line on the display. """
        y_pos = lineNum * 20 + 2
        self.draw.rectangle([0, y_pos, 320, y_pos + 19], fill=(0, 0, 0))
        self.drawAutoText(self.terminalBuffer[lineNum], 10, y_pos, display=display)

    # --- Image & Button Methods ---

    def fillBmp(self, x, y, width, height, path, display=True):
        """ Displays a bitmap/image from a file path. """
        try:
            img = Image.open(path).resize((width, height))
            self.buffer.paste(img, (x, y))
            if display: self.refresh()
        except:
            pass

    def fillImgArray(self, x, y, width, height, image, display=True):
        """ Optimized for OpenCV arrays (AI Camera). """
        img_pil = Image.fromarray(image).resize((width, height))
        self.buffer.paste(img_pil, (x, y))
        if display: self.refresh()

    def drawButton(self, x, y, width, height, prefix="btns_", text="OK", display=True, align="left", image=None, font_size=16):
        """ Draws a stylized button. """
        self.fillRoundRect(x, y, int(width), int(height), 5, fill=(180,180,180), display=False)
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", font_size)
        except:
            font = ImageFont.load_default()
        text_w = font.getlength(str(text))
        if align == "xcenter":
            tx = x + (int(width) - int(text_w)) // 2
        else:
            tx = x + 10
        ty = y + (int(height) // 2) - (font_size // 2)
        self.draw.text((tx, ty), str(text), fill=(0, 0, 0), font=font)
        if display:
            self.refresh()

    def drawArrows(self, display=True):
        """ Draws navigation arrows on the screen. """
        self.drawButton(0, 0, 50, 40, text="<", display=False)
        self.drawButton(self.screenWidth() - 50, 0, 50, 40, text=">", display=display)

    def showArrows(self, refresh=True):
        self.drawArrowsbool = True
        self.drawArrows(display=refresh)

    def hideArrows(self, refresh=True):
        self.drawArrowsbool = False
        if refresh: self.refresh()

    # --- Touch & Coordinates ---

    def isTouched(self):
        """ Detects touch and updates self.x, self.y. """
        tx, ty = self.comm.getTouchscreenCoordinates()
        if tx == 0 and ty == 0:
            return False
        # tx (PS_TSX): inverted, high=left → low=right across 320px
        # ty (PS_TSY): direct top→bottom across 240px
        # Calibrated from two touch points: (tx=233→x=80), (tx=62→x=240)
        self.x = max(0, min(319, round(298 - tx * 160 / 171)))
        self.y = max(0, min(239, ty))
        return True

    def checkButton(self, x, y, width, height):
        """ Checks if a specific area is currently touched. """
        if self.isTouched():
            if x <= self.x <= x + width and y <= self.y <= y + height:
                return True
        return False

    def checkArrows(self):
        """ Checks if navigation arrows are pressed. """
        return (self.checkButton(0, 0, 50, 40), self.checkButton(self.screenWidth() - 50, 0, 50, 40))

    def screenWidth(self):
        return 320 if self.currentRotation in [1, 3] else 240

    def screenHeight(self):
        return 240 if self.currentRotation in [1, 3] else 320

    # --- Coordinate Utilities (Compatibility) ---
    def screenXFromImageCoords(self, x, y): return x
    def screenYFromImageCoords(self, x, y): return y
    def TS_To_ImageCoords_X(self, x, y): return y
    def TS_To_ImageCoords_Y(self, x, y): return x