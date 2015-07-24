# ##############################################################################
#
# PALLANTIR - Programa de manipulacion para la esfera interactiva Pallantir
#   Pallantir es un proyecto de LINTI - UNLP para realizar una esfera digital
#    interactiva.
#
# Copyright (C) 2015 Joaquin Bogado <jbogado@linti.unlp.edu.ar>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#################################################################################

import SimpleCV as scv
import numpy as np
import pyautogui as gui
from time import sleep
from math import *

gui.FAILSAFE = True

d = scv.Display((480,640))

def calibrate():
    k = scv.Kinect()
    center = None
    radii = None
    while d.isNotDone():
        im = scv.Image(k.getDepthMatrix())
        if d.mouseLeft:
            center = (d.mouseX, d.mouseY)
        if center:
            im.drawCircle(center, 1, color=scv.Color.RED)
            im.drawLine(center,(d.mouseX, d.mouseY), color=scv.Color.RED)
            x, y = d.mouseX, d.mouseY
            radii = sqrt((x - center[0])**2 + (y - center[1])**2)
            im.drawCircle(center, radii, color=scv.Color.RED)
            if d.mouseRight:
                im.save(d)
                sleep(1)
                d.quit()
                return center,int(ceil(radii))
        im.save(d)

def detect_movement(center,radii):
    '''One-Gaussian motion detection algorithm'''
    display = scv.Display((radii*2,radii*2))

    deltax = gui.size()[0]/(radii*2.0)
    deltay = gui.size()[1]/(radii*2.0)
    print center, radii, deltax, deltay
    # initialization
    k = scv.Kinect()

    bg = [k.getDepthMatrix().astype('float32') for x in range(10) ]
    mu0 = bg[0]
    ssq0 = bg[0]
    mutprev = mu0
    ssqtprev = mu0
    p = 0.01

    # background calculation
    for i in range(1,10):
        mut = bg[i] * p + (1 - p) * mutprev
        ssqt = np.abs((bg[i] - mut)*(bg[i] - mut))*p + (1 - p)*ssqtprev

    # motion detection loop
    while display.isNotDone():
        It = k.getDepthMatrix().astype('float32')
        mut = It * p + (1 - p) * mutprev
        ssqt = np.abs((It - mut)*(It - mut))*p + (1 - p)*ssqtprev
        back = np.abs(It - mut)/ssqt > 0.075
        fore = np.abs(It - mut)/ssqt <= 0.035
        im = scv.Image(fore).binarize().blur().blur().blur().blur()
        im = im.crop(x=center[0], y=center[1], w=radii*2, h=radii*2, centered=True)
        im = im.flipVertical().rotate(90)
        #im = scv.Image(mask).binarize().invert()
        # moving if minbsize < blob size < maxbsize
        minbsize = 4000
        maxbsize = 150000
        # movementes are captured if and only if are contained in this circle
        circle = scv.Circle(im, im.size()[0]/2, im.size()[1]/2, radii)
        circle.draw()
        blobs = im.findBlobs(minsize = minbsize)
        if blobs:
            blobs = filter(lambda b: circle.overlaps(b), blobs)
        if blobs:
            handblob = blobs[0]
            x0,y0 = handblob.topRightCorner()
            x0 = handblob.topLeftCorner()[0] + ((x0 - handblob.topLeftCorner()[0])/2)
            #y0 -= j.topLeftCorner()[1]
            #x0,y0 = j.coordinates()
            if handblob.area() > maxbsize:
                print 'not moving'
            else:
                plist.append((x0,y0))
                im.drawRectangle(handblob.topLeftCorner()[0],handblob.topLeftCorner()[1],handblob.width(), 100, color=scv.Color.RED)
                c = im.crop(x=handblob.topLeftCorner()[0],y=handblob.topLeftCorner()[1],w=handblob.width(), h=100)
                c.findBlobs()[0].draw()
                handblob.draw()
                # media for at least two points
                if len(plist) >= 2:
                    x0,y0 = plist[-2]
                    x1,y1 = plist[-1]
                    vector = ((x1-x0)*deltax,(y1-y0)*deltay)
                    magnitude = sqrt(vector[0]**2 + vector[1]**2)
                    # Draw the vector and magnitude
                    #im.drawLine((x0, y0), (x1, y1), color=scv.Color.RED);
                    #im.blit(c, pos=(handblob.topLeftCorner()[0],handblob.topLeftCorner()[1]))
                    #im = im.skeletonize()
                    im.save(display)
                    #gui.mouseDown()
                    gui.moveTo(int(x0*deltax),int(y0*deltay),0)
                if len(plist) == 0:
                    gui.moveTo(int(x0*deltax),int(y0*deltay),0)
            im.drawCircle((x0,y0), 10, color=scv.Color.RED); im.save(display)
            im.save(display);
        else:
            #gui.mouseUp()
            #im = im.skeletonize()
            im.save(display)
            plist = []

center, radii = calibrate()
detect_movement(center, radii)
