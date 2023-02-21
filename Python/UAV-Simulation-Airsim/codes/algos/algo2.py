"""
Information about projections:
------------------
What is EPSG:4326?
The World Geodetic System (WGS) is a standard for use in cartography, geodesy, and satellite navigation including GPS. 
This standard includes the definition of the coordinate system’s fundamental and derived constants, the ellipsoidal (normal) Earth Gravitational Model (EGM), 
a description of the associated World Magnetic Model (WMM), and a current list of local datum transformations. 
The latest revision is WGS 84 (also known as WGS 1984, EPSG:4326), established and maintained by the United States National Geospatial-Intelligence Agency since 1984, and last revised in 2004. 
Earlier schemes included WGS 72, WGS 66, and WGS 60. WGS 84 is the reference coordinate system used by the Global Positioning System.
------------------
"EPSG:32714"
Description: axis order change (2D) + UTM zone 14S
Area of Use:
- name: World
- bounds: (-180.0, -90.0, 180.0, 90.0)
------------------
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import imageio
import math
import random
import numpy as np
from shapely.geometry import Polygon, Point
import shapefile
import pyproj
import fiona
import os
os.environ['USE_PYGEOS'] = '0'
import geopandas as gpd
import sys
import zipfile
from pathlib import Path
import shutil
import argparse

class HexagonObj():
    def __init__(self, polyHex, number):
        self.ws = polyHex.exterior.coords[0]
        self.wn = polyHex.exterior.coords[1]
        self.nn = polyHex.exterior.coords[2]
        self.en = polyHex.exterior.coords[3]
        self.es = polyHex.exterior.coords[4]
        self.ss = polyHex.exterior.coords[5]
        self.priority = random.randint(0, 100)
        self.geometry = polyHex
        self.center = polyHex.centroid
        self.alpha = self.priority / 100.0
        self.number = number
        self.assignedUAV = 0
        
    def getNumber (self):
        return self.number  
        
    def getPoints (self):
        return self.polyPoints  
        
    def getCenter(self):
        return self.center          

    def getUAVNum (self):
        return self.assignedUAV  

class UAVObj():      
    def __init__(self, rPoint, number):
        self.rPoint = rPoint
        self.number = number 
        self.isAssigned = False 
        self.assignedHex = [] 
        
    def getNumber (self):
        return self.number  
    
def createHexagons(polyMain, rPoly):
    # get boundary of polygon
    xmin, ymin , xmax, ymax = polyMain.bounds
           
    # round number for precision
    rNum = 3
    
    # create empty list of hexagons
    hexList = []
    
    # determine step sizes
    yStep = rPoly+rPoly//2
    xStep = 2*round(rPoly* math.cos(math.radians(30)) ,rNum)
      
    # start of algorithm
    kk = 0 # control variable
    toleranceHex = -3 # tolerable areas for edge points
    jj = ymin
    while jj < ymax+yStep:
        ii = xmin
        while ii < xmax+xStep:
            xCenter = ii if  kk % 2 == 0 else ii + round(rPoly* math.cos(math.radians(150)),rNum)
            yCenter = jj
            cP = (xCenter,yCenter)
            AA =  (round(rPoly* math.cos(math.radians(30)) ,rNum)+cP[0], round(rPoly* math.sin(math.radians(30)) ,rNum)+cP[1])
            BB =  (round(rPoly* math.cos(math.radians(90)) ,rNum)+cP[0], round(rPoly* math.sin(math.radians(90)) ,rNum)+cP[1]) 
            CC =  (round(rPoly* math.cos(math.radians(150)),rNum)+cP[0], round(rPoly* math.sin(math.radians(150)),rNum)+cP[1]) 
            DD =  (round(rPoly* math.cos(math.radians(210)),rNum)+cP[0], round(rPoly* math.sin(math.radians(210)),rNum)+cP[1]) 
            EE =  (round(rPoly* math.cos(math.radians(270)),rNum)+cP[0], round(rPoly* math.sin(math.radians(270)),rNum)+cP[1]) 
            FF =  (round(rPoly* math.cos(math.radians(330)),rNum)+cP[0], round(rPoly* math.sin(math.radians(330)),rNum)+cP[1])  
            hexPolygon = Polygon([AA, BB, CC, DD, EE, FF])     
            if hexPolygon.within(polyMain):
                hexList.append(hexPolygon) 
            else:
                # remove just touching points by using buffer otherwise keep them
                if hexPolygon.buffer(toleranceHex).intersects(polyMain):
                    hexList.append(hexPolygon) 
                else: 
                    pass               
            ii += xStep
        kk += 1
        jj += yStep

    # create hexagon object for each hexagon
    jj = 1
    hexObjList = []
    for ii in hexList:
        hexObj = HexagonObj(ii, jj)
        hexObjList.append(hexObj)   
        jj += 1
        
    return hexObjList    
    
def createUAVs(polycoords, swarmSize): 

    # get random points which are located in polygon
    pts = getRandomPoints(polycoords, swarmSize)
    
    # create uav object for each uav
    uavObjList = []
    for ii in range(1, swarmSize+1):
        uavObj = UAVObj(pts[ii-1], ii)
        uavObjList.append(uavObj)   
    return uavObjList  

def getPolyCoordinates(fName):
    kmzToKml(fName)
    fiona.drvsupport.supported_drivers['KML'] = 'rw'
    geo_df = gpd.read_file("doc.kml",driver='KML')

    # remove doc.kml file
    os.remove("doc.kml")
    
    # get polygon
    geoSeries = geo_df.geometry.iloc[0]
    # get coordinates of polygon
    xx, yy =  geoSeries.exterior.coords.xy
    
    pointsLotLan = []
    for ii in range(len(xx)):
        pointsLotLan.append((xx[ii],yy[ii]))
    
    # do projection
    source_crs = "EPSG:4326"
    target_crs = "EPSG:32714" # denton utm zone 14S according to specific area you should change this 
    pointsXY = []
    proj = pyproj.Transformer.from_crs(source_crs, target_crs, always_xy=True)
    for ii in pointsLotLan:
        x2, y2 = proj.transform(ii[0], ii[1])
        pointsXY.append((x2, y2))
    return pointsXY
    
def kmzToKml (kmzfName):
    import zipfile
    import shutil
    
    def copyFile(src, dst):
        if os.path.isdir(dst):
            dst = os.path.join(dst, os.path.basename(src))
        shutil.copyfile(src, dst)

    p = os.path.abspath('')

    srcKMZ = p.__str__() + "//" + "KMZFiles" + "//" + kmzfName + ".kmz"
    dstKMZ = p.__str__()
    
    # copy file from source to dest
    copyFile(srcKMZ, dstKMZ)

    # convert kmz to zip
    for filename in os.listdir(p):
        base_file, ext = os.path.splitext(filename)    
        if ext == ".kmz":
            os.rename(filename, base_file + ".zip")
       
    dir_name = p.__str__()
    extension = ".zip"

    os.chdir(dir_name) # change directory from working dir to dir with files
    
    # extract zip files
    for item in os.listdir(dir_name): # loop through items in dir
        if item.endswith(extension): # check for ".zip" extension
            file_name = os.path.abspath(item) # get full path of files
            zip_ref = zipfile.ZipFile(file_name) # create zipfile object
            zip_ref.extractall(dir_name) # extract file to dir
            zip_ref.close() # close file
            os.remove(file_name) # delete zipped file
        
def getRandomPoints(polygon, number):
    points = []
    minx, miny, maxx, maxy = polygon.bounds
    while len(points) < number:
        pnt = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
        if polygon.contains(pnt):
            points.append(pnt)
    return points       

def assignTasks(listOfHex, uavs, polyMain, lfps):
    # create plots
    fig1 = plt.figure()
    ax1 = plt.axes()
       
    # sort list according to priorities
    listOfHexSorted = sorted(listOfHex, key=lambda x: x.priority, reverse=True)
    
    # remaining part
    rem = len(listOfHex) %  len(uavs)
    div = len(listOfHex) // len(uavs)
    
    def genColor(colorNum):
        colors = []
        for i in range(colorNum):
            # Generate random values for red, green, and blue
            r = random.randint(0, 255)/255
            g = random.randint(0, 255)/255
            b = random.randint(0, 255)/255
            # Create a color tuple and append it to the list
            color = (r, g, b)
            colors.append(color)
        return colors
   
   # generate colors
    genColors = genColor(len(uavs)+1) 
    
    def getColor(varNum):
        return genColors[varNum % len(genColors)]
        
    global fncCnt
    fncCnt = 0
    
    # Create an initialization function
    def initPlot():
        for ii, poly in enumerate(listOfHexSorted):
            hexPoly = poly.geometry
            ax1.add_patch(plt.Polygon(np.array(hexPoly.exterior.coords), fc="white", ec="orange"))    
            x1 = hexPoly.centroid.x
            y1 = hexPoly.centroid.y
            plt.annotate(ii+1,xy=(x1, y1))            
 
    # Function to update data for animation
    def update(ii):
        global fncCnt
        
        # control update function
        boolValUpdate = (ii == (div+1)*len(uavs))
        
        sP = (fncCnt-1)*len(uavs)
        if sP < 0:
            sP = 0       
        if boolValUpdate and rem != 0: 
            for uav in uavs:
                uav.isAssigned = False    
            for jj in range(rem):   
                hexagon = listOfHexSorted[jj+ii-len(uavs)]
                closest_distance = float('inf')  
                closest_uavNum = 0
                for uav in uavs:
                    if not uav.isAssigned:
                        curr_distance = hexagon.center.distance(uav.rPoint)
                        if curr_distance < closest_distance:
                            closest_distance = curr_distance
                            closest_uavNum = uav.getNumber()                   
                for uav in uavs:
                    if uav.getNumber() == closest_uavNum:
                        uav.rPoint = hexagon.center
                        uav.isAssigned = True 
                        hexagon.assignedUAV = uav.getNumber()
                        newPoly = hexagon.geometry 
                        ax1.patches[jj+ii-len(uavs)].set_xy(np.array(newPoly.exterior.coords))
                        ax1.patches[jj+ii-len(uavs)].set_facecolor(getColor(uav.getNumber())) 
                            
        else:  
            if not boolValUpdate:
                for jj in range(sP, ii):       
                    if jj % len(uavs) == 0:
                        for uav in uavs:
                            uav.isAssigned = False   
                    hexagon = listOfHexSorted[jj]
                    closest_distance = float('inf')  
                    closest_uavNum = 0
                    for uav in uavs:
                        if not uav.isAssigned:
                            curr_distance = hexagon.center.distance(uav.rPoint)
                            if curr_distance < closest_distance:
                                closest_distance = curr_distance
                                closest_uavNum = uav.getNumber()       
                    for uav in uavs:
                        if uav.getNumber() == closest_uavNum:
                            uav.rPoint = hexagon.center
                            uav.isAssigned = True 
                            hexagon.assignedUAV = uav.getNumber()
                            newPoly = hexagon.geometry
                            ax1.patches[jj].set_xy(np.array(newPoly.exterior.coords))
                            ax1.patches[jj].set_facecolor(getColor(uav.getNumber()))
                              
        fncCnt += 1
        
    plt.plot(*polyMain.exterior.xy, color='red')    
    
    
    def frameFunc(hexNumber, uavNumber):
        remainder = hexNumber % uavNumber 
        for ii in range(0, hexNumber-remainder+uavNumber+1, uavNumber):
            yield ii    
              

    # Create animation
    anim = FuncAnimation(fig1, update, frames=frameFunc(len(listOfHex), len(uavs)), init_func=initPlot, repeat=False)
    
    # Save the animation        
    anim.save('Algorithm2.gif', writer='PillowWriter', fps=lfps)        
    
    # Display the plot
    plt.show()
   

def runPoly2(argsPoly, sfps):  
    # get poly coordinates frome kmz file
    polyCoordinates = getPolyCoordinates (argsPoly["filename"])

    # create polygon from poly coordinates 
    mainPolygon = Polygon(polyCoordinates)

    # create hexagons inside of given polygon coordinates
    hexagonList = createHexagons(mainPolygon, argsPoly["radious"])
    
    # create UAVs
    uavList = createUAVs(mainPolygon, argsPoly["size"])
    
    # assign tasks
    assignTasks(hexagonList, uavList, mainPolygon, sfps)
    
    
    

