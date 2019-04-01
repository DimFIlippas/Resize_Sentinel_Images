# -*- coding: utf-8 -*-

import sys
import numpy as np
import os
from osgeo import gdal
#gdal.UseExceptions()
from gdalconst import *
import cv2
import subprocess
import time
import argparse
import glob

parser = argparse.ArgumentParser()
parser.add_argument('path', help='path to sen data')
args = parser.parse_args()

def Save_MS_Images(image, sample, dirpath, name):
    sample = gdal.Open(sample, GA_ReadOnly) # Reading a 10m pixel image
    geotr  = sample.GetGeoTransform() # Getting the Geotransform of the 10m pixel size image
    proj   = sample.GetProjection() # Getting the Projection of the 10m pixel size image
    sample = np.array(sample.GetRasterBand(1).ReadAsArray(), dtype=float)  # Reading image as a numpy array
    tableshape = sample.shape #Getting array's shape
    driver = gdal.GetDriverByName('GTiff') #Creating the .tif file
    #dst_ds = driver.Create(name, tableshape[1], tableshape[0], 1, gdal.GDT_Float32,[ 'COMPRESS=LZW' ])
    dst_ds = driver.Create(name, tableshape[1], tableshape[0], 1, gdal.GDT_Int32,[ 'COMPRESS=LZW' ])
    dst_ds.SetGeoTransform(geotr) # Setting Geotransform
    dst_ds.SetProjection(proj) # Setting Projection
    if image.ndim == 3:
        dst_ds = driver.Create(name, tableshape[1], tableshape[0], 3, gdal.GDT_Int32,[ 'COMPRESS=LZW' ])
        dst_ds.SetGeoTransform(geotr) # Setting Geotransform
        dst_ds.SetProjection(proj) # Setting Projection
        dst_ds.GetRasterBand(1).WriteArray(image[:,:,0]) # Writing band
        dst_ds.GetRasterBand(2).WriteArray(image[:,:,1]) # Writing band
        dst_ds.GetRasterBand(3).WriteArray(image[:,:,2]) # Writing band
    else:
        dst_ds = driver.Create(name, tableshape[1], tableshape[0], 1, gdal.GDT_Int32,[ 'COMPRESS=LZW' ])
        dst_ds.SetGeoTransform(geotr) # Setting Geotransform
        dst_ds.SetProjection(proj) # Setting Projection
        dst_ds.GetRasterBand(1).WriteArray(image[:,:]) # Writing band
    dst_ds=None # save, close

class SenPathImage(object):

    def Resize_Images(self, ):
        if not os.path.exists(self.pathImg):
            os.makedirs(self.pathImg)
        # Reading MS images
        B02 = os.path.join(self.path10m,self.images['10m']['B02_10m'][0])
        b02 = gdal.Open(B02, GA_ReadOnly)
        b02 = b02.ReadAsArray()
        sz  = np.shape(b02)
        for key, val in self.images.items():
            tmppath = self.path20m if key == '20m' else self.path60m if key == '60m' else self.path10m
            if key != '10m':
                for image in val:
                    RZI = os.path.join(tmppath,self.images[key][image][0])
                    rzi = gdal.Open(RZI, GA_ReadOnly)
                    rzi = rzi.ReadAsArray()
                    if rzi.shape[0] == 3:
                        b_ms0 = cv2.resize(rzi[0], (sz[1], sz[0]), interpolation=cv2.INTER_NEAREST)
                        b_ms1 = cv2.resize(rzi[1], (sz[1], sz[0]), interpolation=cv2.INTER_NEAREST)
                        b_ms2 = cv2.resize(rzi[2], (sz[1], sz[0]), interpolation=cv2.INTER_NEAREST)
                        b_ms = np.dstack([b_ms0,b_ms1,b_ms2])
                    else:
                        b_ms = cv2.resize(rzi, (sz[1], sz[0]), interpolation=cv2.INTER_NEAREST)
                    #{cv2.INTER_NEAREST, cv2.INTER_LINEAR, cv2.INTER_CUBIC,cv2.INTER_AREA, cv2.INTER_LANCZOS4, cv2.INTER_MAX,cv2.WARP_FILL_OUTLIERS, cv2.WARP_INVERSE_MAP}
                    Save_MS_Images(b_ms, B02, self.pathImg, name = os.path.join(self.pathImg,self.images[key][image][0].split('.')[0].split('_')[-2]+'_10m.tif'))

            else:
                for image in val:
                    cmd = "gdal_translate -co TILED=YES -co COMPRESS=LZW -of GTiff %s %s"\
                           %(os.path.join(tmppath,self.images[key][image][0]),os.path.join(self.pathImg,image + '.tif'))
                    os.system(cmd)

    def FindImg(self, ):
        self.images = {}
        self.images['10m'] = {}
        self.images['20m'] = {}
        self.images['60m'] = {}
        os.chdir(self.path10m)
        self.images['10m'] = {'B02_10m' :glob.glob('*_B02_10m.jp2'),\
                              'B03_10m' :glob.glob('*_B03_10m.jp2'),\
                              'B04_10m' :glob.glob('*_B04_10m.jp2'),\
                              'B08_10m' :glob.glob('*_B08_10m.jp2')}
        os.chdir(self.path20m)
        self.images['20m'] = {'B05_20m' :glob.glob('*_B05_20m.jp2'),\
                              'B06_20m' :glob.glob('*_B06_20m.jp2'),\
                              'B07_20m' :glob.glob('*_B07_20m.jp2'),\
                              'B8A_20m' :glob.glob('*_B8A_20m.jp2'),\
                              'B11_20m' :glob.glob('*_B11_20m.jp2'),\
                              'B12_20m' :glob.glob('*_B12_20m.jp2'),\
                              'SCL_20m' :glob.glob('*_SCL_20m.jp2'),\
                              'TCI_20m' :glob.glob('*_TCI_20m.jp2')}
        os.chdir(self.path60m)
        self.images['60m'] = {'B01_60m' :glob.glob('*_B01_60m.jp2'),\
                              'B09_60m' :glob.glob('*_B09_60m.jp2')}

    def __init__(self,path):
        self.path    = path
        for dirName, subdirList, fileList in os.walk(path):
            if dirName.endswith('IMG_DATA'):
                self.pathImg = os.path.join(dirName,'Rsz')
                self.path10m = os.path.join(dirName,'R10m')
                self.path20m = os.path.join(dirName,'R20m')
                self.path60m = os.path.join(dirName,'R60m')

def main(argv):
    args    = argv[1:]
    try:
        args   = argv[1:]
        SenObj = SenPathImage(args[0])
        SenObj.FindImg()
        SenObj.Resize_Images()
    except Exception as e:
        sys.exit(1)

if __name__ == '__main__':
    sys.exit(main(sys.argv))
