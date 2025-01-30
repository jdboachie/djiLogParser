import os
import csv
import itertools
import struct
import tkinter
import logging
from pprint import pprint
from datetime import datetime
from math import tan, radians, degrees, cos, pi, sin
from klvdata.misb0601 import ( # type:ignore
    PlatformHeadingAngle,
    PlatformPitchAngle,
    SlantRange,
    PlatformRollAngle,
    SensorLatitude,
    FrameCenterElevation,
    SensorLongitude,
    SensorTrueAltitude,
    TargetWidth,
    SensorHorizontalFieldOfView,
    SensorRelativeElevationAngle,
    SensorEllipsoidHeightConversion,
    PlatformRollAngleFull,
    PlatformPitchAngleFull,
    SensorRelativeAzimuthAngle,
    SensorVerticalFieldOfView,
    PrecisionTimeStamp,
    SensorRelativeRollAngle,
    FrameCenterLatitude,
    Checksum,
    FrameCenterLongitude,
    CornerLatitudePoint1Full,
    CornerLongitudePoint1Full,
    CornerLatitudePoint2Full,
    CornerLongitudePoint2Full,
    CornerLatitudePoint3Full,
    CornerLongitudePoint3Full,
    CornerLatitudePoint4Full,
    CornerLongitudePoint4Full,
)
from klvdata.common import datetime_to_bytes, int_to_bytes, float_to_bytes # type:ignore
from datetime import datetime
from tkinter import filedialog

from constants import (
  KLV_FOLDER,
  CSV_ENCODING,
  CRC64TABLE,
  EARTH_MEAN_RADIUS,
  UASLocalMetadataSet,
)

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(levelname).4s %(filename)s %(lineno)d %(message)s',
    datefmt='%H:%M:%S'
)

DJIFrameList = []
DJIFrameList.extend(
  [ 'start',
    'OSD',
    'Home',
    'Gimbal',
    'RemoteController',
    'CUSTOM',
    'DEFORM',
    'Battery',
    'SmartBattery',
    'Message',
    'WARNING',
    'RemoteController-GPS',
    'RemoteController-DEBUG',
    'RECOVER',
    'APPGPS',
    'FIRMWARE',
    'OFDM',
    'VISION',
    'VISION-WARNING',
    'MC',
    'APP-operation',
    '??',
    '??',
    '??',
    'APPSER',
    '??',
    '??'
  ]
)
DJIFrameList.extend(['??' for x in range(0,228)])
DJIFrameList[40]='Component'
DJIFrameList[57]='JPG'

DJIFrame: dict[str, list] = {
  'OSD':[],
  'Home':[],
  'Gimbal':[],
  'RemoteController':[],
  'CUSTOM':[],
  'DEFORM':[],
  'Battery':[],
  'SmartBattery':[],
  'Message':[],
  'WARNING':[],
  'RemoteController-GPS':[],
  'RemoteController-DEBUG':[],
  'RECOVER':[],
  'APPGPS':[],
  'FIRMWARE':[],
  'OFDM':[],
  'VISION':[],
  'VISION-WARNING':[],
  'MC':[],
  'APP-operation':[],
  'COMPONENT':[],
  'JPG':[],
  'APPSER':[],
  'unknown':[],
  'Detail':[]
}

def Battery(payload):
  relativeCapacity,\
  currentPV,\
  currentCapacity,\
  fullCapacity,\
  life,\
  loopNum,\
  errorType,\
  current,\
  voltageCell1,\
  voltageCell2,\
  voltageCell3,\
  voltageCell4,\
  voltageCell5,\
  voltageCell6,\
  serialNo,\
  productDate,\
  temperature,\
  connStatus = struct.unpack_from('<BHHHBHLHHHHHHHHHHB',payload,0)
  if len(DJIFrame['Battery']) == 0:
    #add Header
    DJIFrame['Battery'].append(['PayloadSize',"relativeCapacity","currentPV","currentCapacity",
      "fullCapacity","life","loopNum",
      "errorType","current",
      "voltageCell1","voltageCell2","voltageCell3","voltageCell4","voltageCell5","voltageCell6",
      "serialNo","productDate","temperature","connStatus"])
  DJIFrame['Battery'].append([len(payload),relativeCapacity,currentPV/1000,currentCapacity,fullCapacity,life,loopNum,
    errorType,current/1000,voltageCell1/1000,voltageCell2/1000,voltageCell3/1000,voltageCell4/1000,voltageCell5/1000,
    voltageCell6/1000,serialNo,productDate,temperature/100,connStatus])

def SmartBattery(payload):
  usefulTime,\
  goHomeTime,\
  landTime,\
  goHomeBattery,\
  landBattery,\
  safeFlyRadius,\
  volumeConsume,\
  status,\
  goHomeStatus,\
  goHomeCountdown,\
  voltage,\
  battery,\
  byte1,\
  byte2,\
  voltagePercent = struct.unpack_from('<HHHHHLfLBBHBBBB',payload,0)
  if len(DJIFrame['SmartBattery']) == 0:
    #add Header
    DJIFrame['SmartBattery'].append(['PayloadSize',"usefulTime","goHomeTime","landTime",
      "goHomeBattery","landBattery","safeFlyRadius",
      "volumeConsume","status",
      "goHomeStatus","goHomeCountdown","voltage","battery","lowWarningGoHome","lowWarning",
      "seriousLowWarningLanding","seriousLowWarning","voltagePercent"])

  DJIFrame['SmartBattery'].append([len(payload),usefulTime,goHomeTime,landTime,goHomeBattery,landBattery,safeFlyRadius,volumeConsume,status,
    goHomeStatus,goHomeCountdown,voltage/1000,
    (byte1&0x80)>>7,byte1&0x7f,
    (byte2&0x80)>>7,byte2&0x7f,
    voltagePercent])

def Message(payload):
  if len(DJIFrame['Message']) == 0:
    #add Header
    DJIFrame['Message'].append(['PayloadSize',"MESSAGE"])
  DJIFrame['Message'].append([len(payload),payload.decode('utf-8',errors='ignore')])
  # print(DJIFrame['Message'])

def WARNING(payload):
  if len(DJIFrame['WARNING']) == 0:
    #add Header
    DJIFrame['WARNING'].append(['PayloadSize',"WARNING"])
  DJIFrame['WARNING'].append([len(payload),payload.decode('utf-8',errors='ignore')])
  # print(DJIFrame['WARNING'])

def RECOVER(payload):
  if len(payload) < 109:
    raise Exception("RECOVER Length too small!")
  droneType,\
  appType,\
  appVersionL,\
  appVersionM,\
  appVersionH,\
  aircraftSn,\
  aircraftName,\
  activeTimestamp,\
  cameraSn,\
  rcSn,\
  batterySn = struct.unpack_from('<BBBBB16s32sL16s16s16s',payload,0)
  if len(DJIFrame['RECOVER']) == 0:
    DJIFrame['RECOVER'].append(['PayloadSize','droneType','appType','appVersion','aircraftSn','aircraftName','activeTimestamp','cameraSn','rcSn','batterySn'])
  DJIFrame['RECOVER'].append([len(payload),droneType,appType,(appVersionH<<16)|(appVersionM<<8)|appVersionL,aircraftSn.decode('utf-8',errors='ignore'),aircraftName.decode('utf-8',errors='ignore'),datetime.fromtimestamp(activeTimestamp).strftime('%Y-%m-%d %H:%M:%S'),cameraSn.decode('utf-8',errors='ignore'),rcSn.decode('utf-8',errors='ignore'),batterySn.decode('utf-8',errors='ignore')])

def APPGPS(payload):
  latitude,\
  longitude,\
  accuracy = struct.unpack_from('<ddf',payload,0)
  if len(DJIFrame['APPGPS']) == 0:
    DJIFrame['APPGPS'].append(['PayloadSize','latitude','longitude','accuracy'])
  DJIFrame['APPGPS'].append([len(payload),latitude,longitude,accuracy])

def FIRMWARE(payload):
  unknown,\
  versionL,\
  versionM,\
  versionH,\
  Firmstr = struct.unpack_from('<HBBB'+str(len(payload)-5)+'s',payload,0)
  if len(DJIFrame['FIRMWARE']) == 0:
    DJIFrame['FIRMWARE'].append(['PayloadSize','version','unkonwn'])
  DJIFrame['FIRMWARE'].append([len(payload),(versionH<<18)|(versionM<<8)|versionL,[c for c in Firmstr]])

def APPSER(payload):
  if len(DJIFrame['APPSER']) == 0:
    #add Header
    DJIFrame['APPSER'].append(['PayloadSize',"WARNING"])
  DJIFrame['APPSER'].append([len(payload),payload.decode('utf-8',errors='ignore')])
  # print(DJIFrame['WARNING'])

def COMPONENT(payload):
  select = ['','','','','']
  componentType,\
  serialNumberLength = struct.unpack_from('<HB',payload,0)
  string = struct.unpack_from('<'+str(serialNumberLength)+'s',payload,3)
  select[componentType-1] = string.decode('utf-8',errors='ignore')
  if len(DJIFrame['COMPONENT']) == 0:
    DJIFrame['COMPONENT'].append(['PayloadSize','componentType','serialNumberLength','cameraSn','aircraftSn','rcSn','batterySn','unknownSn'])
  DJIFrame['COMPONENT'].append([len(payload),componentType,serialNumberLength,select[0],select[1],select[2],select[3],select[4]])

def JPG(payload):
  lastByte = 0
  resultPic = []
  picIndex = 0
  for x in payload:
    if ((lastByte<<8)|x ) == 0xFFD8:
      resultPic.append([255,216])
    if ((lastByte<<8)|x) == 0xFFD9:
      resultPic[picIndex].append(255)
      resultPic[picIndex].append(217)
      picIndex = picIndex + 1
    if x == 0xff:
      return 0
    if len(resultPic) > 0:
      resultPic[picIndex].append(x)
    lastByte = x
  if len(resultPic) == 0:
    return 0

  header = ["PayloadSize"]
  inside = [len(payload)]
  for i in range(len(resultPic)):
    header.append("Pic_"+str(len(i+1)))
    inside.append(resultPic[i])
  if len(DJIFrame['JPG']) == 0:
    DJIFrame['JPG'].append(header)
  DJIFrame['JPG'].append(inside)

def OSD(payload):
  motorFailReason,ctrlDevice = 0,0
  if len(payload) > 53:
    longitude,\
    latitude,\
    height,\
    xSpeed,\
    ySpeed,\
    zSpeed,\
    pitch,\
    roll,\
    yaw,\
    byte1,\
    flycCommandRAW,\
    byte2,\
    byte3,\
    byte4,\
    byte5,\
    gpsNum,\
    flightActionRAW,\
    motorStartFailedCause,\
    byte6,\
    battery,\
    sWaveHeight,\
    flyTime,\
    motorRevolution,\
    unkonwn2Bytes,\
    flycVersion,\
    droneType,\
    imuInitFailReason,\
    motorFailReason,\
    unkonwn1byte,\
    ctrlDevice= struct.unpack_from('<ddhhhhhhhBBBBBBBBBBBBHBHBBBBBB',payload,0)
  else:
    longitude,\
    latitude,\
    height,\
    xSpeed,\
    ySpeed,\
    zSpeed,\
    pitch,\
    roll,\
    yaw,\
    byte1,\
    flycCommandRAW,\
    byte2,\
    byte3,\
    byte4,\
    byte5,\
    gpsNum,\
    flightActionRAW,\
    motorStartFailedCause,\
    byte6,\
    battery,\
    sWaveHeight,\
    flyTime,\
    motorRevolution,\
    unkonwn2Bytes,\
    flycVersion,\
    droneType,\
    imuInitFailReason =  struct.unpack_from('<ddhhhhhhhBBBBBBBBBBBBHBHBBB',payload,0)

  if len(DJIFrame['OSD']) == 0:
    # Add header
    DJIFrame['OSD'].append(['PayloadSize',"longitude","latitude","height","xSpeed","ySpeed","zSpeed","pitch","roll","yaw",
      "rcState","flycState",
      "flycCommandRAW",
      "goHomeStatus","isSwaveWork","isMotorUp","groundOrSky","canIOCWork",
      "modeChannel","isImuPreheated","voltageWarning","isVisionUsed",
      "batteryType","gpsLevel","waveError","compassError",
      "isAcceletorOverRange","isVibrating","isBarometerDeadInAir","isNotEnoughForce","isMotorBlocked","isPropellerCatapult","isGoHomeHeightModified","isOutOfLimit",
      "gpsNum","flightActionRAW","motorStartFailedCause",
      "waypointLimitMode","nonGPSCause",
      "battery",
      "sWaveHeight",
      "flyTime",
      "motorRevolution",
      "unkonwn2Bytes",
      "flycVersion","droneType","imuInitFailReason",
      "motorFailReason","ctrlDevice"])

  DJIFrame['OSD'].append([len(payload),longitude*180/pi,latitude*180/pi,height*0.1,xSpeed*0.1,ySpeed*0.1,zSpeed*0.1,pitch*0.1,roll*0.1,yaw*0.1,
    (byte1&0x80) >> 7,byte1&0x7f,
    flycCommandRAW,
    (byte2&0xe0) >> 5,(byte2&0x10)>>4,(byte2&0x08)>>3,(byte2&0x06)>>1,byte2&0x01,
    (byte3&0x60) >> 5,(byte3&0x10)>>4,(byte3&0x06)>>1,byte3&0x01,
    (byte4&0xc0) >> 6,(byte4&0x3c)>>2,(byte4&0x02)>>1,byte4&0x01,
    (byte5&0x80) >> 7,(byte5&0x40) >> 6,(byte5&0x20) >> 5,(byte5&0x10) >> 4,(byte5&0x08) >> 3,(byte5&0x04) >> 2,(byte5&0x02) >> 1,byte5&0x01,
    gpsNum,flightActionRAW,motorStartFailedCause,
    (byte6&0x10) >> 4,byte6&0x0f,
    battery,sWaveHeight*0.1,flyTime*0.1,motorRevolution,unkonwn2Bytes,flycVersion,droneType,imuInitFailReason,motorFailReason,ctrlDevice])

def Home(payload):
  longitude,\
  latitude,\
  height,\
  byte1,\
  byte2,\
  goHomeHeight,\
  courseLockAngle,\
  dataRecorderStatus,\
  dataRecorderRemainCapacity,\
  dataRecorderRemainTime,\
  dataRecorderFileIndex,\
  ss,\
  maxAllowedHeight,\
  restStr,\
  unknown = struct.unpack_from('<ddfBBHHBBHH5sf'+str(len(payload)-50)+'s9s',payload,0)
  if len(DJIFrame['Home']) == 0:
    #add Header
    DJIFrame['Home'].append(['PayloadSize',"longitude","latitude","height",
      "hasGoHome","goHomeStatus","isDynamicHomePointEnabled","aircraftHeadDirection","goHomeMode","isHomeRecord",
      "iocMode","isIOCEnabled","isBeginnerMode","isCompassCeleing","compassCeleStatus",
      "goHomeHeight","courseLockAngle","dataRecorderStatus","dataRecorderRemainCapacity","dataRecorderRemainTime","dataRecorderFileIndex",
      "maxAllowedHeight","unkonwnString","unknownbyte"])
  DJIFrame['Home'].append([len(payload),longitude*180/pi,latitude*180/pi,height*0.1,
    (byte1&0x80) >> 7,(byte1&0x70) >>4,(byte1&0x08) >> 3,(byte1&0x04) >> 2,(byte1&0x02) >> 1,byte1&0x01,
    (byte2&0xe0) >> 5,(byte2&0x10)>>4,(byte2&0x08)>>3,(byte2&0x04)>>2,byte2&0x03,
    goHomeHeight,courseLockAngle*0.1,dataRecorderStatus,dataRecorderRemainCapacity,dataRecorderRemainTime,dataRecorderFileIndex,maxAllowedHeight,restStr.decode('utf-8',errors='ignore'),[x for x in unknown]])

def Gimbal(payload):
  pitch,\
  roll,\
  yaw,\
  GIMBALmode,\
  rollAdjust,\
  yawAngle,\
  byte1,\
  byte2 = struct.unpack_from('<hhhBbhBB',payload,0)
  if len(DJIFrame['Gimbal']) == 0:
    #add Header
    DJIFrame['Gimbal'].append(['PayloadSize',"pitch","roll","yaw",
      "GIMBALmode","rollAdjust","yawAngle",
      "isStuck","autoCalibrationResult","isAutoCalibration","isYawInLimit","isRollInLimit","isPitchInLimit",
      "isSingleClick","isTripleClick","isDoubleClick","version"])
  DJIFrame['Gimbal'].append([len(payload),pitch*0.1,roll*0.1,yaw*0.1,
    (GIMBALmode&0xc0) >> 6,rollAdjust*0.1,yawAngle*0.1,
    (byte1&0x40) >>6,(byte1&0x10) >> 4,(byte1&0x08) >> 3,(byte1&0x04) >> 2,(byte1&0x02) >> 1,byte1&0x01,
    (byte2&0x80) >> 7,(byte2&0x40)>>6,(byte2&0x20)>>5,byte2&0x0f])

def RemoteController(payload):
  aileron,\
  elevator,\
  throttle,\
  rudder,\
  gimbal,\
  wheelOffset,\
  byte1,\
  byte2 = struct.unpack_from('<hhhhhBBB',payload,0)
  if len(DJIFrame['RemoteController']) == 0:
    #add Header
    DJIFrame['RemoteController'].append(['PayloadSize',"aileron","elevator","throttle",
      "rudder","gimbal","wheelOffset",
      "RCmode","goHome",
      "record","shutter","playback","custom1","custom2"])
  DJIFrame['RemoteController'].append([len(payload),(aileron-1024)/0.066,(elevator-1024)/0.066,(throttle-1024)/0.066,(rudder-1024)/0.066,(gimbal-1024)/0.066,
    (wheelOffset&0x3e) >> 1,
    (byte1&0x30) >>4,(byte1&0x08) >> 3,
    (byte2&0x80) >> 7,(byte2&0x40)>>6,(byte2&0x20)>>5,(byte2&0x10)>>4,(byte2&0x08) >>3])

def CUSTOM(payload):
  unkonwn2Bytes,\
  hSpeed,\
  distance,\
  updateTime  = struct.unpack_from('<HffQ',payload,0)
  if len(DJIFrame['CUSTOM']) == 0:
    #add Header
    DJIFrame['CUSTOM'].append(['PayloadSize',"hSpeed","distance","updateTime"])
  if updateTime > 17000991717187400:
    DJIFrame['CUSTOM'].append([len(payload),hSpeed,distance,updateTime])
  else:
    DJIFrame['CUSTOM'].append([len(payload),hSpeed,distance,datetime.fromtimestamp(updateTime/1000).strftime('%Y-%m-%d %H:%M:%S')])

def DEFORM(payload):
  byte1  = struct.unpack_from('<B',payload,0)
  if len(DJIFrame['DEFORM']) == 0:
    #add Header
    DJIFrame['DEFORM'].append(['PayloadSize',"deformMode","deformStatus","isDeformProtected"])
  DJIFrame['CUSTOM'].append([len(payload),(byte1&0x30)>>4,(byte1&0x0e)>>1,byte1&0x01])

def UNKNOWN(payload,type):
  if len(DJIFrame['unknown']) == 0:
    DJIFrame['unknown'].append(['type','length','payload'])
  DJIFrame['unknown'].append([type,len(payload),[x for x in payload]])

def unscramble(payload,record_type):
  result = []
  if len(payload) < 1:
    raise Exception("BAD Payload! Payload format error ",payload)
  key = payload[0]
  crc = (key + record_type)& 0xff
  dataforBUFFER = 0x123456789ABCDEF0*key

  bufferToCRC = [0 for x in range(0,8)]
  for i in range(8):
    bufferToCRC[i] = dataforBUFFER & 0xff
    dataforBUFFER >>= 8

  for i in range(8):
    tableIndex = (bufferToCRC[i] ^ crc) & 0xff
    crc = CRC64TABLE[tableIndex] ^(crc>>8)
  scrambleBytes=[0 for x in range(0,8)]
  for i in range(8):
    scrambleBytes[i] = crc & 0xff
    crc >>= 8
  for index in range(1,len(payload)):
    result.append(payload[index] ^ scrambleBytes[(index-1)%8])
  return result

def parseDetails(payload,version):
  logging.info('Parsing details')

  #  DETAILS.cityPart: string (length 20):
  #  DETAILS.street: string (length 20):
  #  DETAILS.city: string (length 20):
  #  DETAILS.area: string (length 20):
  #  DETAILS.isFavorite: 1 byte unsigned:
  #  DETAILS.isNew: 1 byte unsigned:
  #  DETAILS.needsUpload: 1 byte unsigned:
  #  DETAILS.recordLineCount: 4 bytes little-endian unsigned:
  #  unknown (4 bytes):
  #  DETAILS.timestamp: 8 bytes little-endian, multiple of 0.001 seconds, in Unix time format:
  #  DETAILS.longitude: 8 bytes little-endian double, in degrees:
  #  DETAILS.latitude: 8 bytes little-endian double, in degrees:
  #  DETAILS.totalDistance: 4 bytes little-endian float:
  #  DETAILS.totalTime: 4 bytes little-endian unsigned:
  #  DETAILS.maxHeight: 4 bytes little-endian float:
  #  DETAILS.maxHorizontalSpeed: 4 bytes little-endian float:
  #  DETAILS.maxVerticalSpeed: 4 bytes little-endian float:
  #  DETAILS.photoNum: 4 bytes little-endian unsigned:
  #  DETAILS.videoTime: 4 bytes little-endian unsigned:
  #  here total : 143 bytes

  activeTimestamp = 0

  cityPart,\
  street,\
  city,\
  area,\
  isFavorite,\
  isNew,\
  needUpload,\
  recordLineCount,\
  unknown4bytes,\
  timestamp,\
  longitude,\
  latitude,\
  totalDistance,\
  totalTime,\
  maxHeight,\
  maxHorizontalSpeed,\
  maxVerticalSpeed,\
  photoNum,\
  videoTime = struct.unpack_from("<20s20s20s20sBBBLLQddfLfffLL",payload,0)

  if version < 6:
    unknown124byte,\
    aircraftSn,\
    unknown1byte,\
    aircraftName,\
    unknown7bytes,\
    activeTimestamp,\
    cameraSn,\
    rcSn,\
    batterySn,\
    appType,\
    appVersion = struct.unpack_from("<124s10sB25s7sQ10s10s10sB3s",payload,143)
  else:
    unknown137byte,\
    aircraftName,\
    aircraftSn,\
    cameraSn,\
    rcSn,\
    batterySn,\
    appType,\
    appVersion = struct.unpack_from("<137s32s16s16s16s16sB3s",payload,143)
    appversion2 = 0
  if len(DJIFrame['Detail']) == 0:
    DJIFrame['Detail'].append(['cityPart','street','city','area','isFavorite','isNew','needUpload','recordLineCount',
      'timestamp','longitude','latitude','totalDistance','totalTime','maxHeight','maxHorizontalSpeed','maxVerticalSpeed',
      'photoNum','videoTime','aircraftName','aircraftSn','cameraSn','rcSn','batterySn',
      'appType','appVersion','activeTimestamp'])
    DJIFrame['Detail'].append([cityPart.decode('utf-8',errors='ignore'),street.decode('utf-8',errors='ignore'),city.decode('utf-8',errors='ignore'),area.decode('utf-8',errors='ignore'),
      isFavorite,isNew,needUpload,recordLineCount,
      datetime.fromtimestamp(timestamp/1000).strftime('%Y-%m-%d %H:%M:%S'),longitude,latitude,totalDistance,totalTime/1000,maxHeight,maxHorizontalSpeed,maxVerticalSpeed,
      photoNum,videoTime,aircraftName.decode('utf-8',errors='ignore'),aircraftSn.decode('utf-8',errors='ignore'),cameraSn.decode('utf-8',errors='ignore'),rcSn.decode('utf-8',errors='ignore'),batterySn.decode('utf-8',errors='ignore'),
      appType,'.'.join([str(x) for x in appVersion]),datetime.fromtimestamp(activeTimestamp/1000).strftime('%Y-%m-%d %H:%M:%S')])

def parseBody(
    body: bytes,
    version: int,
    detail: int,
    recordEnd: int
  ):
  """
  Parses the body of the log file
  based on my understanding:

  XXXXXXXXX XXXXXXXXXXXXXXX XXXXXXXXXXXXXXXXXX....
  |_______| |_____________| |________________....
      |            |               |
     head        detail          record

  Args:
      body (bytes): body of the log file
      version (int): log file version number
      detail (int): length of the details section
      recordEnd (int): size of file, ie. file size
  """

  logging.info('Parsing body')

  headSize = 0
  isScrambled = 0
  recordStart = 0
  detailStart = 0
  detailEnd = 0
  computeFileSize = 0

  if version < 6:
    headSize = 12
  else:
    headSize = 100
    isScrambled = 1


  if version >= 12:
    recordStart = headSize + detail
    detailStart = headSize
    detailEnd = detailStart + detail
    computeFileSize = recordEnd
  else:
    recordStart = headSize
    detailStart=recordEnd
    detailEnd = detailStart+detail
    computeFileSize = detailEnd


  if computeFileSize < headSize+3:
    raise Exception("BAD FILE! file format error ",computeFileSize)

  recordArea = body[recordStart:recordEnd+1]

  i = 0
  frameNumber = 1
  while i < len(recordArea):
    record_type = recordArea[i]
    #JPG format
    if record_type == 57:
      JPG(recordArea[i+2:])
      break
    else:
      record_size = recordArea[i+1]
      payload = recordArea[i+2:i+2+record_size]
      record_end = recordArea[i+2+record_size]

      if record_end != 255:
        raise Exception("File not right")

      i += record_size+3
      # logging.info(f"Dealing with frame {frameNumber} with type {DJIFrameList[record_type]}")
      frameNumber=frameNumber+1
      # unscramble payload!
      if isScrambled > 0:
        payload = bytearray(unscramble(payload,record_type))

      if record_type == 1:
        OSD(payload)
      elif record_type == 5:
        CUSTOM(payload)
      elif record_type == 4:
        RemoteController(payload)
      elif record_type == 3:
        Gimbal(payload)
      elif record_type == 2:
        Home(payload)
      elif record_type == 6:
        DEFORM(payload)
      # elif record_type == 7:
      # 	Battery(payload)
      # elif record_type == 8:
      # 	SmartBattery(payload)
      elif record_type == 9:
        Message(payload)
      elif record_type ==10:
        WARNING(payload)
      # elif record_type == 11:
      # 	GPSFrame(payload)
      # elif record_type == 12:
      # 	DebugFrame(payload)
      elif record_type == 13:
        RECOVER(payload)
      elif record_type == 14:
        APPGPS(payload)
      elif record_type == 15:
        FIRMWARE(payload)
      elif record_type == 24:
        APPSER(payload)
      elif record_type == 40:
        COMPONENT(payload)
      else:
        UNKNOWN(payload,record_type)

  parseDetails(body[detailStart:detailEnd],version)

def decodeFile(path: str) -> None:
    with open(path, 'rb') as f:
      logging.info(f'Reading from {path}')
      body = f.read()
      header, \
      detailAreaBytes, \
      version = struct.unpack_from('<Qhb', body, 0)

      parseBody(body, version, detailAreaBytes, header)

def writeToMultipleCSV():
  logging.info('Writing to CSV')
  root = os.getcwd()
  dir = os.path.join(root,"csv_output")

  if not os.path.exists(dir):
    os.makedirs(dir)

  for item in DJIFrame:
    if len(DJIFrame[item]) > 0:
      with open(os.path.join(dir,item+".csv"),'w') as csvfile:
        w = csv.writer(csvfile)
        for row in DJIFrame[item]:
          w.writerow(row)
        csvfile.close()

def writeToSingleCSV(path: str):
    logging.info('Writing to CSV')
    root = os.getcwd()
    dir = os.path.join(root, "csv_output")

    if not os.path.exists(dir):
        os.makedirs(dir)

    combined_data = []
    combined_headers: list[str] = []

    for item in DJIFrame:
        if len(DJIFrame[item]) > 0:
            headers = DJIFrame[item][0]
            new_headers = [f"{item}.{header}" for header in headers]
            combined_headers.extend(new_headers)

            for row in DJIFrame[item][1:]:
              combined_data.append([f"{item}.{value}" if isinstance(value, str) else value for value in row])

    with open(os.path.join(dir, f"{path}.csv"), 'w', newline='') as csvfile:
        w = csv.writer(csvfile)
        w.writerow(combined_headers)
        for row in combined_data:
            w.writerow(row)

def writeToKLV():
  logging.info('Writing to KLV')

  out_record = 'csv_output/DJIFlightRecord_2020-03-14_[12-03-20].csv'
  out_record = 'C:/Users/Jude/Downloads/DJI_0001-TxtLogToCsv.csv'

  HFOV = 81
  VFOV = 66

  d = {}
  with open(out_record, encoding=CSV_ENCODING) as csvfile:
      reader = csv.DictReader(csvfile)
      for row in reader:
          date_start = datetime.strptime(
              row["CUSTOM.updateTime"], "%Y/%m/%d %H:%M:%S.%f"
          )
          break

  with open(out_record, encoding=CSV_ENCODING) as csvfile:
      reader = csv.DictReader(csvfile)
      for row in reader:
          for k in row:
              stripK = k.strip()
              stripV = row[k].strip()
              d[stripK] = stripV

          # We create the klv file for every moment
          bufferData = b""
          cnt = 0

          for k, v in d.items():
              try:
                  if k == "CUSTOM.updateTime":
                      # We prevent it from failing in the exact times
                      # that don't have milliseconds
                      try:
                          date_end = datetime.strptime(v, "%Y/%m/%d %H:%M:%S.%f")
                      except Exception:
                          date_end = datetime.strptime(v, "%Y/%m/%d %H:%M:%S")

                      _bytes = bytes(
                          PrecisionTimeStamp(datetime_to_bytes(date_end))
                      )
                      bufferData += _bytes

                  # Platform Heading Angle
                  if k == "OSD.yaw":
                      OSD_yaw = float(v)
                      if OSD_yaw < 0:
                          OSD_yaw = OSD_yaw + 360

                      _bytes = bytes(PlatformHeadingAngle(OSD_yaw))
                      bufferData += _bytes

                  # Platform Pitch Angle
                  if k == "OSD.pitch":
                      OSD_pitch = float(v)
                      _bytes = bytes(PlatformPitchAngle(OSD_pitch))
                      bufferData += _bytes

                  # Platform Roll Angle
                  if k == "OSD.roll":
                      OSD_roll = float(v)
                      _bytes = bytes(PlatformRollAngle(OSD_roll))
                      bufferData += _bytes

                  # Sensor Latitude
                  if k == "OSD.latitude":
                      OSD_latitude = float(v)
                      _bytes = bytes(SensorLatitude(OSD_latitude))
                      bufferData += _bytes

                  # Sensor Longitude
                  if k == "OSD.longitude":
                      OSD_longitude = float(v)
                      _bytes = bytes(SensorLongitude(OSD_longitude))
                      bufferData += _bytes

                  # Sensor True Altitude
                  if k == "OSD.altitude [m]":
                      OSD_altitude = float(v)
                      _bytes = bytes(SensorTrueAltitude(OSD_altitude))
                      bufferData += _bytes

                  # Sensor Ellipsoid Height
                  if k == "OSD.height [m]":
                      OSD_height = float(v)
                      _bytes = bytes(SensorEllipsoidHeightConversion(OSD_height))
                      bufferData += _bytes

                  # Sensor Relative Azimuth Angle
                  if k == "GIMBAL.yaw":
                      # GIMBAL_yaw = float(v)
                      GIMBAL_yaw = 0.0
                      _bytes = bytes(SensorRelativeAzimuthAngle(GIMBAL_yaw))
                      bufferData += _bytes

                  # Sensor Relative Elevation Angle
                  if k == "GIMBAL.pitch":
                      GIMBAL_pitch = float(v)
                      _bytes = bytes(SensorRelativeElevationAngle(GIMBAL_pitch))
                      bufferData += _bytes

                  # Sensor Relative Roll Angle
                  if k == "GIMBAL.roll":
                      GIMBAL_roll = float(v)
                      _bytes = bytes(SensorRelativeRollAngle(GIMBAL_roll))
                      bufferData += _bytes

              except Exception as e:
                  logging.error(str(e))
                  continue

          try:
              # Diference time
              td = date_end - date_start
              end_path = KLV_FOLDER + "/%.1f.klv" % (
                  round(td.total_seconds(), 1)
              )

              # CheckSum
              v = abs(hash(end_path)) % (10 ** 4)
              _bytes = bytes(Checksum(v))
              bufferData += _bytes

              # Sensor Horizontal Field of View
              v = HFOV
              _bytes = bytes(SensorHorizontalFieldOfView(float(v)))
              bufferData += _bytes

              # Sensor Vertical Field of View
              v = VFOV
              _bytes = bytes(SensorVerticalFieldOfView(float(v)))
              bufferData += _bytes

              # TODO : Check these calculations
              # Slant Range
              anlge = 180 + (OSD_pitch + GIMBAL_pitch)
              slantRange = abs(OSD_altitude / (cos(radians(anlge))))

              _bytes = bytes(SlantRange(slantRange))
              bufferData += _bytes

              # Target Width
              # targetWidth = 0.0
              targetWidth = 2.0 * slantRange * tan(radians(HFOV / 2.0))

              try:
                  _bytes = bytes(TargetWidth(targetWidth))
              except Exception:
                  _bytes = bytes(TargetWidth(0.0))

              bufferData += _bytes

              # Frame Center Latitude
              angle = 90 + (OSD_pitch + GIMBAL_pitch)
              tgHzDist = OSD_altitude * tan(radians(angle))

              dy = tgHzDist * cos(radians(OSD_yaw))
              framecenterlatitude = OSD_latitude + degrees(
                  (dy / EARTH_MEAN_RADIUS)
              )

              _bytes = bytes(FrameCenterLatitude(framecenterlatitude))
              bufferData += _bytes

              # Frame Center Longitude
              dx = tgHzDist * sin(radians(OSD_yaw))
              framecenterlongitude = OSD_longitude + degrees(
                  (dx / EARTH_MEAN_RADIUS)
              ) / cos(radians(OSD_latitude))

              _bytes = bytes(FrameCenterLongitude(framecenterlongitude))
              bufferData += _bytes

              # Frame Center Elevation
              frameCenterElevation = 0.0
              _bytes = bytes(FrameCenterElevation(frameCenterElevation))
              bufferData += _bytes

              # CALCULATE CORNERS COORDINATES
              # FIXME : If we add this values, the klv parse has a overflow
              # Probably the packets is not created correctly
              #                     sensor = (OSD_longitude, OSD_latitude, OSD_altitude)
              #                     frameCenter = (framecenterlongitude, framecenterlatitude, frameCenterElevation)
              #                     FOV = (VFOV, HFOV)
              #                     others = (OSD_yaw, GIMBAL_yaw, targetWidth, slantRange)
              #                     cornerPointUL, cornerPointUR, cornerPointLR, cornerPointLL = CornerEstimationWithoutOffsets(sensor=sensor, frameCenter=frameCenter, FOV=FOV, others=others)
              #
              #                     # Corner Latitude Point 1 (Full) CornerLatitudePoint1Full
              #                     _bytes = bytes(CornerLatitudePoint1Full(cornerPointUL[0]))
              #                     bufferData += _bytes
              #
              #                     # Corner Longitude Point 1 (Full)
              #                     _bytes = bytes(CornerLongitudePoint1Full(cornerPointUL[1]))
              #                     bufferData += _bytes
              #
              #                     # Corner Latitude Point 2 (Full)
              #                     _bytes = bytes(CornerLatitudePoint2Full(cornerPointUR[0]))
              #                     bufferData += _bytes
              #
              #                     # Corner Longitude Point 2 (Full)
              #                     _bytes = bytes(CornerLongitudePoint2Full(cornerPointUR[1]))
              #                     bufferData += _bytes
              #
              #                     # Corner Latitude Point 3 (Full)
              #                     _bytes = bytes(CornerLatitudePoint3Full(cornerPointLR[0]))
              #                     bufferData += _bytes
              #
              #                     # Corner Longitude Point 3 (Full)
              #                     _bytes = bytes(CornerLongitudePoint3Full(cornerPointLR[1]))
              #                     bufferData += _bytes
              #
              #                     # Corner Latitude Point 4 (Full)
              #                     _bytes = bytes(CornerLatitudePoint4Full(cornerPointLL[0]))
              #                     bufferData += _bytes
              #
              #                     # Corner Longitude Point 4 (Full)
              #                     _bytes = bytes(CornerLongitudePoint4Full(cornerPointLL[1]))
              #                     bufferData += _bytes

              # Platform Pitch Angle (Full)
              _bytes = bytes(PlatformPitchAngleFull(OSD_pitch))
              bufferData += _bytes

              # Platform Roll Angle (Full)
              _bytes = bytes(PlatformRollAngleFull(OSD_roll))
              bufferData += _bytes

              # set packet header
              writeData = UASLocalMetadataSet
              sizeTotal = len(bufferData)
              writeData += int_to_bytes(sizeTotal)
              writeData += bufferData

              # Write packet
              f_write = open(end_path, "wb+")
              f_write.write(writeData)
              f_write.close()

              cnt += 1

          except Exception as e:
            logging.info(str(e))

def main():
  root = tkinter.Tk()
  root.withdraw() # don't show

  path = filedialog.askopenfilename() # str

  if os.path.exists(path):
    decodeFile(path)
    writeToSingleCSV(path.split('/')[-1].split('.')[0])
    writeToKLV()
    logging.info('Done')
  else:
    logging.error('File not found')
    raise FileNotFoundError()

if __name__ == "__main__":
  main()