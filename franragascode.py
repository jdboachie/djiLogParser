import os
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtWidgets import QDialog, QApplication, QProgressBar

from gui.ui_FmvMultiplexer import Ui_VideoMultiplexer # type:ignore
from utils.QgsFmvUtils import (
    askForFiles,
    getVideoFolder,
    CornerEstimationWithoutOffsets,
) # type:ignore
from klvdata.common import datetime_to_bytes, int_to_bytes, float_to_bytes # type:ignore
import csv
import itertools
from datetime import datetime
from math import tan, radians, degrees, cos, pi, sin
from utils.QgsUtils import QgsUtils as qgsu
# from qgis.core import Qgis as QGis
from io import StringIO
from QgsFmvConstants import UASLocalMetadataSet, EARTH_MEAN_RADIUS

from klvdata.misb0601 import (
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


"""
'TODO : After all the tests I haven't managed to generate a MISB video,
for now I make this adaptation to be able to see it in QGIS FMV
https://gist.github.com/All4Gis/509fbe06ce53a0885744d16595811e6f
"""

# Csv Encoding
encoding = "ISO-8859-1"


class Multiplexor(QDialog, Ui_VideoMultiplexer):
    """ About Dialog """

    def __init__(self, iface, parent=None, Exts=None):
        """ Contructor """
        super().__init__(parent)
        self.setupUi(self)
        self.iface = iface
        self.parent = parent

        self.video_file = None
        self.csv_file = None
        self.Exts = Exts

    def OpenCsvFile(self):
        """ Open Csv File """
        filename, _ = askForFiles(
            self, QCoreApplication.translate("QgsMultiplexor", "Open file"), exts="csv"
        )
        if filename:
            self.csv_file = filename
            self.ln_inputMeta.setText(self.csv_file)
        return

    def OpenVideoFile(self):
        """ Open Video File """
        filename, _ = askForFiles(
            self,
            QCoreApplication.translate("QgsMultiplexor", "Open file"),
            exts=self.Exts,
        )
        if filename:
            self.video_file = filename
            self.ln_inputVideo.setText(self.video_file)
        return

    def CreateCSV(self):
        """ Create csv for each recording """
        self.cmb_telemetry.clear()
        input_video = self.ln_inputVideo.text()
        input_metadata = self.ln_inputMeta.text()

        if input_video == "" or input_metadata == "":
            qgsu.showUserAndLogMessage(
                QCoreApplication.translate(
                    "Multiplexor", "You must complete all the information"
                )
            )
            return

        self.ReadCSVRecordings(input_metadata)
        # Enable create video button
        self.bt_createMISB.setEnabled(True)
        return

    def GetRows(self, csv_file):
        with open(csv_file, "r", encoding=encoding) as f:
            reader = csv.reader(f, delimiter=",")
            data = list(reader)
            row_count = len(data)
        return row_count

    def CreateMISB(self):
        """ Create MISB Video """
        """ Only tested using DJI Data """
        # Create ProgressBar
        self.iface.messageBar().clearWidgets()
        progressMessageBar = self.iface.messageBar().createMessage(
            "Creating video packets..."
        )
        progress = QProgressBar()
        progress.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        progressMessageBar.layout().addWidget(progress)
        # self.iface.messageBar().pushWidget(progressMessageBar, QGis.Info)

        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()

        HFOV = self.sp_hfov.value()
        VFOV = self.sp_vfov.value()

        index = self.cmb_telemetry.currentIndex()
        out_record = self.cmb_telemetry.itemData(index)
        rowCount = self.GetRows(out_record)
        progress.setMaximum(rowCount)

        d = {}
        with open(out_record, encoding=encoding) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                date_start = datetime.strptime(
                    row["CUSTOM.updateTime"], "%Y/%m/%d %H:%M:%S.%f"
                )
                break

        with open(out_record, encoding=encoding) as csvfile:
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
                        qgsu.showUserAndLogMessage(
                            QCoreApplication.translate(
                                "Multiplexor", "Multiplexer error"
                            )
                            + e,
                            onlyLog=True,
                        )
                        continue

                try:
                    # Diference time
                    td = date_end - date_start
                    end_path = self.klv_folder + "/%.1f.klv" % (
                        round(td.total_seconds(), 1)
                    )

                    # CheckSum
                    v = abs(hash(end_path)) % (10 ** 4)
                    _bytes = bytes(Checksum(v))
                    bufferData += _bytes

                    # Sensor Horizontal Field of View
                    v = self.sp_hfov.value()
                    _bytes = bytes(SensorHorizontalFieldOfView(float(v)))
                    bufferData += _bytes

                    # Sensor Vertical Field of View
                    v = self.sp_vfov.value()
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

                    progress.setValue(cnt)

                except Exception as e:
                    qgsu.showUserAndLogMessage(
                        QCoreApplication.translate("Multiplexor", "Multiplexer error")
                        + str(e),
                        onlyLog=True,
                    )

        QApplication.restoreOverrideCursor()
        QApplication.processEvents()
        progress.setValue(rowCount)
        self.iface.messageBar().clearWidgets()
        # We add it to the manager
        _, name = os.path.split(self.video_file)
        self.parent.AddFileRowToManager(
            name, self.video_file, islocal=True, klv_folder=self.klv_folder
        )
        # Close dialog
        self.close()
        return

    def ReadCSVRecordings(self, csv_raw):
        """ Read the csv for each recording """
        rows_list = []
        time_list = []
        with open(csv_raw, encoding=encoding) as csvfile:
            # Prevent “_csv.Error: line contains NULL byte�?
            data = csvfile.read()
            data = data.replace("\x00", "?")
            reader = csv.DictReader(StringIO(data))
            rows = []
            index = 0
            for row in reader:
                for k in row:
                    if k == "CUSTOM.isVideo":
                        if row[k].strip() == "":
                            if not rows:
                                continue
                            else:
                                rows_list.append(rows)
                                rows = []
                        else:
                            rows.append(index)
                    if k == "CUSTOM.updateTime":
                        # Used to set the csv name using Update Time
                        t = row[k].split(" ")[1].split(":")
                        time_list.append(t[0] + "_" + t[1] + "_" + t[2])
                index += 1

        if not rows_list:
            rows_list.append(rows)
        # Create csv
        self.CreateDJICsv(rows_list, csv_raw, time_list)
        return

    def CreateDJICsv(self, rows_list, csv_raw, time_list):
        """ DJI Drone: Create csv result files for each record """
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()

        folder = getVideoFolder(self.video_file)

        qgsu.createFolderByName(folder, "klv")
        qgsu.createFolderByName(folder, "csv")

        self.klv_folder = os.path.join(folder, "klv")
        out_csv = os.path.join(folder, "csv")

        for idx, val in enumerate(rows_list):
            filename = "_".join(["recording", time_list[idx]])
            out_record = os.path.join(out_csv, filename + ".csv")
            # The column that corresponds to the stop is also removed
            with open(csv_raw, "r", encoding=encoding) as f_input, open(
                out_record, "w", newline="", encoding=encoding
            ) as f_output:
                # Prevent “_csv.Error: line contains NULL byte
                data = f_input.read()
                data = data.replace("\x00", "?")
                csv_input = csv.reader(StringIO(data))
                csv.writer(f_output).writerows(itertools.islice(csv_input, 0, 1))
                csv.writer(f_output).writerows(
                    itertools.islice(csv_input, int(val[0]), int(val[-1]))
                )

            self.cmb_telemetry.addItem(filename, out_record)

        self.bt_createMISB.setEnabled(True)
        QApplication.restoreOverrideCursor()
        QApplication.processEvents()
        return