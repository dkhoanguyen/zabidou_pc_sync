from arena_api.system import system
import ctypes
import numpy as np
import cv2
import time
from arena_api.enums import PixelFormat


UNSIGNED_16BIT_MAX = 65535
SIGNED_16BIT_MAX = 32767

class PointData:
	
	'''
	store x, y, z data in millimeters and intensity for a given point
	'''

	def __init__(self, x, y, z, intensity):
		self.x = x
		self.y = y
		self.z = z
		self.intensity = intensity


def find_min_and_max_z_for_signed(pdata_16bit, total_number_of_channels,
								channels_per_pixel, scale_x, scale_y, scale_z):

	# min_depth z value is set to SIGNED_16BIT_MAX to guarantee closer points
	# exist as this is the largest value possible
	min_depth = PointData(x=0, y=0, z=SIGNED_16BIT_MAX, intensity=0)
	max_depth = PointData(x=0, y=0, z=0, intensity=0)

	for i in range(0, total_number_of_channels, channels_per_pixel):

		# Extract channels from point/pixel
		#   The first channel is the x coordinate,
		#   the second channel is the y coordinate,
		#   the third channel is the z coordinate, and
		#   the fourth channel is intensity.
		# We offset by 1 for each channel because pdata_16bit is 16 bit
		#  integer
		x = pdata_16bit[i]
		y = pdata_16bit[i + 1]
		z = pdata_16bit[i + 2]
		intensity = pdata_16bit[i + 3]

		x = int(x * scale_x)
		y = int(y * scale_y)
		z = int(z * scale_z)

		if 0 < z < min_depth.z:
			min_depth.x = x
			min_depth.y = y
			min_depth.z = z
			min_depth.intensity = intensity

		elif z > max_depth.z:
			max_depth.x = x
			max_depth.y = y
			max_depth.z = z
			max_depth.intensity = intensity

	return min_depth, max_depth


def find_min_and_max_z_for_unsigned(pdata_16bit, total_number_of_channels,
									channels_per_pixel, scale_x, scale_y, scale_z,
									offset_x, offset_y):
	# min_depth z value is set to SIGNED_16BIT_MAX to guarantee closer points
	# exist as this is the largest value possible
	min_depth = PointData(x=0, y=0, z=SIGNED_16BIT_MAX, intensity=0)
	max_depth = PointData(x=0, y=0, z=0, intensity=0)

	for i in range(0, total_number_of_channels, channels_per_pixel):

		# Extract channels from point/pixel
		#   The first channel is the x coordinate,
		#   the second channel is the y coordinate,
		#   the third channel is the z coordinate, and
		#   the fourth channel is intensity.
		# We offset by 1 for each channel because pdata_16bit is 16 bit
		#  integer
		x = pdata_16bit[i]
		y = pdata_16bit[i + 1]
		z = pdata_16bit[i + 2]
		intensity = pdata_16bit[i + 3]

		# if z is less than max value, as invalid values get
		# filtered to UNSIGNED_16BIT_MAX
		if z < UNSIGNED_16BIT_MAX:
			# Convert x, y and z to millimeters
			#   Using each coordinates' appropriate scales,
			#   convert x, y and z values to mm. For the x and y
			#   coordinates in an unsigned pixel format, we must then
			#   add the offset to our converted values in order to
			#   get the correct position in millimeters.
			x = int((x * scale_x) + offset_x)
			y = int((y * scale_y) + offset_y)
			z = int(z * scale_y)

			if 0 < z < min_depth.z:
				min_depth.x = x
				min_depth.y = y
				min_depth.z = z
				min_depth.intensity = intensity

			elif z > max_depth.z:
				max_depth.x = x
				max_depth.y = y
				max_depth.z = z
				max_depth.intensity = intensity

	return min_depth, max_depth


devices = system.create_device()
device = devices[0]
nodemap = device.nodemap

# # # Stop stream if running
# # device.stop_stream()

# Set to raw Bayer - true color data from the IMX264MYR sensor
nodemap['PixelFormat'].value = 'Coord3D_ABCY16'
# nodemap['BinningHorizontal'].value = 2
# nodemap['BinningVertical'].value = 2
# nodemap['BinningHorizontalMode'].value = 'Average'  # or 'Average'
# nodemap['BinningVerticalMode'].value = 'Average'

tl_stream_nodemap = device.tl_stream_nodemap
tl_stream_nodemap['StreamBufferHandlingMode'].value = 'NewestOnly'
tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
tl_stream_nodemap['StreamPacketResendEnable'].value = True


print(f'Get xyz coordinate scales and offsets from nodemap')
nodemap["Scan3dCoordinateSelector"].value = "CoordinateA"
scale_x = nodemap["Scan3dCoordinateScale"].value
offset_x = nodemap["Scan3dCoordinateOffset"].value
nodemap["Scan3dCoordinateSelector"].value = "CoordinateB"
scale_y = nodemap["Scan3dCoordinateScale"].value
offset_y = nodemap["Scan3dCoordinateOffset"].value
nodemap["Scan3dCoordinateSelector"].value = "CoordinateC"
scale_z = nodemap["Scan3dCoordinateScale"].value


device.start_stream(1)

try:
    while True:
        start_time = time.time()
        buffer = device.get_buffer()

        # raw = np.array(buffer.data, dtype=np.uint8).view(np.uint16).reshape(buffer.height, buffer.width, 4)
        # intensity = raw[:, :, 3]  # Y (intensity) channel
        # bgr_frame = cv2.cvtColor(cv2.normalize(intensity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U), cv2.COLOR_GRAY2BGR)

        # device.requeue_buffer(buffer)

        # cv2.imshow("frame", bgr_frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        channels_per_pixel = int(buffer.bits_per_pixel / 16)
        total_number_of_channels = buffer.width * buffer.height * channels_per_pixel

		# find points with min and max z values
        print(f'Finding points with min and max z values')

        if buffer.pixel_format == PixelFormat.Coord3D_ABCY16s:

			# Buffer.pdata is a (uint8, ctypes.c_ubyte) pointer.
			# This pixelformat has 4 channels, and each channel is 16 bits.
			# It is easier to deal with Buffer.pdata if it is cast to 16bits
			# so each channel value is read correctly.
			# The pixelformat is suffixed with "S" to indicate that the data
			# should be interpereted as signed.
            pdata_as_int16 = ctypes.cast(buffer.pdata,
                                        ctypes.POINTER(ctypes.c_int16))

            # offset is needed to generate the negative coordinates in the
            # unsigned integer only
            min_depth, max_depth = find_min_and_max_z_for_signed(pdata_as_int16,
                                                                total_number_of_channels,
                                                                channels_per_pixel,
                                                                scale_x, scale_y, scale_z)

        elif buffer.pixel_format == PixelFormat.Coord3D_ABCY16:

            # Buffer.pdata is a (uint8, ctypes.c_ubyte) pointer.
            # This pixelformat has 4 channels, and each channel is 16 bits.
            # It is easier to deal with Buffer.pdata if it is cast to 16bits
            # so each channel value is read correctly.
            # The pixelformat is suffixed with "S" to indicate that the data
            # should be interpereted as signed. This one does not have "S", so
            # we cast it to unsigned.
            pdata_as_uint16 = ctypes.cast(buffer.pdata,
                                        ctypes.POINTER(ctypes.c_uint16))

            # offset is needed to generate the negative coordinates in the
            # unsigned integer only
            min_depth, max_depth = find_min_and_max_z_for_unsigned(pdata_as_uint16,
                                                                total_number_of_channels,
                                                                channels_per_pixel,
                                                                scale_x, scale_y, scale_z,
                                                                offset_x, offset_y)

        else:
            raise Exception('This example requires the camera to be in either '
                            f'3D image format Coord3D_ABCY16 or '
                            f'Coord3D_ABCY16s')
		
        # display data
        print(f'Minimum depth point found with '
            f'z distance of {min_depth.z} mm and '
            f'intensity {min_depth.intensity} at coordinates '
            f'( {min_depth.x} mm, {min_depth.y } mm )')

        print(f'Maximum depth point found with '
            f'z distance of {max_depth.z} mm and '
            f'intensity {max_depth.intensity} at coordinates '
            f'( {max_depth.x} mm, {max_depth.y } mm )')

        # Display intensity (Y) channel
        pdata_uint16 = ctypes.cast(buffer.pdata, ctypes.POINTER(ctypes.c_uint16))
        raw = np.ctypeslib.as_array(pdata_uint16, shape=(buffer.height * buffer.width * channels_per_pixel,))
        intensity = raw[3::channels_per_pixel].reshape(buffer.height, buffer.width)
        intensity_8u = cv2.normalize(intensity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imshow("Intensity", intensity_8u)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Requeue the chunk data buffers
        device.requeue_buffer(buffer)

        # print(f"Time: {time.time() - start_time}")
finally:
    cv2.destroyAllWindows()
    device.stop_stream()