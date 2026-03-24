import time

import numpy as np
from arena_api import enums
from arena_api.buffer import BufferFactory
from arena_api.system import system

TAB1 = "  "
TAB2 = "    "
TAB3 = "      "

COLOR_THRESHOLD = 5  # min channel mean difference to consider real color


def create_devices_with_tries():
	tries = 0
	tries_max = 6
	sleep_time_secs = 10
	while tries < tries_max:
		devices = system.create_device()
		if not devices:
			print(f'{TAB1}Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} secs for a device to be connected!')
			for sec_count in range(sleep_time_secs):
				time.sleep(1)
				print(f'{TAB1}{sec_count + 1} seconds passed ', '.' * sec_count, end='\r')
			tries += 1
		else:
			print(f'{TAB1}Created {len(devices)} device(s)')
			return devices
	raise Exception(f'{TAB1}No device found! Please connect a device and run the example again.')


def has_real_color(np_bgr):
	"""Return (is_color, max_channel_diff, B, G, R means) for a BGR uint8 array."""
	b, g, r = np_bgr[:, :, 0], np_bgr[:, :, 1], np_bgr[:, :, 2]
	diff = max(abs(b.mean() - g.mean()), abs(g.mean() - r.mean()), abs(b.mean() - r.mean()))
	return diff > COLOR_THRESHOLD, diff, b.mean(), g.mean(), r.mean()


def grab_bgr(device):
	"""Grab one buffer, convert to BGR8 numpy array, requeue. Returns array or None."""
	try:
		buffer = device.get_buffer()
		buf_bgr = BufferFactory.convert(buffer, enums.PixelFormat.BGR8)
		bpp = int(len(buf_bgr.data) / (buf_bgr.width * buf_bgr.height))
		arr = np.asarray(buf_bgr.data, dtype=np.uint8).reshape(buf_bgr.height, buf_bgr.width, bpp)
		BufferFactory.destroy(buf_bgr)
		device.requeue_buffer(buffer)
		return arr
	except Exception:
		try:
			device.requeue_buffer(buffer)
		except Exception:
			pass
		return None


def inspect_stream(device):
	nodemap = device.nodemap
	tl_stream_nodemap = device.tl_stream_nodemap

	tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
	tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
	tl_stream_nodemap['StreamPacketResendEnable'].value = True

	# Get the PixelFormat node and its enumerable entries
	pf_node = nodemap.get_node("PixelFormat")
	initial_pixel_format = pf_node.value

	# Only test formats the camera actually supports
	available_formats = pf_node.enumentry_names
	print(f'{TAB1}Camera supports {len(available_formats)} pixel formats: {available_formats}\n')

	color_hits = []
	skipped = 0

	print(f'{TAB2}=== Scanning pixel formats ===\n')

	for fmt_name in available_formats:
		# Set pixel format while stream is stopped, then start stream
		try:
			pf_node.value = fmt_name
		except Exception as e:
			print(f'{TAB3}[skip ] {fmt_name:<30} | cannot set: {e}')
			skipped += 1
			continue

		device.start_stream(1)
		arr = grab_bgr(device)
		device.stop_stream()

		if arr is None or arr.ndim != 3 or arr.shape[2] != 3:
			print(f'{TAB3}[skip ] {fmt_name:<30} | could not convert to BGR8')
			skipped += 1
			continue

		is_color, diff, bm, gm, rm = has_real_color(arr)
		status = "COLOR" if is_color else "mono "
		print(f'{TAB3}[{status}] {fmt_name:<30} | B={bm:6.1f} G={gm:6.1f} R={rm:6.1f} | diff={diff:.1f}')

		if is_color:
			color_hits.append((fmt_name, diff, bm, gm, rm))

	# Restore original pixel format
	try:
		pf_node.value = initial_pixel_format
	except Exception:
		pass
	print(f'\n{TAB1}Restored pixel format to: {initial_pixel_format}')

	# --- Summary ---
	print(f'\n{TAB2}=== Results ===')
	print(f'{TAB2}Formats skipped: {skipped}')
	if color_hits:
		print(f'{TAB2}Formats with real color data ({len(color_hits)} found):')
		for name, diff, bm, gm, rm in sorted(color_hits, key=lambda x: -x[1]):
			print(f'{TAB3}{name:<30} diff={diff:.1f}  (B={bm:.1f} G={gm:.1f} R={rm:.1f})')
		best = sorted(color_hits, key=lambda x: -x[1])[0]
		print(f'\n{TAB2}=> Best candidate: {best[0]}  (channel diff={best[1]:.1f})')
	else:
		print(f'{TAB2}=> No pixel format produced real color data. The scene or camera may be grayscale.')


if __name__ == '__main__':
	print('Stream color inspection — pixel format scan\n')
	devices = create_devices_with_tries()
	device = system.select_device(devices)
	inspect_stream(device)
	system.destroy_device()
	print('\nDone')
