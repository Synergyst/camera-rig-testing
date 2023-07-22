#!/usr/bin/env python3
import time
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-fb', '--flash-bootloader', default=False, action="store_true")
parser.add_argument('-f',  '--flash-app',        default=False, action="store_true")
parser.add_argument('-l',  '--load-and-exit',    default=False, action="store_true")
args = parser.parse_args()
if args.load_and_exit:
  import os
  # Disabling device watchdog, so it doesn't need the host to ping periodically.
  # Note: this is done before importing `depthai`
  os.environ["DEPTHAI_WATCHDOG"] = "0"
import depthai as dai
def getPipeline():
  pipeline = dai.Pipeline()
  cam_rgb = pipeline.createColorCamera()
  cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
  manip = pipeline.create(dai.node.ImageManip)
  cam_rgb.setInterleaved(False)
  #cam_rgb.setPreviewKeepAspectRatio(False)
  cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP) # 1080_P, 12_MP, 4_K
  cam_rgb.setFps(25)
  cam_rgb.setIsp3aFps(25)
  sensorResX = cam_rgb.getIspWidth()
  sensorResY = cam_rgb.getIspHeight()
  sensorCropX = 3840
  sensorCropY = 2160
  sensorResizeX = 640
  sensorResizeY = 512
  offsetX = -48
  offsetY = 80
  #cam_rgb.setPreviewSize(int(sensorResX), int(sensorResY))
  topX = int((int(sensorResX) / 2) - (int(sensorCropX) / 2) + offsetX)
  topY = int((int(sensorResY) / 2) - (int(sensorCropY) / 2) + offsetY)
  botX = int((int(sensorResX) / 2) + (int(sensorCropX) / 2) + offsetX)
  botY = int((int(sensorResY) / 2) + (int(sensorCropY) / 2) + offsetY)
  normalizedTopX = topX / sensorResX
  normalizedTopY = topY / sensorResY
  normalizedBotX = botX / sensorResX
  normalizedBotY = botY / sensorResY
  print(str(sensorResX) + "x" + str(sensorResY) + " " + str(sensorCropX) + "x" + str(sensorCropY) + " " + str(topX) + ":" + str(topY) + "/" + str(botX) + ":" + str(botY) + " " + str(normalizedTopX) + ":" + str(normalizedTopY) + "/" + str(normalizedBotX) + ":" + str(normalizedBotY))
  #cam_rgb.initialControl.setManualFocus(130)
  uvc = pipeline.createUVC() # Create an UVC (USB Video Class) output node
  manip.setMaxOutputFrameSize(int(int(sensorCropX) * int(sensorCropY) * 1.5)) # max: 12441600
  #manip.setMaxOutputFrameSize(int(int(sensorResizeX) * int(sensorResizeY) * 1.5)) # max: 12441600
  manip.initialConfig.setCropRect(float(normalizedTopX), float(normalizedTopY), float(normalizedBotX), float(normalizedBotY))
  manip.initialConfig.setResize(int(sensorResizeX), int(sensorResizeY))
  manip.initialConfig.setFrameType(dai.ImgFrame.Type.YUV420p)
  #cam_rgb.preview.link(manip.inputImage)
  cam_rgb.isp.link(manip.inputImage)
  manip.out.link(uvc.input)
  # Note: if the pipeline is sent later to device (using startPipeline()), it is important to pass the device config separately when creating the device
  config = dai.Device.Config()
  config.board.uvc = dai.BoardConfig.UVC(int(sensorResizeX), int(sensorResizeY)) # config.board.uvc = dai.BoardConfig.UVC()  # enable default 1920x1080 NV12
  config.board.uvc.frameType = dai.ImgFrame.Type.YUV420p
  config.board.uvc.cameraName = "OAKamera"
  pipeline.setBoardConfig(config.board)
  return pipeline
# Will flash the bootloader if no pipeline is provided as argument
def flash(pipeline=None):
  (f, bl) = dai.DeviceBootloader.getFirstAvailableDevice()
  bootloader = dai.DeviceBootloader(bl, True)
  # Create a progress callback lambda
  progress = lambda p : print(f'Flashing progress: {p*100:.1f}%')
  startTime = time.monotonic()
  if pipeline is None:
    print("Flashing bootloader...")
    bootloader.flashBootloader(progress)
  else:
    print("Flashing application pipeline...")
    bootloader.flash(progress, pipeline)
  elapsedTime = round(time.monotonic() - startTime, 2)
  print("Done in", elapsedTime, "seconds")
if args.flash_bootloader or args.flash_app:
  if args.flash_bootloader: flash()
  if args.flash_app: flash(getPipeline())
  print("Flashing successful. Please power-cycle the device")
  quit()
if args.load_and_exit:
  device = dai.Device(getPipeline())
  print("\nDevice started. Attempting to force-terminate this process...")
  print("Open an UVC viewer to check the camera stream.")
  print("To reconnect with depthai, a device power-cycle may be required in some cases")
  # We do not want the device to be closed, so terminate the process uncleanly.
  # (TODO add depthai API to be able to cleanly exit without closing device)
  import signal
  os.kill(os.getpid(), signal.SIGTERM)
# Standard UVC load with depthai
with dai.Device(getPipeline()) as device:
  print("\nDevice started, please keep this process running")
  print("and open an UVC viewer to check the camera stream.")
  print("\nTo close: Ctrl+C")
  # Doing nothing here, just keeping the host feeding the watchdog
  while True:
    try:
      time.sleep(0.1)
    except KeyboardInterrupt:
      break
