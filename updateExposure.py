import pyrealsense2 as pyrs

pipeline = pyrs.pipeline()
config = pyrs.config()
profile = config.resolve(pipeline) # does not start streaming
profile = pipeline.start(config) # Start streaming
depth_sensor = profile.get_device().first_depth_sensor()
motion_range = depth_sensor.get_option(pyrs.option.motion_range)
val = 0
depth_sensor.set_option(pyrs.option.motion_range, val)