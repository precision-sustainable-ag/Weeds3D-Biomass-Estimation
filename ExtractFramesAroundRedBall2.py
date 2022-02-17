from video import Video
from circle import Circle
from point import Point
import circle


# Testing video
print("Hello World")
vid = Video(path_to_folder=".\\data\\Red_balls", filename=None)
vid.load_frames_from_file(load_meta=True)
vid.find_red_balls_in_video()
vid.evaluate_red_ball_predictions(saveImages=True)

