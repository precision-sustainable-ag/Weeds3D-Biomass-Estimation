from video import Video

# Testing video
print("Hello World")
folderpath = '.\\data\\Low_density_GX010447'
file = None
vid = Video(folderpath, file)
vid.load_frames_from_file(load_meta=True, meta_file_type='txt')
vid.fill_in_individual_dropouts()
vid.find_red_ball_ids(vid.red_ball_elevation_method)
vid.isolate_frames_around_red_balls()
print(vid.frame_cuts)

