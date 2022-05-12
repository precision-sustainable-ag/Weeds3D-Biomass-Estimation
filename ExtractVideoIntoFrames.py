from video import Video

folderpath = '.\\data'
file = 'GX010343.MP4'
vid = Video(folderpath, file)
vid.extract_frames()
vid.save_frames_to_file('GX010343', subsample_rate=10)