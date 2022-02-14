import os
import json
import shutil

def load_json(filepath):
    f = open(filepath)
    data = json.load(f)
    f.close()
    return data

def check_folder_exists(filepath):
    if os.path.isdir(filepath):
        return True
    else:
        return False

def check_folder_empty(self, filepath):
    if check_folder_exists(filepath):
        shutil.rmtree(filepath)
    os.makedirs(filepath)