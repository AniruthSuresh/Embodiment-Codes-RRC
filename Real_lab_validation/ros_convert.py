import bagpy
from bagpy import bagreader
import pandas as pd



bag = bagreader("../real_lab_bag_extract/today.bag")

print(f"Available topics: {bag.topics}")

info = bag.topic_table
print(info)

camera_info_csv = bag.message_by_topic('/camera/color/camera_info')
camera_info_df = pd.read_csv(camera_info_csv)

camera_details_filename = 'camera_info.csv'
camera_info_df.to_csv(camera_details_filename, index=False)
print(f"Extracted Camera Info: {camera_info_df.head()}")



