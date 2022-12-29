from bagpy import bagreader
import pandas as pd
import numpy as np
import os
import shutil
import xml.etree.ElementTree as ET
import re

buildingEditorModels = "/home/onur/building_editor_models" 





listdir=os.listdir(buildingEditorModels)
listdir=sorted(listdir, key=lambda x : int(re.findall(r'\d+', x)[0]) )

for j, folder in enumerate(listdir):
  break_count = 0
  if os.path.isdir(buildingEditorModels + "/" + folder) != True:
    break
  for i in range(5):
   
   if(os.path.isdir(buildingEditorModels + "/" + folder + "/sensors/" + str(i) ) != True
        or os.path.isfile(buildingEditorModels + "/" + folder + "/sensors/" + str(i) + "/front_scan.bag") != True
        or os.path.isfile(buildingEditorModels + "/" + folder + "/sensors/" + str(i) + "/odometry.bag" != True)):
    break_count = break_count + 1
    continue

   b = bagreader(buildingEditorModels + "/" + folder + "/sensors/" + str(i) + "/front_scan.bag")

   LASER_MSG = b.message_by_topic("/front/scan")

   df_laser=pd.read_csv(LASER_MSG)
   binData = np.array([ df_laser["ranges_" + str(i)] for i in range(3601) ])

   lidarBinDataset = binData.transpose()

   o = bagreader(buildingEditorModels +"/" + folder + '/sensors/' + str(i) +'/odometry.bag')

   ODOMETRY_MSG = o.message_by_topic('/ground_truth/state')

   df_odometry=pd.read_csv(ODOMETRY_MSG)

   odometryPosData=np.vstack((df_odometry["pose.pose.position.x"] , df_odometry["pose.pose.position.y"], df_odometry["pose.pose.orientation.z"]) )
   odometryPosData=odometryPosData.transpose()

   dataset_translation = np.sqrt((odometryPosData[1:,0] - odometryPosData[:-1,0]) ** 2 + (odometryPosData[1:,1] - odometryPosData[:-1,1]) ** 2 )
   dataset_rotation = odometryPosData[1:,2] - odometryPosData[:-1,2]

   label = np.array([dataset_translation, dataset_rotation])
   label = label.transpose()

  #  lidarBinDataset=lidarBinDataset[1:]
  #  label=label[1:]
   binDataset = np.array({'lidarDataset':lidarBinDataset,'labelDataset': label})

   if break_count==5:
    break
   #setfacl -R -m u:username:rwx myfolder
   np.save(arr=binDataset, file="/home/ftpuser/ftp/files/dataset/"+str(j)+str(i))

 
print("duvar " + str(j+1) + " de atma işlemi bitirildi.")

wall_idx = j


for i in range(1,wall_idx):
  shutil.rmtree(buildingEditorModels + "/wall" + str(i) )

if break_count == 5:
  for i, file in enumerate(os.listdir(buildingEditorModels), start=1):
    os.rename(buildingEditorModels + "/" + file, buildingEditorModels + "/wall"+str(i) )

    model_config_tree = ET.parse(buildingEditorModels + "/wall" + str(i) + "/model.config")
    model_sdf_tree = ET.parse(buildingEditorModels + "/wall" + str(i) + "/model.sdf")

    model_config_root = model_config_tree.getroot()
    model_sdf_root = model_sdf_tree.getroot()

    model_config_root[0].text = 'wall' + str(i)
    model_sdf_root[0].attrib['name'] = 'walls' + str(i)

    model_sdf_tree.write(buildingEditorModels + "/wall" + str(i) + "/model.sdf")
    model_config_tree.write(buildingEditorModels + "/wall" + str(i) + "/model.config")