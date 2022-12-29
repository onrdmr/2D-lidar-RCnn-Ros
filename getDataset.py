import ftplib
import os

localfilepath = r"D:\Ara\numpy_dataset_sequences_gazebo"

ftp = ftplib.FTP("192.168.1.105")
ftp.login("ftpuser", "425262026Asd")


ftp.cwd("/files/dataset")

folder = sorted(os.listdir(localfilepath), key = lambda x : int(x.split('_')[0]), reverse=True )
if(len(folder) == 0):
    lastFolderIdx = 0
else:    
    lastFolderIdx = int(folder[0].split('_')[0]);
for i,file in enumerate(ftp.nlst()):
    i = lastFolderIdx + i + 1
    ftp.retrbinary('RETR %s'%file, open(localfilepath + "\{i}_angleBin.npy".format(i=i), 'wb').write)
    ftp.delete(file)

ftp.quit()