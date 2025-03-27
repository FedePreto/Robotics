%% Exploring TFs

exampleHelperROSStartTfPublisher

%% Extracting info from TFs

tftree = rostf;
tftree.AvailableFrames

mountTocamera = getTransform(tftree,'mounting_point','camera_center');

transl = mountTocamera.Transform.Translation %The RF of the camera is 0.5m above the RF of the mount 
quat = mountTocamera.Transform.Rotation

rot_angles = rad2deg(quat2eul([quat.W quat.X quat.Y quat.Z],"XYZ")) %Those angles are respect to the mounting point