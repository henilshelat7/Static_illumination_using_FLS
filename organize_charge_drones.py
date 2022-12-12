import airsim


def place_objects(client):
    left_bottom = client.simSetObjectPose('leftbottomcylinder', airsim.Pose(airsim.Vector3r(25, 30, 0)))
    right_bottom = client.simSetObjectPose('rightbottomcylinder',airsim.Pose(airsim.Vector3r(0, 30, 0)))
    left_top = client.simSetObjectPose('leftopcylinder',airsim.Pose(airsim.Vector3r(25, 0, 0)))
    right_top = client.simSetObjectPose('rightopcylinder',airsim.Pose(airsim.Vector3r(0, 0, 0)))


    chargingstation = client.simSetObjectPose('ChargingStation1',airsim.Pose(airsim.Vector3r(12.5, 36, 0)))
    chargingstation2 = client.simSetObjectPose('ChargingStation2',airsim.Pose(airsim.Vector3r(12.5, -6, 0)))
    belt = client.simSetObjectPose('M_Conveyor_BP_2', airsim.Pose(airsim.Vector3r(12.5,15,0)))

client = airsim.MultirotorClient()
client.enableApiControl(True)
place_objects(client)


pose = client.simGetObjectPose('ChargingStation1')
scale = client.simGetObjectScale('ChargingStation2')
pose.position.z_val -= (scale.z_val -  4.4)
client.simAddVehicle('FLS_Drone1','simpleflight',pose,'FLSDronePawn')