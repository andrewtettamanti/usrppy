#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as n
import uhd
import scipy.signal as ss
import time
import matplotlib.pyplot as plt
import h5py
import rospy
from geometry_msgs.msg import PoseStamped

pose_x = None
pose_y = None


# ROS Stuff
def callback(data):
    print("hello from ros callback")
    pose_x = data.pose.position.x
    pose_y = data.pose.position.y
    print("pose_x", pose_x)

TOPIC="/car/car_pose"
rospy.init_node('listener', anonymous=True)
#rospy.Subscriber(TOPIC, PoseStamped, callback)

def acquire_spectrum(freq=770e6,
                     sample_rate=50e6,
                     n_fft=1024,
                     n_avg=64,
                     n_t=5000,
                     subdev="A:A",
                     ofname="spectrogram1.h5"):

    usrp = uhd.usrp.MultiUSRP("recv_buff_size=500000000")
    subdev_spec=uhd.usrp.SubdevSpec(subdev)
    usrp.set_rx_subdev_spec(subdev_spec)


    message = rospy.wait_for_message(TOPIC, PoseStamped)
    w=ss.blackmanharris(n_fft)
    freqv=n.fft.fftshift(n.fft.fftfreq(n_fft, d=1.0/50e6))+freq
    S=n.zeros([n_t,n_fft])
    pose=n.zeros([[10],[10]])

    for ti in range(n_t):
        
        samps = usrp.recv_num_samps(n_fft*n_avg, freq, 50000000, [0], 0)

        
        pose[[ti][ti]] = ([[message.pose.position.x], [message.pose.position.y]])

        #tvec.append(time.time())#change to append position
        
        #check samps
        if len(samps[0]) != n_fft*n_avg:
            print("error")
        else:
            z=samps[0]
            z=z-n.mean(z) # change of mean
            
            for ai in range(n_avg):
                S[ti,:]+=n.abs(n.fft.fftshift(
                n.fft.fft(z[(ai*n_fft):(ai*n_fft + n_fft)]*w)))**2.0
            
            n.append(pose,S)

    n.save('usrp.npy',pose) 


    plt.ion()
    fig = plt.figure()
    ax=fig.add_subplot(111)
    ax.clear()
    ax.pcolormesh(freqv/1e6,tvec, 10.0*n.log10(S))
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Time (Sec)")
    fig.canvas.draw()
    fig.canvas.flush_events()
 
if __name__ == "__main__":

    try:
        while True:
            callback()
            acquire_spectrum()
    except KeyboardInterrupt:
        pass

