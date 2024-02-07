import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';

const RosbagStatus = () => {
  const [ros] = useState(new ROSLIB.Ros({ url: 'ws://localhost:9090' }));
  const [rosbagStatus, setRosbagStatus] = useState('no');

  useEffect(() => {
    const statusListener = new ROSLIB.Topic({
      ros: ros,
      name: '/rosbag_status',
      messageType: 'std_msgs/String'
    });

    statusListener.subscribe((message) => {
      setRosbagStatus(message.data);
    });

    return () => {
      statusListener.unsubscribe();
    };
  }, [ros]);

  return (
    <div>
      <h2>:  {rosbagStatus}</h2>
    </div>
  );
};

export default RosbagStatus;
