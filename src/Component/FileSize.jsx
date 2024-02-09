import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';

const RosbagSize = () => {
  const [ros] = useState(new ROSLIB.Ros({ url: 'ws://localhost:9090' }));
  const [rosbagSize, setRosbagSize] = useState('0 GB');

  useEffect(() => {
    const sizeListener = new ROSLIB.Topic({
      ros: ros,
      name: '/rosbag_size',
      messageType: 'std_msgs/String'
    });

    sizeListener.subscribe((message) => {
      setRosbagSize(message.data);
    });

    return () => {
      sizeListener.unsubscribe();
    };
  }, [ros]);

  return (
    <p>{rosbagSize}</p>
  );
};

export default RosbagSize;
